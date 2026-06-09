#include <Arduino.h>

#include <M5Unified.h>

#include <Preferences.h>
#include <Dynamixel2Arduino.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include "imu_driver.h"

HardwareSerial& DXL_SERIAL = Serial1;

// M5Stack Core2 PORT A pin configuration
const uint8_t PIN_RX_SERVO = 33;
const uint8_t PIN_TX_SERVO = 32;

// Default controller params
const float DEFAULT_KP = 15.0f;
const float DEFAULT_KI = 0.05f;
const float DEFAULT_KD = 0.0f;
const float DEFAULT_PITCH_TARGET = 86.0f;
const TickType_t xPeriodMs = 10;

// DYNAMIXEL Params.
const uint8_t DXL_ID_L = 0;
const uint8_t DXL_ID_R = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl;
ImuDriver imuDriver;

// I2C bus mutex
SemaphoreHandle_t g_i2c_mutex = nullptr;

// --- Inter-task communication ---

enum class CtrlCommandType : uint8_t {
    SET_PITCH_TARGET, SET_KP, SET_KI, SET_KD,
};
struct CtrlCommand { CtrlCommandType type; float value; };

struct TelemetryData {
    float pitch_deg;
    float pitch_rate_dps;
    float rpm_cmd;
    float P_term, I_term, D_term;
    uint32_t loop_us;
    bool fallen;
};

QueueHandle_t g_cmd_queue = nullptr;
QueueHandle_t g_telemetry_queue = nullptr;

// --- Motor drive ---

void driveTire(int rpm) {
    dxl.setGoalVelocity(DXL_ID_L, -1 * rpm, UNIT_RPM);
    dxl.setGoalVelocity(DXL_ID_R, rpm, UNIT_RPM);
}

// --- Control task ---

void controlLoopTask(void *pvParameters) {
    esp_task_wdt_add(NULL);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    float kp = DEFAULT_KP;
    float ki = DEFAULT_KI;
    float kd = DEFAULT_KD;
    float target = DEFAULT_PITCH_TARGET;
    float I_acc = 0.0f;
    float preP = 0.0f;
    float D_filtered = 0.0f;
    bool pid_active = false;
    int64_t prev_us = 0;

    while (true) {
        esp_task_wdt_reset();
        int64_t now_us = esp_timer_get_time();

        // Drain pending commands (non-blocking)
        CtrlCommand cmd;
        while (xQueueReceive(g_cmd_queue, &cmd, 0) == pdTRUE) {
            switch (cmd.type) {
                case CtrlCommandType::SET_KP:           kp = cmd.value; break;
                case CtrlCommandType::SET_KI:           ki = cmd.value; break;
                case CtrlCommandType::SET_KD:           kd = cmd.value; break;
                case CtrlCommandType::SET_PITCH_TARGET: target = cmd.value; break;
            }
        }

        // Compute dt from actual elapsed time
        float dt;
        if (prev_us == 0) {
            dt = 0.010f;
        } else {
            dt = (float)(now_us - prev_us) / 1000000.0f;
            dt = constrain(dt, 0.002f, 0.050f);
        }
        prev_us = now_us;

        // IMU read + Mahony fusion
        auto imu = imuDriver.update(dt);
        if (!imu.valid) {
            vTaskDelayUntil(&xLastWakeTime, xPeriodMs);
            continue;
        }

        float pitch_filtered = imu.pitch_deg;

        float pitch_error = target - pitch_filtered;

        // Fall detection
        if (pitch_error < -40.0f || 40.0f < pitch_error) {
            driveTire(0);
            I_acc = 0.0f;
            preP = 0.0f;
            D_filtered = 0.0f;
            pid_active = false;
            TelemetryData tel = {pitch_filtered, imu.pitch_rate_dps, 0, 0, 0, 0,
                                 (uint32_t)(esp_timer_get_time() - now_us), true};
            xQueueOverwrite(g_telemetry_queue, &tel);
            vTaskDelayUntil(&xLastWakeTime, xPeriodMs);
            continue;
        }

        // PID
        float P_val = pitch_error;
        I_acc += P_val * dt;
        float i_limit = 50.0f / (fabsf(ki) + 1e-6f);
        I_acc = constrain(I_acc, -i_limit, i_limit);

        float D_raw;
        if (!pid_active) {
            D_raw = 0.0f;
            pid_active = true;
        } else {
            D_raw = (P_val - preP) / dt;
        }
        // Low-pass filter on D term (alpha=0.2: heavy smoothing)
        D_filtered = 0.2f * D_raw + 0.8f * D_filtered;
        float D_val = D_filtered;
        preP = P_val;

        float rpm_f = kp * P_val + ki * I_acc + kd * D_val;
        int rpm_motor = constrain((int)rpm_f, -300, 300);
        driveTire(rpm_motor);

        // Telemetry (non-blocking overwrite)
        uint32_t elapsed = (uint32_t)(esp_timer_get_time() - now_us);
        TelemetryData tel = {pitch_filtered, imu.pitch_rate_dps, (float)rpm_motor,
                             P_val, I_acc, D_val, elapsed, false};
        xQueueOverwrite(g_telemetry_queue, &tel);

        vTaskDelayUntil(&xLastWakeTime, xPeriodMs);
    }
}

// --- UI helpers ---

void drawButton(const char* label, int x, int y, float value) {
    M5.Display.drawRect(x, y, 50, 30, WHITE);
    M5.Display.drawString("-", x + 20, y, 2);
    M5.Display.drawRect(x + 200, y, 50, 30, WHITE);
    M5.Display.drawString("+", x + 200 + 20, y, 2);
    M5.Display.setCursor(x + 70, y + 5);
    M5.Display.print(label);
    M5.Display.print(": ");
    M5.Display.println(value, 1);
}

// --- UI task ---

static float ui_kp = DEFAULT_KP;
static float ui_ki = DEFAULT_KI;
static float ui_kd = DEFAULT_KD;
static float ui_target = DEFAULT_PITCH_TARGET;
bool enShowCtrlPanel = false;

void drawCtrlPanel() {
    drawButton("Ref", 20, 30, ui_target);
    drawButton("Kp", 20, 70, ui_kp);
    drawButton("Ki", 20, 110, ui_ki);
    drawButton("Kd", 20, 150, ui_kd);

    if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        int batteryPercentage = M5.Power.getBatteryLevel();
        xSemaphoreGive(g_i2c_mutex);
        M5.Display.setCursor(20, 190);
        M5.Display.printf("M5Core2 Battery: %d%%", batteryPercentage);
    }
}

void handleCtrlPanelTouched() {
    auto pos = M5.Touch.getDetail();
    if (!pos.wasPressed()) return;

    CtrlCommand cmd;
    bool send = true;

    if (pos.y >= 30 && pos.y < 60) {
        if (pos.x >= 15 && pos.x < 120)      ui_target -= 0.2f;
        else if (pos.x >= 220 && pos.x < 270) ui_target += 0.2f;
        else send = false;
        if (send) { cmd = {CtrlCommandType::SET_PITCH_TARGET, ui_target}; drawButton("Ref", 20, 30, ui_target); }
    }
    else if (pos.y >= 70 && pos.y < 100) {
        if (pos.x >= 15 && pos.x < 120)      ui_kp -= 0.2f;
        else if (pos.x >= 220 && pos.x < 270) ui_kp += 0.2f;
        else send = false;
        if (send) { cmd = {CtrlCommandType::SET_KP, ui_kp}; drawButton("Kp", 20, 70, ui_kp); }
    }
    else if (pos.y >= 110 && pos.y < 140) {
        if (pos.x >= 15 && pos.x < 120)      ui_ki -= 0.2f;
        else if (pos.x >= 220 && pos.x < 270) ui_ki += 0.2f;
        else send = false;
        if (send) { cmd = {CtrlCommandType::SET_KI, ui_ki}; drawButton("Ki", 20, 110, ui_ki); }
    }
    else if (pos.y >= 150 && pos.y < 180) {
        if (pos.x >= 15 && pos.x < 120)      ui_kd -= 0.2f;
        else if (pos.x >= 220 && pos.x < 270) ui_kd += 0.2f;
        else send = false;
        if (send) { cmd = {CtrlCommandType::SET_KD, ui_kd}; drawButton("Kd", 20, 150, ui_kd); }
    }
    else {
        send = false;
    }

    if (send) xQueueSend(g_cmd_queue, &cmd, 0);
}

void uiLoopTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t last_telemetry_ms = 0;

    while (true) {
        if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            M5.update();
            xSemaphoreGive(g_i2c_mutex);
        }

        // Toggle tuning panel
        if (M5.BtnB.wasPressed()) {
            enShowCtrlPanel = !enShowCtrlPanel;
            M5.Lcd.clear();
            if (enShowCtrlPanel) drawCtrlPanel();
        }

        if (enShowCtrlPanel) {
            handleCtrlPanelTouched();
        }

        // Telemetry output via Serial (10Hz)
        uint32_t now_ms = millis();
        TelemetryData tel;
        if (now_ms - last_telemetry_ms >= 100) {
            if (xQueuePeek(g_telemetry_queue, &tel, 0) == pdTRUE) {
                last_telemetry_ms = now_ms;
                Serial.printf(">pitch:%.2f\n>rate:%.2f\n>rpm:%.1f\n>P:%.2f\n>I:%.3f\n>D:%.2f\n>loop_us:%u\n",
                              tel.pitch_deg, tel.pitch_rate_dps, tel.rpm_cmd,
                              tel.P_term, tel.I_term, tel.D_term, tel.loop_us);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

// --- Setup ---

void setup() {
    Serial.begin(115200);

    auto cfg = M5.config();
    cfg.internal_rtc = false;
    cfg.internal_mic = false;
    cfg.internal_spk = false;
    cfg.internal_imu = true;
    M5.begin(cfg);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(0, 0);

    // I2C mutex
    g_i2c_mutex = xSemaphoreCreateMutex();
    configASSERT(g_i2c_mutex != nullptr);

    // Inter-task queues
    g_cmd_queue = xQueueCreate(8, sizeof(CtrlCommand));
    g_telemetry_queue = xQueueCreate(1, sizeof(TelemetryData));
    configASSERT(g_cmd_queue != nullptr);
    configASSERT(g_telemetry_queue != nullptr);

    // DYNAMIXEL Settings
    DXL_SERIAL.begin(1000000, SERIAL_8N1, PIN_RX_SERVO, PIN_TX_SERVO);
    dxl = Dynamixel2Arduino(DXL_SERIAL);
    dxl.begin(1000000);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    if (!dxl.ping(DXL_ID_L)) { M5.Lcd.println("DXL L ping FAIL"); }
    if (!dxl.ping(DXL_ID_R)) { M5.Lcd.println("DXL R ping FAIL"); }
    dxl.torqueOff(DXL_ID_L);
    dxl.torqueOff(DXL_ID_R);
    dxl.setOperatingMode(DXL_ID_L, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID_R, OP_VELOCITY);
    dxl.torqueOn(DXL_ID_L);
    dxl.torqueOn(DXL_ID_R);

    // IMU driver init + calibration
    imuDriver.begin(g_i2c_mutex);
    M5.Lcd.println("Calibrating...");
    imuDriver.calibrate(500, 0.5f);
    M5.Lcd.println("Ready!");

    // WDT
    esp_task_wdt_init(1, true);

    // RTOS Tasks
    const uint32_t MEMORY_STACK = 8192;
    xTaskCreatePinnedToCore(controlLoopTask, "Control", MEMORY_STACK, NULL, 20, NULL, 1);
    xTaskCreatePinnedToCore(uiLoopTask,      "UI",      MEMORY_STACK, NULL, 3,  NULL, 0);
}

void loop() {
}
