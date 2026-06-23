#include <Arduino.h>
#include <M5Unified.h>
#include <Dynamixel2Arduino.h>
#include <esp_task_wdt.h>
#include "config.h"
#include "shared_state.h"
#include "imu_driver.h"

// Global instances
Dynamixel2Arduino dxl;
ImuDriver imuDriver;

QueueHandle_t g_cmd_queue = nullptr;
QueueHandle_t g_telemetry_queue = nullptr;
SemaphoreHandle_t g_i2c_mutex = nullptr;

// Task entry points (defined in control_task.cpp / ui_task.cpp)
extern void controlLoopTask(void *pvParameters);
extern void uiLoopTask(void *pvParameters);

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

    // DYNAMIXEL
    HardwareSerial& dxl_serial = Serial1;
    dxl_serial.begin(DXL_BAUDRATE, SERIAL_8N1, PIN_RX_SERVO, PIN_TX_SERVO);
    dxl = Dynamixel2Arduino(dxl_serial);
    dxl.begin(DXL_BAUDRATE);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    if (!dxl.ping(DXL_ID_L)) { M5.Lcd.println("DXL L ping FAIL"); }
    if (!dxl.ping(DXL_ID_R)) { M5.Lcd.println("DXL R ping FAIL"); }
    dxl.torqueOff(DXL_ID_L);
    dxl.torqueOff(DXL_ID_R);
    dxl.setOperatingMode(DXL_ID_L, OP_VELOCITY);
    dxl.setOperatingMode(DXL_ID_R, OP_VELOCITY);
    dxl.torqueOn(DXL_ID_L);
    dxl.torqueOn(DXL_ID_R);

    // IMU
    imuDriver.begin(g_i2c_mutex);
    M5.Lcd.println("Calibrating...");
    imuDriver.calibrate(500, 0.5f);
    M5.Lcd.println("Ready!");

    // WDT
    esp_task_wdt_init(1, true);

    // RTOS Tasks
    const uint32_t STACK_SIZE = 8192;
    xTaskCreatePinnedToCore(controlLoopTask, "Control", STACK_SIZE, NULL, 20, NULL, 1);
    xTaskCreatePinnedToCore(uiLoopTask,      "UI",      STACK_SIZE, NULL, 3,  NULL, 0);
}

void loop() {
}
