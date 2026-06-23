#include <Arduino.h>
#include <M5Unified.h>
#include <Dynamixel2Arduino.h>
#include <esp_task_wdt.h>
#include <cstring>
#include "config.h"
#include "shared_state.h"
#include "imu_driver.h"

Dynamixel2Arduino dxl;
ImuDriver imuDriver;

QueueHandle_t g_cmd_queue = nullptr;
QueueHandle_t g_telemetry_queue = nullptr;
SemaphoreHandle_t g_i2c_mutex = nullptr;

extern void controlLoopTask(void *pvParameters);
extern void uiLoopTask(void *pvParameters);

static int32_t dxlReadU8(uint8_t id, uint16_t addr) {
    uint8_t buf[1];
    int32_t n = dxl.read(id, addr, 1, buf, sizeof(buf));
    if (n < 0) return -1;
    return (int32_t)buf[0];
}

static int32_t dxlReadU16(uint8_t id, uint16_t addr) {
    uint8_t buf[2];
    int32_t n = dxl.read(id, addr, 2, buf, sizeof(buf));
    if (n < 0) return -1;
    return (int32_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

static int32_t dxlReadI16(uint8_t id, uint16_t addr) {
    uint8_t buf[2];
    int32_t n = dxl.read(id, addr, 2, buf, sizeof(buf));
    if (n < 0) return -1;
    uint16_t u = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    int16_t s;
    memcpy(&s, &u, sizeof(s));
    return (int32_t)s;
}

static bool dxlWriteU8(uint8_t id, uint16_t addr, uint8_t val) {
    return dxl.write(id, addr, &val, 1);
}

static bool dxlWriteU16(uint8_t id, uint16_t addr, uint16_t val) {
    uint8_t buf[2] = { (uint8_t)(val & 0xFF), (uint8_t)(val >> 8) };
    return dxl.write(id, addr, buf, 2);
}

static bool dxlWriteI16(uint8_t id, uint16_t addr, int16_t val) {
    uint8_t buf[2];
    memcpy(buf, &val, 2);
    return dxl.write(id, addr, buf, 2);
}

static bool writeIfDifferentU8(uint8_t id, uint16_t addr, uint8_t desired, const char* name) {
    int32_t current = dxlReadU8(id, addr);
    if (current < 0) {
        Serial.printf("DXL ID%d %s read failed\n", id, name);
        return false;
    }
    if ((uint8_t)current == desired) return true;
    if (!dxlWriteU8(id, addr, desired)) {
        Serial.printf("DXL ID%d %s write failed\n", id, name);
        return false;
    }
    int32_t verify = dxlReadU8(id, addr);
    if (verify < 0 || (uint8_t)verify != desired) {
        Serial.printf("DXL ID%d %s verify failed: got %d, expected %d\n",
                      id, name, (int)verify, (int)desired);
        return false;
    }
    return true;
}

static bool writeIfDifferentU16(uint8_t id, uint16_t addr, uint16_t desired, const char* name) {
    int32_t current = dxlReadU16(id, addr);
    if (current < 0) {
        Serial.printf("DXL ID%d %s read failed\n", id, name);
        return false;
    }
    if ((uint16_t)current == desired) return true;
    if (!dxlWriteU16(id, addr, desired)) {
        Serial.printf("DXL ID%d %s write failed\n", id, name);
        return false;
    }
    int32_t verify = dxlReadU16(id, addr);
    if (verify < 0 || (uint16_t)verify != desired) {
        Serial.printf("DXL ID%d %s verify failed: got %d, expected %d\n",
                      id, name, (int)verify, (int)desired);
        return false;
    }
    return true;
}

static bool initMotor(uint8_t id) {
    // Torque OFF first (may still be armed from previous boot, attempt even before ping)
    dxl.torqueOff(id);

    if (!dxl.ping(id)) {
        Serial.printf("DXL ID%d: PING failed\n", id);
        return false;
    }

    uint16_t model = dxl.getModelNumber(id);
    if (model != DXL_EXPECTED_MODEL_NUMBER) {
        Serial.printf("DXL ID%d: model %d != expected %d\n", id, model, DXL_EXPECTED_MODEL_NUMBER);
        return false;
    }

    // Clear stale Bus Watchdog (may be -1 after ESP reboot with XL330 still powered)
    dxlWriteU8(id, DXL_ADDR_BUS_WATCHDOG, 0);

    if (!writeIfDifferentU8(id, DXL_ADDR_OPERATING_MODE, DXL_OP_MODE_CURRENT, "OperatingMode"))
        return false;

    uint16_t current_limit_raw = (uint16_t)(EEPROM_CURRENT_LIMIT_A / DXL_CURRENT_UNIT_A);
    if (!writeIfDifferentU16(id, DXL_ADDR_CURRENT_LIMIT, current_limit_raw, "CurrentLimit"))
        return false;

    if (!writeIfDifferentU8(id, DXL_ADDR_RETURN_DELAY, 0, "ReturnDelay"))
        return false;

    // Verify settings
    int32_t v_mode = dxlReadU8(id, DXL_ADDR_OPERATING_MODE);
    int32_t v_clim = dxlReadU16(id, DXL_ADDR_CURRENT_LIMIT);
    if (v_mode != DXL_OP_MODE_CURRENT || v_clim != (int32_t)current_limit_raw) {
        Serial.printf("DXL ID%d: verify failed (mode=%d, clim=%d)\n", id, (int)v_mode, (int)v_clim);
        return false;
    }

    // Goal Current = 0
    if (!dxlWriteI16(id, DXL_ADDR_GOAL_CURRENT, 0)) {
        Serial.printf("DXL ID%d: Goal Current write failed\n", id);
        return false;
    }

    // Readback Goal Current
    int32_t gc = dxlReadI16(id, DXL_ADDR_GOAL_CURRENT);
    if (gc != 0) {
        Serial.printf("DXL ID%d: Goal Current readback = %d (expected 0)\n", id, (int)gc);
        return false;
    }

    Serial.printf("DXL ID%d: init OK (CurrentControl, limit=%dmA)\n",
                  id, (int)current_limit_raw);
    return true;
}

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

    g_i2c_mutex = xSemaphoreCreateMutex();
    configASSERT(g_i2c_mutex != nullptr);

    g_cmd_queue = xQueueCreate(8, sizeof(CtrlCommand));
    g_telemetry_queue = xQueueCreate(1, sizeof(TelemetryData));
    configASSERT(g_cmd_queue != nullptr);
    configASSERT(g_telemetry_queue != nullptr);

    HardwareSerial& dxl_serial = Serial1;
    dxl_serial.begin(DXL_BAUDRATE, SERIAL_8N1, PIN_RX_SERVO, PIN_TX_SERVO);
    dxl = Dynamixel2Arduino(dxl_serial);
    dxl.begin(DXL_BAUDRATE);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    bool init_L = initMotor(DXL_ID_L);
    bool init_R = initMotor(DXL_ID_R);
    bool init_ok = init_L && init_R;

    if (!init_ok) {
        M5.Lcd.println("DXL INIT FAIL");
        Serial.println("DXL init failed - staying DISARMED");
    }

    imuDriver.begin(g_i2c_mutex);
    M5.Lcd.println("Calibrating...");
    imuDriver.calibrate(500, 0.5f);

    bool armed = false;
    if (init_ok && SYSID_CONFIGURED) {
        dxl.torqueOn(DXL_ID_L);
        dxl.torqueOn(DXL_ID_R);

        dxlWriteI16(DXL_ID_L, DXL_ADDR_GOAL_CURRENT, 0);
        dxlWriteI16(DXL_ID_R, DXL_ADDR_GOAL_CURRENT, 0);

        uint8_t wd_raw = (uint8_t)((BUS_WATCHDOG_TIMEOUT_MS + 19) / 20);
        if (wd_raw < 1) wd_raw = 1;
        if (wd_raw > 127) wd_raw = 127;
        bool wd_ok = dxlWriteU8(DXL_ID_L, DXL_ADDR_BUS_WATCHDOG, wd_raw)
                  && dxlWriteU8(DXL_ID_R, DXL_ADDR_BUS_WATCHDOG, wd_raw);

        if (wd_ok) {
            armed = true;
            M5.Lcd.println("Armed!");
            Serial.printf("CTRL_UNIT=%d FIT_INPUT=%d\n", CTRL_UNIT, FIT_INPUT_SOURCE);
        } else {
            Serial.println("Bus Watchdog setup failed - disarming");
            dxl.torqueOff(DXL_ID_L);
            dxl.torqueOff(DXL_ID_R);
            M5.Lcd.println("WD FAIL");
        }
    } else {
        if (!SYSID_CONFIGURED) {
            M5.Lcd.println("BLOCKED: SYSID");
            Serial.println("BLOCKED: CTRL_UNIT or FIT_INPUT_SOURCE not configured");
        } else {
            M5.Lcd.println("DISARMED");
        }
    }

    esp_task_wdt_init(1, true);

    const uint32_t STACK_SIZE = 8192;
    xTaskCreatePinnedToCore(controlLoopTask, "Control", STACK_SIZE,
                            (void*)(uintptr_t)armed, 20, NULL, 1);
    xTaskCreatePinnedToCore(uiLoopTask, "UI", STACK_SIZE, NULL, 3, NULL, 0);
}

void loop() {
}
