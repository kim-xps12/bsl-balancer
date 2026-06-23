#include <Arduino.h>
#include <M5Unified.h>
#include "config.h"
#include "shared_state.h"

static float ui_kp = DEFAULT_KP;
static float ui_ki = DEFAULT_KI;
static float ui_kd = DEFAULT_KD;
static float ui_target = DEFAULT_PITCH_TARGET;
static bool enShowCtrlPanel = false;
static bool ui_speed_enabled = false;
static bool tel_speed_prev = false;
static uint8_t speed_confirm_wait = 0;

static const char* stateStr(SafetyState s) {
    switch (s) {
        case SafetyState::DISARMED:     return "DISARMED";
        case SafetyState::INITIALIZING: return "INIT";
        case SafetyState::ARMED_IDLE:   return "IDLE";
        case SafetyState::BALANCING:    return "BALANCE";
        case SafetyState::FAULT:        return "FAULT";
        case SafetyState::SHUTDOWN:     return "SHUTDOWN";
        default:                        return "???";
    }
}

static void drawButton(const char* label, int x, int y, float value) {
    M5.Display.drawRect(x, y, 50, 30, WHITE);
    M5.Display.drawString("-", x + 20, y, 2);
    M5.Display.drawRect(x + 200, y, 50, 30, WHITE);
    M5.Display.drawString("+", x + 200 + 20, y, 2);
    M5.Display.setCursor(x + 70, y + 5);
    M5.Display.print(label);
    M5.Display.print(": ");
    M5.Display.println(value, 2);
}

static void drawCtrlPanel() {
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

    M5.Display.setCursor(20, 210);
    M5.Display.printf("Speed: %s  ", ui_speed_enabled ? "ON " : "OFF");
}

static void handleCtrlPanelTouched() {
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
        if (pos.x >= 15 && pos.x < 120)      ui_kp -= 0.01f;
        else if (pos.x >= 220 && pos.x < 270) ui_kp += 0.01f;
        else send = false;
        if (send) { cmd = {CtrlCommandType::SET_KP, ui_kp}; drawButton("Kp", 20, 70, ui_kp); }
    }
    else if (pos.y >= 110 && pos.y < 140) {
        if (pos.x >= 15 && pos.x < 120)      ui_ki -= 0.01f;
        else if (pos.x >= 220 && pos.x < 270) ui_ki += 0.01f;
        else send = false;
        if (send) { cmd = {CtrlCommandType::SET_KI, ui_ki}; drawButton("Ki", 20, 110, ui_ki); }
    }
    else if (pos.y >= 150 && pos.y < 180) {
        if (pos.x >= 15 && pos.x < 120)      ui_kd -= 0.01f;
        else if (pos.x >= 220 && pos.x < 270) ui_kd += 0.01f;
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

        if (M5.BtnB.wasClicked()) {
            enShowCtrlPanel = !enShowCtrlPanel;
            M5.Lcd.clear();
            if (enShowCtrlPanel) drawCtrlPanel();
        }

        if (M5.BtnB.wasHold()) {
            ui_speed_enabled = !ui_speed_enabled;
            tel_speed_prev = false;
            CtrlCommand cmd = {CtrlCommandType::SET_SPEED_ENABLED, ui_speed_enabled ? 1.0f : 0.0f};
            xQueueSend(g_cmd_queue, &cmd, 0);
            if (enShowCtrlPanel) {
                M5.Display.setCursor(20, 210);
                M5.Display.printf("Speed: %s  ", ui_speed_enabled ? "ON " : "OFF");
            }
        }

        if (enShowCtrlPanel) {
            handleCtrlPanelTouched();
        }

        {
            TelemetryData peek;
            if (xQueuePeek(g_telemetry_queue, &peek, 0) == pdTRUE) {
                if (peek.speed_enabled) {
                    tel_speed_prev = true;
                    speed_confirm_wait = 0;
                } else if (tel_speed_prev && ui_speed_enabled) {
                    ui_speed_enabled = false;
                    tel_speed_prev = false;
                    speed_confirm_wait = 0;
                    if (enShowCtrlPanel) {
                        M5.Display.setCursor(20, 210);
                        M5.Display.printf("Speed: OFF ");
                    }
                } else if (ui_speed_enabled && !tel_speed_prev) {
                    speed_confirm_wait++;
                    if (speed_confirm_wait >= 10) {
                        ui_speed_enabled = false;
                        speed_confirm_wait = 0;
                        if (enShowCtrlPanel) {
                            M5.Display.setCursor(20, 210);
                            M5.Display.printf("Speed: OFF ");
                        }
                    }
                }
            }
        }

        if (ui_speed_enabled) {
            CtrlCommand cmd_v = {CtrlCommandType::SET_VELOCITY_CMD, 0.0f};
            CtrlCommand cmd_y = {CtrlCommandType::SET_YAW_RATE_CMD, 0.0f};
            xQueueSend(g_cmd_queue, &cmd_v, 0);
            xQueueSend(g_cmd_queue, &cmd_y, 0);
        }

        uint32_t now_ms = millis();
        TelemetryData tel;
        if (now_ms - last_telemetry_ms >= 100) {
            if (xQueuePeek(g_telemetry_queue, &tel, 0) == pdTRUE) {
                last_telemetry_ms = now_ms;
                Serial.printf(">pitch:%.2f\n>rate:%.2f\n>iA:%.4f\n>P:%.3f\n>I:%.4f\n>D:%.3f\n>loop_us:%u\n"
                              ">v:%.4f\n>theta_off:%.2f\n>v_err:%.4f\n>v_int:%.4f\n>sr_us:%u\n>spd:%d\n"
                              ">state:%s\n>iL:%.3f\n>iR:%.3f\n>V:%.1f\n>T:%d\n",
                              tel.pitch_deg, tel.pitch_rate_dps, tel.current_cmd_A,
                              tel.P_term, tel.I_term, tel.D_term, tel.loop_us,
                              tel.v_measured, tel.theta_offset, tel.v_error, tel.v_integral,
                              tel.sync_read_us, tel.speed_enabled ? 1 : 0,
                              stateStr(tel.state),
                              tel.current_L_A, tel.current_R_A,
                              tel.voltage_V, (int)tel.temp_C);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}
