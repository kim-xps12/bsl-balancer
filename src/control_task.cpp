#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include "config.h"
#include "shared_state.h"
#include "imu_driver.h"

extern Dynamixel2Arduino dxl;
extern ImuDriver imuDriver;

static void driveTire(int rpm) {
    dxl.setGoalVelocity(DXL_ID_L, -1 * rpm, UNIT_RPM);
    dxl.setGoalVelocity(DXL_ID_R, rpm, UNIT_RPM);
}

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

        CtrlCommand cmd;
        while (xQueueReceive(g_cmd_queue, &cmd, 0) == pdTRUE) {
            switch (cmd.type) {
                case CtrlCommandType::SET_KP:           kp = cmd.value; break;
                case CtrlCommandType::SET_KI:           ki = cmd.value; break;
                case CtrlCommandType::SET_KD:           kd = cmd.value; break;
                case CtrlCommandType::SET_PITCH_TARGET: target = cmd.value; break;
            }
        }

        float dt;
        if (prev_us == 0) {
            dt = 0.005f;
        } else {
            dt = (float)(now_us - prev_us) / 1000000.0f;
            dt = constrain(dt, 0.002f, 0.050f);
        }
        prev_us = now_us;

        auto imu = imuDriver.update(dt);
        if (!imu.valid) {
            vTaskDelayUntil(&xLastWakeTime, CTRL_PERIOD_TICKS);
            continue;
        }

        float pitch_filtered = imu.pitch_deg;
        float pitch_error = target - pitch_filtered;

        if (pitch_error < -FALL_THRESHOLD_DEG || FALL_THRESHOLD_DEG < pitch_error) {
            driveTire(0);
            I_acc = 0.0f;
            preP = 0.0f;
            D_filtered = 0.0f;
            pid_active = false;
            TelemetryData tel = {pitch_filtered, imu.pitch_rate_dps, 0, 0, 0, 0,
                                 (uint32_t)(esp_timer_get_time() - now_us), true};
            xQueueOverwrite(g_telemetry_queue, &tel);
            vTaskDelayUntil(&xLastWakeTime, CTRL_PERIOD_TICKS);
            continue;
        }

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
        D_filtered = 0.2f * D_raw + 0.8f * D_filtered;
        float D_val = D_filtered;
        preP = P_val;

        float rpm_f = kp * P_val + ki * I_acc + kd * D_val;
        int rpm_motor = constrain((int)rpm_f, -RPM_LIMIT, RPM_LIMIT);
        driveTire(rpm_motor);

        uint32_t elapsed = (uint32_t)(esp_timer_get_time() - now_us);
        TelemetryData tel = {pitch_filtered, imu.pitch_rate_dps, (float)rpm_motor,
                             P_val, I_acc, D_val, elapsed, false};
        xQueueOverwrite(g_telemetry_queue, &tel);

        vTaskDelayUntil(&xLastWakeTime, CTRL_PERIOD_TICKS);
    }
}
