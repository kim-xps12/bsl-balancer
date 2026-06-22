#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <cstring>
#include <cmath>
#include "config.h"
#include "shared_state.h"
#include "imu_driver.h"

extern Dynamixel2Arduino dxl;
extern ImuDriver imuDriver;

// --- Sync Write for Goal Velocity (non-blocking) ---
static uint8_t goal_vel_buf_L[DXL_LEN_VELOCITY];
static uint8_t goal_vel_buf_R[DXL_LEN_VELOCITY];
static DYNAMIXEL::XELInfoSyncWrite_t vel_write_xels[2] = {
    {goal_vel_buf_L, DXL_ID_L},
    {goal_vel_buf_R, DXL_ID_R},
};
static DYNAMIXEL::InfoSyncWriteInst_t vel_sync_write = {
    DXL_ADDR_GOAL_VELOCITY,
    DXL_LEN_VELOCITY,
    vel_write_xels,
    2,
    true,
    {nullptr, 0, 0, false}
};

static void driveMotors(int rpm_L, int rpm_R) {
    int32_t raw_L = (int32_t)(-rpm_L / DXL_VEL_UNIT);
    int32_t raw_R = (int32_t)( rpm_R / DXL_VEL_UNIT);
    memcpy(goal_vel_buf_L, &raw_L, 4);
    memcpy(goal_vel_buf_R, &raw_R, 4);
    vel_sync_write.is_info_changed = true;
    dxl.syncWrite(&vel_sync_write);
}

// --- Sync Read for Present Velocity ---
static uint8_t vel_recv_buf_L[DXL_LEN_VELOCITY];
static uint8_t vel_recv_buf_R[DXL_LEN_VELOCITY];
static DYNAMIXEL::XELInfoSyncRead_t vel_read_xels[2] = {
    {vel_recv_buf_L, DXL_ID_L, 0},
    {vel_recv_buf_R, DXL_ID_R, 0},
};
static DYNAMIXEL::InfoSyncReadInst_t vel_sync_read = {
    DXL_ADDR_PRESENT_VELOCITY,
    DXL_LEN_VELOCITY,
    vel_read_xels,
    2,
    true,
    {nullptr, 0, 0, false}
};

static inline int32_t decodeInt32(const uint8_t* buf) {
    uint32_t u = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8)
              | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    int32_t s;
    memcpy(&s, &u, sizeof(s));
    return s;
}

struct VelReadResult {
    float v;
    uint32_t elapsed_us;
    bool valid;
};

static VelReadResult readWheelVelocity() {
    VelReadResult r = {0, 0, false};
    int64_t t0 = esp_timer_get_time();
    uint8_t recv_count = dxl.syncRead(&vel_sync_read, 2);
    r.elapsed_us = (uint32_t)(esp_timer_get_time() - t0);

    if (recv_count < 2) return r;

    int32_t raw_L = decodeInt32(vel_recv_buf_L);
    int32_t raw_R = decodeInt32(vel_recv_buf_R);
    float avg_raw = (float)(-raw_L + raw_R) * 0.5f;
    float avg_rpm = avg_raw * DXL_VEL_UNIT;
    float avg_omega = avg_rpm * (2.0f * M_PI / 60.0f);
    r.v = avg_omega * WHEEL_R;
    r.valid = true;
    return r;
}

// --- Control Loop ---
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

    // Speed control state
    bool speed_enabled = false;
    bool speed_was_enabled = false;
    float kp_speed = DEFAULT_KP_SPEED;
    float ki_speed = DEFAULT_KI_SPEED;
    float v_d = 0.0f;
    float yaw_rate_cmd = 0.0f;
    float v_integral = 0.0f;
    float theta_offset = 0.0f;
    float v_measured = 0.0f;
    float v_error_out = 0.0f;
    float v_d_prev = 0.0f;
    uint8_t speed_divider_count = 0;
    uint8_t sync_read_fail_count = 0;
    uint32_t last_cmd_ms = 0;
    uint32_t last_sync_read_us = 0;

    while (true) {
        esp_task_wdt_reset();
        int64_t now_us = esp_timer_get_time();

        // --- Command queue drain ---
        CtrlCommand cmd;
        while (xQueueReceive(g_cmd_queue, &cmd, 0) == pdTRUE) {
            switch (cmd.type) {
                case CtrlCommandType::SET_KP:           kp = cmd.value; break;
                case CtrlCommandType::SET_KI:           ki = cmd.value; break;
                case CtrlCommandType::SET_KD:           kd = cmd.value; break;
                case CtrlCommandType::SET_PITCH_TARGET: target = cmd.value; break;
                case CtrlCommandType::SET_SPEED_ENABLED:
                    speed_enabled = (cmd.value > 0.5f);
                    if (speed_enabled) {
                        last_cmd_ms = millis();
                        sync_read_fail_count = 0;
                    } else {
                        v_d = 0; yaw_rate_cmd = 0;
                        v_integral = 0; theta_offset = 0;
                        sync_read_fail_count = 0;
                    }
                    break;
                case CtrlCommandType::SET_KP_SPEED:     kp_speed = cmd.value; break;
                case CtrlCommandType::SET_KI_SPEED:     ki_speed = cmd.value; break;
                case CtrlCommandType::SET_VELOCITY_CMD:
                    v_d = constrain(cmd.value, -V_CMD_MAX, V_CMD_MAX);
                    last_cmd_ms = millis();
                    break;
                case CtrlCommandType::SET_YAW_RATE_CMD:
                    yaw_rate_cmd = cmd.value;
                    last_cmd_ms = millis();
                    break;
            }
        }

        // --- Command freshness timeout ---
        if (speed_enabled && (millis() - last_cmd_ms > CMD_FRESHNESS_MS)) {
            speed_enabled = false;
            v_d = 0; yaw_rate_cmd = 0;
            v_integral = 0; theta_offset = 0;
            sync_read_fail_count = 0;
        }

        // --- dt calculation ---
        float dt;
        if (prev_us == 0) {
            dt = 0.005f;
        } else {
            dt = (float)(now_us - prev_us) / 1000000.0f;
            dt = constrain(dt, 0.002f, 0.050f);
        }
        prev_us = now_us;

        // --- IMU update ---
        auto imu = imuDriver.update(dt);
        if (!imu.valid) {
            vTaskDelayUntil(&xLastWakeTime, CTRL_PERIOD_TICKS);
            continue;
        }

        float pitch_filtered = imu.pitch_deg;

        // --- Fall detection (based on physical target, not effective_target) ---
        float fall_error = target - pitch_filtered;
        if (fall_error < -FALL_THRESHOLD_DEG || FALL_THRESHOLD_DEG < fall_error) {
            driveMotors(0, 0);
            I_acc = 0.0f;
            preP = 0.0f;
            D_filtered = 0.0f;
            pid_active = false;
            speed_enabled = false;
            v_d = 0.0f;
            yaw_rate_cmd = 0.0f;
            theta_offset = 0.0f;
            v_integral = 0.0f;
            v_measured = 0.0f;
            v_d_prev = 0.0f;
            speed_divider_count = 0;
            sync_read_fail_count = 0;
            TelemetryData tel = {pitch_filtered, imu.pitch_rate_dps, 0, 0, 0, 0,
                                 (uint32_t)(esp_timer_get_time() - now_us), true,
                                 0, 0, 0, 0, 0, speed_enabled};
            xQueueOverwrite(g_telemetry_queue, &tel);
            vTaskDelayUntil(&xLastWakeTime, CTRL_PERIOD_TICKS);
            continue;
        }

        // --- Speed control outer loop (25 Hz) ---
        speed_divider_count++;
        if (speed_enabled && speed_divider_count >= SPEED_LOOP_DIVIDER) {
            speed_divider_count = 0;
            float dt_speed = dt * SPEED_LOOP_DIVIDER;

            // Budget guard: only run Sync Read if enough time remains
            int64_t budget_us = esp_timer_get_time() - now_us;
            if (budget_us < 600) {
                VelReadResult vr = readWheelVelocity();
                last_sync_read_us = vr.elapsed_us;
                if (vr.valid) {
                    v_measured = vr.v;
                    sync_read_fail_count = 0;
                } else {
                    sync_read_fail_count++;
                }
            }
            if (sync_read_fail_count >= SYNC_READ_FAIL_LIMIT) {
                speed_enabled = false;
                v_d = 0; yaw_rate_cmd = 0;
                v_integral = 0; theta_offset = 0;
                sync_read_fail_count = 0;
            }

            // Speed PI with 5-condition anti-windup
            float v_error = v_d - v_measured;
            v_error_out = v_error;

            // Condition 1: v_d sign change → reset
            if ((v_d > 0 && v_d_prev < 0) || (v_d < 0 && v_d_prev > 0)) {
                v_integral = 0.0f;
            }
            // Condition 2: v_d nonzero→zero transition → reset
            if (fabsf(v_d) < 1e-4f && fabsf(v_d_prev) >= 1e-4f) {
                v_integral = 0.0f;
            }

            // Condition 3: normal integration
            v_integral += v_error * dt_speed;
            v_integral = constrain(v_integral, -V_INTEGRAL_LIMIT, V_INTEGRAL_LIMIT);

            // Condition 4: output + back-calculation anti-windup
            float theta_raw = kp_speed * v_error + ki_speed * v_integral;
            theta_offset = constrain(theta_raw, -THETA_OFFSET_MAX, THETA_OFFSET_MAX);
            if (theta_raw != theta_offset && fabsf(ki_speed) > 1e-6f) {
                v_integral = (theta_offset - kp_speed * v_error) / ki_speed;
            }

            // Condition 5: direction consistency (with deadband)
            if (v_error * theta_offset < -1e-6f && fabsf(v_error) > V_ERR_DEADBAND) {
                v_integral = 0.0f;
                theta_offset = constrain(kp_speed * v_error, -THETA_OFFSET_MAX, THETA_OFFSET_MAX);
            }

            v_d_prev = v_d;
        }

        if (!speed_enabled) {
            theta_offset = 0.0f;
            v_integral = 0.0f;
        }

        // --- Inner PID/PD loop ---
        float effective_target = target - theta_offset;
        float pitch_error = effective_target - pitch_filtered;

        float P_val = pitch_error;

        // Reset derivative state on speed→non-speed transition BEFORE D computation
        if (!speed_enabled && speed_was_enabled) {
            preP = P_val;
            D_filtered = 0.0f;
        }

        float D_raw;
        if (!pid_active) {
            D_raw = 0.0f;
            pid_active = true;
        } else {
            if (speed_enabled) {
                // Derivative-on-measurement to avoid theta_offset step kicks
                D_raw = -imu.pitch_rate_dps;
            } else {
                // Derivative-on-error (existing behavior)
                D_raw = (P_val - preP) / dt;
            }
        }
        D_filtered = 0.2f * D_raw + 0.8f * D_filtered;
        float D_val = D_filtered;
        preP = P_val;

        float rpm_f;
        if (speed_enabled) {
            I_acc = 0.0f;
            rpm_f = kp * P_val + kd * D_val;
        } else {
            I_acc += P_val * dt;
            float i_limit = 50.0f / (fabsf(ki) + 1e-6f);
            I_acc = constrain(I_acc, -i_limit, i_limit);
            rpm_f = kp * P_val + ki * I_acc + kd * D_val;
        }
        speed_was_enabled = speed_enabled;

        // Yaw differential mixing: turn_ff = 0.5 * yaw_rate * track_width / wheel_r
        float yaw_rpm = 0.0f;
        if (speed_enabled && fabsf(yaw_rate_cmd) > 1e-4f) {
            float yaw_omega = yaw_rate_cmd * WHEEL_BASE / (2.0f * WHEEL_R);
            yaw_rpm = yaw_omega * (60.0f / (2.0f * M_PI));
        }
        int rpm_L = constrain((int)(rpm_f - yaw_rpm), -RPM_LIMIT, RPM_LIMIT);
        int rpm_R = constrain((int)(rpm_f + yaw_rpm), -RPM_LIMIT, RPM_LIMIT);
        driveMotors(rpm_L, rpm_R);

        uint32_t elapsed = (uint32_t)(esp_timer_get_time() - now_us);
        TelemetryData tel = {pitch_filtered, imu.pitch_rate_dps, (float)((rpm_L + rpm_R) / 2),
                             P_val, I_acc, D_val, elapsed, false,
                             v_measured, theta_offset, v_error_out, v_integral,
                             last_sync_read_us, speed_enabled};
        xQueueOverwrite(g_telemetry_queue, &tel);

        vTaskDelayUntil(&xLastWakeTime, CTRL_PERIOD_TICKS);
    }
}
