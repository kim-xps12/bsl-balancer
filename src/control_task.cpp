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
    memcpy(goal_vel_buf_L, &raw_L, sizeof(raw_L));
    memcpy(goal_vel_buf_R, &raw_R, sizeof(raw_R));
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

struct BalancePidState {
    float integral;
    float previous_error;
    float derivative_filtered;
    bool active;
};

struct SpeedControlState {
    bool enabled;
    bool was_enabled;
    float kp;
    float ki;
    float velocity_cmd;
    float yaw_rate_cmd;
    float integral;
    float theta_offset;
    float measured_velocity;
    float velocity_error;
    float previous_velocity_cmd;
    uint8_t divider_count;
    uint8_t sync_read_fail_count;
    uint32_t last_cmd_ms;
    uint32_t last_sync_read_us;
};

static void resetBalancePid(BalancePidState& pid) {
    pid.integral = 0.0f;
    pid.previous_error = 0.0f;
    pid.derivative_filtered = 0.0f;
    pid.active = false;
}

static void resetSpeedCommand(SpeedControlState& speed) {
    speed.velocity_cmd = 0.0f;
    speed.yaw_rate_cmd = 0.0f;
    speed.integral = 0.0f;
    speed.theta_offset = 0.0f;
    speed.sync_read_fail_count = 0;
}

static void enableSpeedControl(SpeedControlState& speed) {
    speed.enabled = true;
    speed.last_cmd_ms = millis();
    speed.sync_read_fail_count = 0;
}

static void disableSpeedControl(SpeedControlState& speed) {
    speed.enabled = false;
    resetSpeedCommand(speed);
}

static void resetSpeedAfterFall(SpeedControlState& speed) {
    disableSpeedControl(speed);
    speed.measured_velocity = 0.0f;
    speed.previous_velocity_cmd = 0.0f;
    speed.divider_count = 0;
}

// --- Control Loop ---
void controlLoopTask(void *pvParameters) {
    esp_task_wdt_add(NULL);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    float kp = DEFAULT_KP;
    float ki = DEFAULT_KI;
    float kd = DEFAULT_KD;
    float target = DEFAULT_PITCH_TARGET;
    BalancePidState pid = {0.0f, 0.0f, 0.0f, false};
    int64_t prev_us = 0;

    SpeedControlState speed = {
        false, false,
        DEFAULT_KP_SPEED, DEFAULT_KI_SPEED,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0, 0,
        0, 0,
    };

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
                    if (cmd.value > 0.5f) enableSpeedControl(speed);
                    else disableSpeedControl(speed);
                    break;
                case CtrlCommandType::SET_KP_SPEED:     speed.kp = cmd.value; break;
                case CtrlCommandType::SET_KI_SPEED:     speed.ki = cmd.value; break;
                case CtrlCommandType::SET_VELOCITY_CMD:
                    speed.velocity_cmd = constrain(cmd.value, -V_CMD_MAX, V_CMD_MAX);
                    speed.last_cmd_ms = millis();
                    break;
                case CtrlCommandType::SET_YAW_RATE_CMD:
                    speed.yaw_rate_cmd = cmd.value;
                    speed.last_cmd_ms = millis();
                    break;
            }
        }

        // --- Command freshness timeout ---
        if (speed.enabled && (millis() - speed.last_cmd_ms > CMD_FRESHNESS_MS)) {
            disableSpeedControl(speed);
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
            resetBalancePid(pid);
            resetSpeedAfterFall(speed);
            TelemetryData tel = {pitch_filtered, imu.pitch_rate_dps, 0, 0, 0, 0,
                                 (uint32_t)(esp_timer_get_time() - now_us), true,
                                 0, 0, 0, 0, 0, speed.enabled};
            xQueueOverwrite(g_telemetry_queue, &tel);
            vTaskDelayUntil(&xLastWakeTime, CTRL_PERIOD_TICKS);
            continue;
        }

        // --- Speed control outer loop (25 Hz) ---
        speed.divider_count++;
        if (speed.enabled && speed.divider_count >= SPEED_LOOP_DIVIDER) {
            speed.divider_count = 0;
            float dt_speed = dt * SPEED_LOOP_DIVIDER;

            // Budget guard: only run Sync Read if enough time remains
            int64_t budget_us = esp_timer_get_time() - now_us;
            if (budget_us < 600) {
                VelReadResult vr = readWheelVelocity();
                speed.last_sync_read_us = vr.elapsed_us;
                if (vr.valid) {
                    speed.measured_velocity = vr.v;
                    speed.sync_read_fail_count = 0;
                } else {
                    speed.sync_read_fail_count++;
                }
            }
            if (speed.sync_read_fail_count >= SYNC_READ_FAIL_LIMIT) {
                disableSpeedControl(speed);
            }

            // Speed PI with 5-condition anti-windup
            float v_error = speed.velocity_cmd - speed.measured_velocity;
            speed.velocity_error = v_error;

            // Condition 1: velocity command sign change -> reset
            if ((speed.velocity_cmd > 0 && speed.previous_velocity_cmd < 0)
                || (speed.velocity_cmd < 0 && speed.previous_velocity_cmd > 0)) {
                speed.integral = 0.0f;
            }
            // Condition 2: velocity command nonzero-to-zero transition -> reset
            if (fabsf(speed.velocity_cmd) < 1e-4f && fabsf(speed.previous_velocity_cmd) >= 1e-4f) {
                speed.integral = 0.0f;
            }

            // Condition 3: normal integration
            speed.integral += v_error * dt_speed;
            speed.integral = constrain(speed.integral, -V_INTEGRAL_LIMIT, V_INTEGRAL_LIMIT);

            // Condition 4: output + back-calculation anti-windup
            float theta_raw = speed.kp * v_error + speed.ki * speed.integral;
            speed.theta_offset = constrain(theta_raw, -THETA_OFFSET_MAX, THETA_OFFSET_MAX);
            if (theta_raw != speed.theta_offset && fabsf(speed.ki) > 1e-6f) {
                speed.integral = (speed.theta_offset - speed.kp * v_error) / speed.ki;
            }

            // Condition 5: direction consistency (with deadband)
            if (v_error * speed.theta_offset < -1e-6f && fabsf(v_error) > V_ERR_DEADBAND) {
                speed.integral = 0.0f;
                speed.theta_offset = constrain(speed.kp * v_error, -THETA_OFFSET_MAX, THETA_OFFSET_MAX);
            }

            speed.previous_velocity_cmd = speed.velocity_cmd;
        }

        if (!speed.enabled) {
            speed.theta_offset = 0.0f;
            speed.integral = 0.0f;
        }

        // --- Inner PID/PD loop ---
        float effective_target = target - speed.theta_offset;
        float pitch_error = effective_target - pitch_filtered;

        float P_val = pitch_error;

        // Reset derivative state on speed-to-normal transition BEFORE D computation
        if (!speed.enabled && speed.was_enabled) {
            pid.previous_error = P_val;
            pid.derivative_filtered = 0.0f;
        }

        float D_raw;
        if (!pid.active) {
            D_raw = 0.0f;
            pid.active = true;
        } else {
            if (speed.enabled) {
                // Derivative-on-measurement to avoid theta_offset step kicks
                D_raw = -imu.pitch_rate_dps;
            } else {
                // Derivative-on-error (existing behavior)
                D_raw = (P_val - pid.previous_error) / dt;
            }
        }
        pid.derivative_filtered = 0.2f * D_raw + 0.8f * pid.derivative_filtered;
        float D_val = pid.derivative_filtered;
        pid.previous_error = P_val;

        float rpm_f;
        if (speed.enabled) {
            pid.integral = 0.0f;
            rpm_f = kp * P_val + kd * D_val;
        } else {
            pid.integral += P_val * dt;
            float i_limit = 50.0f / (fabsf(ki) + 1e-6f);
            pid.integral = constrain(pid.integral, -i_limit, i_limit);
            rpm_f = kp * P_val + ki * pid.integral + kd * D_val;
        }
        speed.was_enabled = speed.enabled;

        // Yaw differential mixing: turn_ff = 0.5 * yaw_rate * track_width / wheel_r
        float yaw_rpm = 0.0f;
        if (speed.enabled && fabsf(speed.yaw_rate_cmd) > 1e-4f) {
            float yaw_omega = speed.yaw_rate_cmd * WHEEL_BASE / (2.0f * WHEEL_R);
            yaw_rpm = yaw_omega * (60.0f / (2.0f * M_PI));
        }
        int rpm_L = constrain((int)(rpm_f - yaw_rpm), -RPM_LIMIT, RPM_LIMIT);
        int rpm_R = constrain((int)(rpm_f + yaw_rpm), -RPM_LIMIT, RPM_LIMIT);
        driveMotors(rpm_L, rpm_R);

        uint32_t elapsed = (uint32_t)(esp_timer_get_time() - now_us);
        TelemetryData tel = {pitch_filtered, imu.pitch_rate_dps, (float)((rpm_L + rpm_R) / 2),
                             P_val, pid.integral, D_val, elapsed, false,
                             speed.measured_velocity, speed.theta_offset, speed.velocity_error,
                             speed.integral, speed.last_sync_read_us, speed.enabled};
        xQueueOverwrite(g_telemetry_queue, &tel);

        vTaskDelayUntil(&xLastWakeTime, CTRL_PERIOD_TICKS);
    }
}
