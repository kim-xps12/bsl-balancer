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

// --- Unit conversion ---
static inline int16_t currentAToRaw(float current_A) {
    float clamped = fmaxf(-EEPROM_CURRENT_LIMIT_A, fminf(EEPROM_CURRENT_LIMIT_A, current_A));
    return (int16_t)roundf(clamped / DXL_CURRENT_UNIT_A);
}

static inline float currentRawToA(int16_t raw) {
    return (float)raw * DXL_CURRENT_UNIT_A;
}

static inline float velocityRawToRadS(int32_t raw) {
    return (float)raw * DXL_VEL_UNIT * (2.0f * M_PI / 60.0f);
}

// --- Sync Write for Goal Current (2 byte × 2 motors) ---
static uint8_t goal_cur_buf_L[DXL_LEN_GOAL_CURRENT];
static uint8_t goal_cur_buf_R[DXL_LEN_GOAL_CURRENT];
static DYNAMIXEL::XELInfoSyncWrite_t cur_write_xels[2] = {
    {goal_cur_buf_L, DXL_ID_L},
    {goal_cur_buf_R, DXL_ID_R},
};
static DYNAMIXEL::InfoSyncWriteInst_t cur_sync_write = {
    DXL_ADDR_GOAL_CURRENT,
    DXL_LEN_GOAL_CURRENT,
    cur_write_xels,
    2,
    true,
    {nullptr, 0, 0, false}
};

static void driveMotors(float body_current_L_A, float body_current_R_A) {
    int16_t raw_L = currentAToRaw(SIGN_LEFT_MOTOR  * body_current_L_A);
    int16_t raw_R = currentAToRaw(SIGN_RIGHT_MOTOR * body_current_R_A);
    memcpy(goal_cur_buf_L, &raw_L, 2);
    memcpy(goal_cur_buf_R, &raw_R, 2);
    cur_sync_write.is_info_changed = true;
    dxl.syncWrite(&cur_sync_write);
}

static void driveMotorsZero() {
    driveMotors(0.0f, 0.0f);
}

// --- Sync Read for feedback block (addr 126, 10 byte × 2 motors) ---
static uint8_t fb_recv_buf_L[DXL_LEN_FEEDBACK_BLOCK];
static uint8_t fb_recv_buf_R[DXL_LEN_FEEDBACK_BLOCK];
static DYNAMIXEL::XELInfoSyncRead_t fb_read_xels[2] = {
    {fb_recv_buf_L, DXL_ID_L, 0},
    {fb_recv_buf_R, DXL_ID_R, 0},
};
static DYNAMIXEL::InfoSyncReadInst_t fb_sync_read = {
    DXL_ADDR_PRESENT_CURRENT,
    DXL_LEN_FEEDBACK_BLOCK,
    fb_read_xels,
    2,
    true,
    {nullptr, 0, 0, false}
};

static inline int16_t decodeInt16(const uint8_t* buf) {
    uint16_t u = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    int16_t s;
    memcpy(&s, &u, sizeof(s));
    return s;
}

static inline int32_t decodeInt32(const uint8_t* buf) {
    uint32_t u = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8)
              | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    int32_t s;
    memcpy(&s, &u, sizeof(s));
    return s;
}

struct FeedbackResult {
    MotorFeedback left;
    MotorFeedback right;
    uint32_t elapsed_us;
};

static FeedbackResult readFeedback() {
    FeedbackResult r = {};
    int64_t t0 = esp_timer_get_time();
    uint8_t recv_count = dxl.syncRead(&fb_sync_read, 2);
    r.elapsed_us = (uint32_t)(esp_timer_get_time() - t0);

    if (recv_count >= 1 && fb_read_xels[0].error == 0) {
        int16_t cur_raw  = decodeInt16(fb_recv_buf_L);
        int32_t vel_raw  = decodeInt32(fb_recv_buf_L + 2);
        int32_t pos_raw  = decodeInt32(fb_recv_buf_L + 6);
        r.left.current_A      = currentRawToA(cur_raw);
        r.left.velocity_rad_s = SIGN_LEFT_MOTOR * velocityRawToRadS(vel_raw);
        r.left.position_rad   = (float)pos_raw * 2.0f * M_PI / DXL_POS_COUNTS_PER_REV;
        r.left.valid = true;
    }
    if (recv_count >= 2 && fb_read_xels[1].error == 0) {
        int16_t cur_raw  = decodeInt16(fb_recv_buf_R);
        int32_t vel_raw  = decodeInt32(fb_recv_buf_R + 2);
        int32_t pos_raw  = decodeInt32(fb_recv_buf_R + 6);
        r.right.current_A      = currentRawToA(cur_raw);
        r.right.velocity_rad_s = SIGN_RIGHT_MOTOR * velocityRawToRadS(vel_raw);
        r.right.position_rad   = (float)pos_raw * 2.0f * M_PI / DXL_POS_COUNTS_PER_REV;
        r.right.valid = true;
    }
    return r;
}

// --- Speed guard (per-wheel, body frame) ---
static float applySpeedGuard(float i_cmd_body, float omega_body,
                             bool& fault_out) {
    float abs_w = fabsf(omega_body);
    if (abs_w >= OMEGA_HARD_RAD_S) {
        fault_out = true;
        return 0.0f;
    }
    bool accelerating = (i_cmd_body * omega_body) > 0.0f;
    if (!accelerating || abs_w <= OMEGA_SOFT_RAD_S) {
        return i_cmd_body;
    }
    float scale = (OMEGA_HARD_RAD_S - abs_w) / (OMEGA_HARD_RAD_S - OMEGA_SOFT_RAD_S);
    scale = fmaxf(0.0f, fminf(1.0f, scale));
    return i_cmd_body * scale;
}

// --- Low-level read helpers ---
static int32_t dxlReadU8(uint8_t id, uint16_t addr, uint32_t timeout_ms = 10) {
    uint8_t buf[1];
    int32_t n = dxl.read(id, addr, 1, buf, sizeof(buf), timeout_ms);
    if (n < 0) return -1;
    return (int32_t)buf[0];
}

static bool dxlWriteU8(uint8_t id, uint16_t addr, uint8_t val) {
    return dxl.write(id, addr, &val, 1);
}

static bool dxlWriteI16(uint8_t id, uint16_t addr, int16_t val) {
    uint8_t buf[2];
    memcpy(buf, &val, 2);
    return dxl.write(id, addr, buf, 2);
}

// --- FAULT handling ---
static void enterFault(const char* reason) {
    Serial.printf("FAULT: %s\n", reason);

    driveMotorsZero();
    delay(2);
    driveMotorsZero();

    dxl.torqueOff(DXL_ID_L);
    dxl.torqueOff(DXL_ID_R);

    int32_t te_l = dxlReadU8(DXL_ID_L, DXL_ADDR_TORQUE_ENABLE);
    int32_t te_r = dxlReadU8(DXL_ID_R, DXL_ADDR_TORQUE_ENABLE);
    if (te_l != 0 || te_r != 0) {
        Serial.println("POWER CYCLE REQUIRED: Torque OFF readback failed");
        return;
    }

    dxlWriteU8(DXL_ID_L, DXL_ADDR_BUS_WATCHDOG, 0);
    dxlWriteU8(DXL_ID_R, DXL_ADDR_BUS_WATCHDOG, 0);
}

static void enterFaultWatchdog() {
    Serial.println("FAULT: Bus Watchdog triggered");

    dxl.torqueOff(DXL_ID_L);
    dxl.torqueOff(DXL_ID_R);

    int32_t te_l = dxlReadU8(DXL_ID_L, DXL_ADDR_TORQUE_ENABLE);
    int32_t te_r = dxlReadU8(DXL_ID_R, DXL_ADDR_TORQUE_ENABLE);
    if (te_l != 0 || te_r != 0) {
        Serial.println("POWER CYCLE REQUIRED: Torque OFF readback failed after watchdog");
        return;
    }

    dxlWriteU8(DXL_ID_L, DXL_ADDR_BUS_WATCHDOG, 0);
    dxlWriteU8(DXL_ID_R, DXL_ADDR_BUS_WATCHDOG, 0);
    int32_t wd_l = dxlReadU8(DXL_ID_L, DXL_ADDR_BUS_WATCHDOG);
    int32_t wd_r = dxlReadU8(DXL_ID_R, DXL_ADDR_BUS_WATCHDOG);
    if (wd_l != 0 || wd_r != 0) {
        Serial.println("POWER CYCLE REQUIRED: Watchdog clear failed");
        return;
    }

    dxlWriteI16(DXL_ID_L, DXL_ADDR_GOAL_CURRENT, 0);
    dxlWriteI16(DXL_ID_R, DXL_ADDR_GOAL_CURRENT, 0);
}

// --- Control Loop ---
void controlLoopTask(void *pvParameters) {
    esp_task_wdt_add(NULL);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    bool armed = (bool)(uintptr_t)pvParameters;
    SafetyState state = armed ? SafetyState::ARMED_IDLE : SafetyState::DISARMED;

    float kp = DEFAULT_KP;
    float ki = DEFAULT_KI;
    float kd = DEFAULT_KD;
    float target = DEFAULT_PITCH_TARGET;
    float I_acc = 0.0f;
    float preP = 0.0f;
    float D_filtered = 0.0f;
    bool pid_active = false;
    int64_t prev_us = 0;

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
    uint8_t sync_read_skip_count = 0;
    uint32_t last_cmd_ms = 0;
    uint32_t last_sync_read_us = 0;

    // Health monitoring
    uint32_t health_tick[2][3] = {};  // [motor][item]: last success tick
    uint8_t health_init_bitmap = 0;   // 6 bits for 2 motors × 3 items
    float last_voltage_V = 5.0f;
    uint8_t last_temp_C = 25;
    uint32_t tick_count = 0;
    uint8_t loop_fault_count = 0;
    float peak_accumulator_s = 0.0f;  // I²t-like: time at peak current

    while (true) {
        esp_task_wdt_reset();
        int64_t now_us = esp_timer_get_time();
        tick_count++;

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
                        sync_read_skip_count = 0;
                    } else {
                        v_d = 0; yaw_rate_cmd = 0;
                        v_integral = 0; theta_offset = 0;
                        sync_read_fail_count = 0;
                        sync_read_skip_count = 0;
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

        if (speed_enabled && (millis() - last_cmd_ms > CMD_FRESHNESS_MS)) {
            speed_enabled = false;
            v_d = 0; yaw_rate_cmd = 0;
            v_integral = 0; theta_offset = 0;
            sync_read_fail_count = 0;
            sync_read_skip_count = 0;
        }

        // --- DISARMED / FAULT: no driving, just telemetry ---
        if (state == SafetyState::DISARMED || state == SafetyState::FAULT ||
            state == SafetyState::SHUTDOWN) {
            TelemetryData tel = {};
            tel.state = state;
            tel.fallen = true;
            xQueueOverwrite(g_telemetry_queue, &tel);
            vTaskDelayUntil(&xLastWakeTime, CTRL_PERIOD_TICKS);
            continue;
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

        // --- Health monitoring (distributed, 5 Hz per item per motor) ---
        {
            uint32_t phase = tick_count % 40;
            int item = -1;
            if (phase == 0)  item = 0;  // HW Error Status
            if (phase == 13) item = 1;  // Voltage
            if (phase == 26) item = 2;  // Temperature

            if (item >= 0) {
                uint32_t elapsed = (uint32_t)(esp_timer_get_time() - now_us);
                if (elapsed < 3500) {  // budget guard
                    for (int m = 0; m < 2; m++) {
                        uint8_t id = (m == 0) ? DXL_ID_L : DXL_ID_R;
                        int32_t val = -1;
                        if (item == 0) {
                            val = dxlReadU8(id, DXL_ADDR_HW_ERROR_STATUS, 2);
                            if (val >= 0 && val != 0) {
                                enterFault("HW Error");
                                state = SafetyState::FAULT;
                                break;
                            }
                            // Check Bus Watchdog: 0xFF means expired, -1 is read failure (ignore)
                            int32_t wd = dxlReadU8(id, DXL_ADDR_BUS_WATCHDOG, 2);
                            if (wd >= 0 && (uint8_t)wd == 0xFF) {
                                enterFaultWatchdog();
                                state = SafetyState::FAULT;
                                break;
                            }
                        } else if (item == 1) {
                            uint8_t vbuf[2];
                            int32_t n = dxl.read(id, DXL_ADDR_PRESENT_VOLTAGE, 2, vbuf, sizeof(vbuf), 2);
                            if (n >= 0) {
                                val = (int32_t)((uint16_t)vbuf[0] | ((uint16_t)vbuf[1] << 8));
                                last_voltage_V = (float)val * 0.1f;
                                if (last_voltage_V < VOLTAGE_MIN_V || last_voltage_V > VOLTAGE_MAX_V) {
                                    enterFault("Voltage out of range");
                                    state = SafetyState::FAULT;
                                    break;
                                }
                            }
                        } else {
                            val = dxlReadU8(id, DXL_ADDR_PRESENT_TEMP, 2);
                            if (val >= 0) {
                                last_temp_C = (uint8_t)val;
                                if (last_temp_C > TEMP_MAX_C) {
                                    enterFault("Overtemperature");
                                    state = SafetyState::FAULT;
                                    break;
                                }
                            }
                        }
                        if (val >= 0) {
                            health_tick[m][item] = tick_count;
                            health_init_bitmap |= (1 << (m * 3 + item));
                        }
                    }
                }
            }

            // Check health freshness
            if (state == SafetyState::BALANCING) {
                for (int m = 0; m < 2; m++) {
                    for (int i = 0; i < 3; i++) {
                        if (tick_count - health_tick[m][i] > HEALTH_STALE_LIMIT) {
                            enterFault("Health stale");
                            state = SafetyState::FAULT;
                            break;
                        }
                    }
                    if (state == SafetyState::FAULT) break;
                }
            }
            if (state == SafetyState::FAULT) {
                vTaskDelayUntil(&xLastWakeTime, CTRL_PERIOD_TICKS);
                continue;
            }
        }

        // --- State transitions: ARMED_IDLE → BALANCING (immediate, pitch only) ---
        if (state == SafetyState::ARMED_IDLE) {
            float pitch_err = fabsf(target - pitch_filtered);
            if (pitch_err < FALL_THRESHOLD_DEG) {
                state = SafetyState::BALANCING;
                pid_active = false;
                I_acc = 0.0f;
                preP = 0.0f;
                D_filtered = 0.0f;
                Serial.println("State: BALANCING");
            } else {
                driveMotorsZero();
                TelemetryData tel = {};
                tel.pitch_deg = pitch_filtered;
                tel.pitch_rate_dps = imu.pitch_rate_dps;
                tel.state = state;
                tel.voltage_V = last_voltage_V;
                tel.temp_C = last_temp_C;
                xQueueOverwrite(g_telemetry_queue, &tel);
                vTaskDelayUntil(&xLastWakeTime, CTRL_PERIOD_TICKS);
                continue;
            }
        }

        // --- Fall detection (revert to ARMED_IDLE for auto-recovery) ---
        float fall_error = target - pitch_filtered;
        if (fall_error < -FALL_THRESHOLD_DEG || FALL_THRESHOLD_DEG < fall_error) {
            driveMotorsZero();
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
            sync_read_skip_count = 0;
            peak_accumulator_s = 0.0f;
            state = SafetyState::ARMED_IDLE;
            TelemetryData tel = {};
            tel.pitch_deg = pitch_filtered;
            tel.pitch_rate_dps = imu.pitch_rate_dps;
            tel.state = state;
            tel.fallen = true;
            tel.voltage_V = last_voltage_V;
            tel.temp_C = last_temp_C;
            xQueueOverwrite(g_telemetry_queue, &tel);
            vTaskDelayUntil(&xLastWakeTime, CTRL_PERIOD_TICKS);
            continue;
        }

        // --- Sync Read feedback (preserve last valid on skip) ---
        static MotorFeedback last_fb_L = {}, last_fb_R = {};
        MotorFeedback fb_L = last_fb_L, fb_R = last_fb_R;
        bool fresh_read = false;
        {
            uint32_t elapsed = (uint32_t)(esp_timer_get_time() - now_us);
            uint32_t remaining = (elapsed < 5000) ? (5000 - elapsed) : 0;
            if (remaining >= SYNC_READ_BUDGET_US) {
                FeedbackResult fb = readFeedback();
                last_sync_read_us = fb.elapsed_us;
                if (fb.left.valid && fb.right.valid) {
                    fb_L = fb.left;
                    fb_R = fb.right;
                    last_fb_L = fb_L;
                    last_fb_R = fb_R;
                    sync_read_fail_count = 0;
                    sync_read_skip_count = 0;
                    fresh_read = true;
                } else {
                    sync_read_fail_count++;
                    last_fb_L.valid = false;
                    last_fb_R.valid = false;
                }
            } else {
                sync_read_skip_count++;
            }
        }

        bool feedback_valid = last_fb_L.valid && last_fb_R.valid
                           && sync_read_skip_count < 2;

        if (sync_read_fail_count >= SYNC_READ_FAIL_LIMIT) {
            enterFault("Sync Read fail limit");
            state = SafetyState::FAULT;
        } else if (sync_read_skip_count >= 5) {
            enterFault("Sync Read skip limit");
            state = SafetyState::FAULT;
        }

        if (state == SafetyState::FAULT) {
            vTaskDelayUntil(&xLastWakeTime, CTRL_PERIOD_TICKS);
            continue;
        }

        if (feedback_valid) {
            float avg_omega = (fb_L.velocity_rad_s + fb_R.velocity_rad_s) * 0.5f;
            v_measured = avg_omega * WHEEL_R;
        }

        // --- Speed control outer loop (25 Hz) ---
        speed_divider_count++;
        if (speed_enabled && speed_divider_count >= SPEED_LOOP_DIVIDER) {
            speed_divider_count = 0;
            float dt_speed = dt * SPEED_LOOP_DIVIDER;

            float v_error = v_d - v_measured;
            v_error_out = v_error;

            if ((v_d > 0 && v_d_prev < 0) || (v_d < 0 && v_d_prev > 0)) {
                v_integral = 0.0f;
            }
            if (fabsf(v_d) < 1e-4f && fabsf(v_d_prev) >= 1e-4f) {
                v_integral = 0.0f;
            }

            v_integral += v_error * dt_speed;
            v_integral = constrain(v_integral, -V_INTEGRAL_LIMIT, V_INTEGRAL_LIMIT);

            float theta_raw = kp_speed * v_error + ki_speed * v_integral;
            theta_offset = constrain(theta_raw, -THETA_OFFSET_MAX, THETA_OFFSET_MAX);
            if (theta_raw != theta_offset && fabsf(ki_speed) > 1e-6f) {
                v_integral = (theta_offset - kp_speed * v_error) / ki_speed;
            }

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

        // --- Inner PID loop (output: current [A]) ---
        float effective_target = target - theta_offset;
        float pitch_error = effective_target - pitch_filtered;
        float P_val = pitch_error;

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
                D_raw = -imu.pitch_rate_dps;
            } else {
                D_raw = (P_val - preP) / dt;
            }
        }
        D_filtered = 0.2f * D_raw + 0.8f * D_filtered;
        float D_val = D_filtered;
        preP = P_val;

        float i_common;
        if (speed_enabled) {
            I_acc = 0.0f;
            i_common = kp * P_val + kd * D_val;
        } else {
            I_acc += P_val * dt;
            float i_limit_int = SW_PEAK_CURRENT_LIMIT_A / (fabsf(ki) + 1e-6f);
            I_acc = constrain(I_acc, -i_limit_int, i_limit_int);
            i_common = kp * P_val + ki * I_acc + kd * D_val;
        }
        speed_was_enabled = speed_enabled;

        // Yaw differential
        float i_yaw = 0.0f;
        if (speed_enabled && fabsf(yaw_rate_cmd) > 1e-4f) {
            float yaw_omega = yaw_rate_cmd * WHEEL_BASE / (2.0f * WHEEL_R);
            i_yaw = yaw_omega * 0.01f;
        }

        // Save pre-clamp per-wheel demand for I²t tracking
        float i_demand_preclamp = fmaxf(fabsf(i_common - i_yaw), fabsf(i_common + i_yaw));

        // Continuous current limit (I²t-like tracking)
        float i_limit_now;
        if (!feedback_valid) {
            i_limit_now = SW_CONT_CURRENT_LIMIT_A;
        } else if (peak_accumulator_s >= PEAK_DURATION_S) {
            i_limit_now = SW_CONT_CURRENT_LIMIT_A;
        } else {
            i_limit_now = SW_PEAK_CURRENT_LIMIT_A;
        }

        // Current saturation: balance priority
        float i_max = i_limit_now;
        i_common = constrain(i_common, -i_max, i_max);
        float yaw_headroom = fmaxf(0.0f, i_max - fabsf(i_common));
        i_yaw = constrain(i_yaw, -yaw_headroom, yaw_headroom);

        float i_body_L = i_common - i_yaw;
        float i_body_R = i_common + i_yaw;

        // Speed guard (per-wheel, body frame)
        if (feedback_valid) {
            bool overspeed = false;
            i_body_L = applySpeedGuard(i_body_L, fb_L.velocity_rad_s, overspeed);
            i_body_R = applySpeedGuard(i_body_R, fb_R.velocity_rad_s, overspeed);
            if (overspeed) {
                enterFault("Overspeed");
                state = SafetyState::FAULT;
                vTaskDelayUntil(&xLastWakeTime, CTRL_PERIOD_TICKS);
                continue;
            }
        }

        driveMotors(i_body_L, i_body_R);

        // I²t accumulator: track pre-clamp demand to avoid limit/release oscillation
        if (i_demand_preclamp > SW_CONT_CURRENT_LIMIT_A) {
            peak_accumulator_s += dt;
        } else if (peak_accumulator_s > 0.0f) {
            peak_accumulator_s = fmaxf(0.0f, peak_accumulator_s - dt * 0.5f);
        }

        // --- Loop timing guard ---
        uint32_t elapsed = (uint32_t)(esp_timer_get_time() - now_us);
        if (elapsed > LOOP_WARN_US) {
            Serial.printf("Loop overrun: %u us\n", elapsed);
        }
        if (elapsed > LOOP_FAULT_US) {
            loop_fault_count++;
            if (loop_fault_count >= LOOP_FAULT_COUNT) {
                enterFault("Loop timing");
                state = SafetyState::FAULT;
            }
        } else {
            loop_fault_count = 0;
        }

        // --- Telemetry ---
        TelemetryData tel = {};
        tel.pitch_deg = pitch_filtered;
        tel.pitch_rate_dps = imu.pitch_rate_dps;
        tel.current_cmd_A = (i_body_L + i_body_R) * 0.5f;
        tel.P_term = P_val;
        tel.I_term = I_acc;
        tel.D_term = D_val;
        tel.loop_us = elapsed;
        tel.fallen = false;
        tel.v_measured = v_measured;
        tel.theta_offset = theta_offset;
        tel.v_error = v_error_out;
        tel.v_integral = v_integral;
        tel.sync_read_us = last_sync_read_us;
        tel.speed_enabled = speed_enabled;
        tel.state = state;
        tel.current_L_A = fb_L.valid ? fb_L.current_A : 0.0f;
        tel.current_R_A = fb_R.valid ? fb_R.current_A : 0.0f;
        tel.voltage_V = last_voltage_V;
        tel.temp_C = last_temp_C;
        xQueueOverwrite(g_telemetry_queue, &tel);

        vTaskDelayUntil(&xLastWakeTime, CTRL_PERIOD_TICKS);
    }
}
