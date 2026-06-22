#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

enum class CtrlCommandType : uint8_t {
    SET_PITCH_TARGET, SET_KP, SET_KI, SET_KD,
    SET_SPEED_ENABLED,
    SET_KP_SPEED,
    SET_KI_SPEED,
    SET_VELOCITY_CMD,
    SET_YAW_RATE_CMD,
};

struct CtrlCommand {
    CtrlCommandType type;
    float value;
};

struct TelemetryData {
    float pitch_deg;
    float pitch_rate_dps;
    float rpm_cmd;
    float P_term, I_term, D_term;
    uint32_t loop_us;
    bool fallen;
    float v_measured;
    float theta_offset;
    float v_error;
    float v_integral;
    uint32_t sync_read_us;
    bool speed_enabled;
};

extern QueueHandle_t g_cmd_queue;
extern QueueHandle_t g_telemetry_queue;
extern SemaphoreHandle_t g_i2c_mutex;
