#pragma once
#include <Arduino.h>

// M5Stack Core2 PORT A pin configuration
constexpr uint8_t PIN_RX_SERVO = 33;
constexpr uint8_t PIN_TX_SERVO = 32;

// Default PID gains
constexpr float DEFAULT_KP = 15.0f;
constexpr float DEFAULT_KI = 0.05f;
constexpr float DEFAULT_KD = 0.0f;
constexpr float DEFAULT_PITCH_TARGET = 86.0f;

// Control loop timing
const TickType_t CTRL_PERIOD_TICKS = pdMS_TO_TICKS(5);  // 200Hz

// DYNAMIXEL
constexpr uint8_t DXL_ID_L = 0;
constexpr uint8_t DXL_ID_R = 1;
constexpr float DXL_PROTOCOL_VERSION = 2.0f;
constexpr uint32_t DXL_BAUDRATE = 1000000;

// Fall detection threshold [deg]
constexpr float FALL_THRESHOLD_DEG = 40.0f;

// Motor RPM limit
constexpr int RPM_LIMIT = 300;

// Speed control (outer loop)
constexpr float WHEEL_R = 0.029f;
constexpr float DEFAULT_KP_SPEED = 5.0f;
constexpr float DEFAULT_KI_SPEED = 2.0f;
constexpr float THETA_OFFSET_MAX = 5.0f;
constexpr float V_INTEGRAL_LIMIT = 2.5f;
constexpr uint8_t SPEED_LOOP_DIVIDER = 8;
constexpr float WHEEL_BASE = 0.0669f;
constexpr float V_CMD_MAX = 0.3f;
constexpr float V_ERR_DEADBAND = 0.05f;
constexpr uint32_t CMD_FRESHNESS_MS = 500;
constexpr uint8_t SYNC_READ_FAIL_LIMIT = 5;

// DXL Sync Read/Write addresses
constexpr uint16_t DXL_ADDR_GOAL_VELOCITY = 104;
constexpr uint16_t DXL_ADDR_PRESENT_VELOCITY = 128;
constexpr uint16_t DXL_LEN_VELOCITY = 4;
constexpr float DXL_VEL_UNIT = 0.229f;
