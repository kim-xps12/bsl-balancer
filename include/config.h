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
