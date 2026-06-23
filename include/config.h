#pragma once
#include <Arduino.h>

// M5Stack Core2 PORT A pin configuration
constexpr uint8_t PIN_RX_SERVO = 33;
constexpr uint8_t PIN_TX_SERVO = 32;

// --- System identification unit gate (fail-closed) ---
#define CTRL_UNIT_UNSET       0
#define CTRL_UNIT_A           1
#define CTRL_UNIT_MA          2
#define CTRL_UNIT_RAW         3
#define CTRL_UNIT_NORMALIZED  4

#define FIT_INPUT_UNSET       0
#define FIT_INPUT_GOAL        1
#define FIT_INPUT_PRESENT     2

#define CTRL_UNIT             CTRL_UNIT_A
#define FIT_INPUT_SOURCE      FIT_INPUT_GOAL

static_assert(
    CTRL_UNIT == CTRL_UNIT_UNSET || CTRL_UNIT == CTRL_UNIT_A,
    "CTRL_UNIT must be UNSET(0) or A(1). MA/RAW/NORMALIZED not yet supported");
static_assert(
    FIT_INPUT_SOURCE == FIT_INPUT_UNSET || FIT_INPUT_SOURCE == FIT_INPUT_GOAL ||
    FIT_INPUT_SOURCE == FIT_INPUT_PRESENT,
    "FIT_INPUT_SOURCE must be one of: UNSET(0), GOAL(1), PRESENT(2)");

constexpr bool SYSID_CONFIGURED =
    (CTRL_UNIT == CTRL_UNIT_A) &&
    (FIT_INPUT_SOURCE == FIT_INPUT_GOAL || FIT_INPUT_SOURCE == FIT_INPUT_PRESENT);

// Default PID gains (output: current [A], input: pitch error [deg])
constexpr float DEFAULT_KP = 0.1f;
constexpr float DEFAULT_KI = 0.0f;
constexpr float DEFAULT_KD = 0.0f;
constexpr float DEFAULT_PITCH_TARGET = 86.0f;

// Control loop timing
const TickType_t CTRL_PERIOD_TICKS = pdMS_TO_TICKS(5);  // 200Hz

// DYNAMIXEL
constexpr uint8_t DXL_ID_L = 0;
constexpr uint8_t DXL_ID_R = 1;
constexpr float DXL_PROTOCOL_VERSION = 2.0f;
constexpr uint32_t DXL_BAUDRATE = 1000000;
constexpr uint16_t DXL_EXPECTED_MODEL_NUMBER = 1190;

// Operating mode
constexpr uint8_t DXL_OP_MODE_CURRENT = 0;

// DXL control table addresses
constexpr uint16_t DXL_ADDR_RETURN_DELAY     = 9;
constexpr uint16_t DXL_ADDR_OPERATING_MODE   = 11;
constexpr uint16_t DXL_ADDR_CURRENT_LIMIT    = 38;
constexpr uint16_t DXL_ADDR_TORQUE_ENABLE    = 64;
constexpr uint16_t DXL_ADDR_HW_ERROR_STATUS  = 70;
constexpr uint16_t DXL_ADDR_BUS_WATCHDOG     = 98;
constexpr uint16_t DXL_ADDR_GOAL_CURRENT     = 102;
constexpr uint16_t DXL_LEN_GOAL_CURRENT      = 2;
constexpr uint16_t DXL_ADDR_PRESENT_CURRENT  = 126;
constexpr uint16_t DXL_LEN_FEEDBACK_BLOCK    = 10;  // Current(2)+Velocity(4)+Position(4)
constexpr uint16_t DXL_ADDR_PRESENT_VELOCITY = 128;
constexpr uint16_t DXL_ADDR_PRESENT_VOLTAGE  = 144;
constexpr uint16_t DXL_ADDR_PRESENT_TEMP     = 146;

// Unit conversion
constexpr float DXL_CURRENT_UNIT_A       = 0.001f;   // ~1 mA/count
constexpr float DXL_VEL_UNIT             = 0.229f;   // rpm/count
constexpr float DXL_POS_COUNTS_PER_REV   = 4096.0f;

// Fall detection threshold [deg]
constexpr float FALL_THRESHOLD_DEG = 40.0f;

// Current limits (A)
constexpr float EEPROM_CURRENT_LIMIT_A    = 1.0f;
constexpr float SW_PEAK_CURRENT_LIMIT_A   = 0.8f;
constexpr float SW_CONT_CURRENT_LIMIT_A   = 0.4f;
constexpr float PEAK_DURATION_S           = 1.0f;
constexpr float CURRENT_SLEW_RATE_A_PER_S = 5.0f;

// Speed guard (rad/s at wheel output shaft)
constexpr float OMEGA_SOFT_RAD_S = 30.0f;
constexpr float OMEGA_HARD_RAD_S = 38.0f;

// Bus Watchdog
constexpr uint32_t BUS_WATCHDOG_TIMEOUT_MS = 200;

// Motor sign convention (body-frame → raw)
constexpr int8_t SIGN_LEFT_MOTOR  = -1;
constexpr int8_t SIGN_RIGHT_MOTOR =  1;

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

// Health monitoring
constexpr uint16_t HEALTH_STALE_LIMIT = 400;  // ticks (2s at 200Hz)
constexpr float VOLTAGE_MIN_V = 3.5f;
constexpr float VOLTAGE_MAX_V = 6.5f;
constexpr uint8_t TEMP_MAX_C  = 65;

// Loop timing guard
constexpr uint32_t LOOP_WARN_US  = 4000;
constexpr uint32_t LOOP_FAULT_US = 4500;
constexpr uint8_t  LOOP_FAULT_COUNT = 3;
constexpr uint32_t SYNC_READ_BUDGET_US = 2600;
