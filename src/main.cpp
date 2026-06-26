#include <Arduino.h>

#include <esp_system.h>
#include <M5Unified.h>
#include <Avatar.h>

#include "TairinEye.h"
#include "TairinMouth.h"

#include <Kalman.h>
#include <Preferences.h>
#include <Dynamixel2Arduino.h>
#include <PS4Controller.h>

HardwareSerial& DXL_SERIAL = Serial1;
#define DEBUG_SERIAL Serial
//#define ENABLE_DEBUG_PRINT
#ifdef ENABLE_DEBUG_PRINT
#define DEBUG_PRINT(...) DEBUG_SERIAL.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) DEBUG_SERIAL.println(__VA_ARGS__)
#define DEBUG_PRINTF(...) DEBUG_SERIAL.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#define DEBUG_PRINTF(...)
#endif


using namespace m5avatar;
Avatar avatar;
Face* tairinFace;

Face* createTairinFace(){
  Face* f;
  f = new Face(
    new tairinMouth(50, 90, 4, 60),
    new tairinEye(8, false),
    new tairinEye(8, true),
    new Eyeblow(32, 0, false),
    new Eyeblow(32, 0, true)
  );
  return f;
}

// M5Stack Core2 PORT A pin configuration
const uint8_t PIN_RX_SERVO = 33;
const uint8_t PIN_TX_SERVO = 32;
const uint32_t BAUD_DXL = 1000000;

// Control Params (Current Control Mode, SI units)
const TickType_t xPeriodMs = 5;
float pitch_target = 86.0f; // [deg] IMU reference angle for upright

const float SOFTWARE_CURRENT_LIMIT_A = 0.35f;
const float SPEED_HARD_RPM = 350.0f;
const float D_LPF_TAU_S = 0.012f;
const int SYNC_READ_DECIMATION = 2;

// VEGA fuzzy controller artifact:
// parent tables/vega_best_mujoco_teleop_200hz_stationhold.npz
const int FUZZY_GRID_N = 11;
const float WHEEL_RADIUS_M = 0.029f;
const float RPM_TO_MPS = WHEEL_RADIUS_M * 2.0f * PI / 60.0f;
const float POSITION_RAW_TO_RAD = 2.0f * PI / 4096.0f;
const float FUZZY_THETA_ERR_MAX_RAD = 0.45f;
const float FUZZY_THETA_RATE_MAX_RAD_S = 4.0f;
const float FUZZY_VEL_ERR_MAX_MPS = 0.4f;
const float FUZZY_VEL_INT_MAX_M = 0.4f;
const float FUZZY_THETA_REF_MAX_RAD = 0.10f;
const float FUZZY_FORCE_LIMIT_NM = 2.0f;
const float FUZZY_MOTOR_GEAR = 0.05239f;
const float FUZZY_CTRL_LIMIT = FUZZY_FORCE_LIMIT_NM / FUZZY_MOTOR_GEAR;
const float FUZZY_CTRL_TO_CURRENT_A = SOFTWARE_CURRENT_LIMIT_A / FUZZY_CTRL_LIMIT;
const float PSI_ERR_MAX_RAD_S = 2.0f;
const float YAW_RATE_SIGN = 1.0f;
const float VELOCITY_CMD_LIMIT_MPS = 0.04f;

const float STATION_HOLD_GAIN = 0.0f;
const float STATION_HOLD_DEADBAND_MPS = 0.005f;
const float STATION_HOLD_SETTLE_MPS = 0.02f;
const float STATION_HOLD_DWELL_S = 0.3f;
const uint32_t PS4_REPORT_PERIOD_MS = 250;
const uint32_t PS4_STATUS_DRAW_PERIOD_MS = 500;
const int PS4_LY_DEADBAND = 8;
const int DXL_PING_RETRIES = 3;
const uint32_t DXL_PING_RETRY_DELAY_MS = 50;

const float T_THETA[FUZZY_GRID_N][FUZZY_GRID_N] = {
  {-42.91900f, -37.72867f, -37.02978f, -42.68309f, -38.84818f, -44.06319f, -11.98498f, -1.037281f, 2.478805f, -10.20923f, -4.862647f},
  {-38.81493f, -32.72190f, -26.34315f, -22.82851f, -31.48345f, -41.34923f, -33.84240f, -18.18435f, -10.12520f, 1.469592f, -0.8395187f},
  {-22.98239f, -16.07554f, -11.24124f, -11.50722f, -23.42779f, -34.49380f, -32.98051f, -20.45271f, -9.722819f, -6.859053f, 5.975559f},
  {-19.91951f, -21.11409f, -19.06324f, -14.76350f, -14.83707f, -28.48312f, -21.29552f, -7.750903f, -5.440594f, -0.5412663f, 6.424665f},
  {-13.31393f, -14.76705f, -12.79264f, -2.833263f, -8.631748f, -10.69216f, -9.399564f, 1.356433f, -1.739972f, 0.05949626f, 11.01979f},
  {5.499407f, 4.497155f, 7.090490f, 1.629094f, 0.6440431f, 3.038283f, 6.275157f, -0.5921091f, 16.08333f, 13.53813f, 12.62891f},
  {-17.50894f, -5.668231f, -1.899337f, 1.250775f, 5.102381f, 5.512905f, 12.58481f, 24.78193f, 28.24998f, 26.18155f, 23.55943f},
  {-19.85215f, -14.37351f, 2.290303f, 4.065852f, 10.09419f, 8.943636f, 20.70283f, 28.49117f, 30.34449f, 35.68570f, 42.31715f},
  {-10.70002f, -4.178781f, 12.99914f, 10.68592f, 15.83191f, 19.06573f, 20.62531f, 29.21165f, 28.17976f, 34.39820f, 42.85419f},
  {0.3900879f, 8.668550f, 17.22789f, 18.52262f, 21.66521f, 22.25903f, 30.53985f, 35.14948f, 32.45472f, 40.60931f, 43.84497f},
  {-0.005025197f, 14.40961f, 26.98668f, 24.72663f, 26.35819f, 23.61532f, 37.60464f, 48.65728f, 46.93891f, 41.42734f, 37.23830f}
};

const float T_V[FUZZY_GRID_N][FUZZY_GRID_N] = {
  {-0.104852f, -0.092249f, -0.076703f, -0.059422f, -0.039717f, -0.030578f, -0.020800f, -0.003485f, 0.009463f, 0.032928f, 0.039619f},
  {-0.091148f, -0.083943f, -0.072039f, -0.054912f, -0.036888f, -0.021841f, -0.014151f, 0.002059f, 0.009627f, 0.032683f, 0.040526f},
  {-0.098120f, -0.082264f, -0.068234f, -0.052351f, -0.039282f, -0.020909f, -0.003365f, 0.011836f, 0.024336f, 0.041944f, 0.057577f},
  {-0.084250f, -0.073367f, -0.055748f, -0.044139f, -0.024388f, -0.015261f, 0.000972f, 0.021351f, 0.026627f, 0.041652f, 0.062170f},
  {-0.077960f, -0.059697f, -0.045542f, -0.038358f, -0.019133f, -0.010173f, 0.010685f, 0.016908f, 0.038574f, 0.049537f, 0.059978f},
  {-0.065151f, -0.057634f, -0.038806f, -0.031081f, -0.014166f, -0.003832f, 0.015664f, 0.025025f, 0.037191f, 0.062323f, 0.078983f},
  {-0.065373f, -0.055877f, -0.034036f, -0.018230f, -0.001841f, 0.008440f, 0.019870f, 0.026451f, 0.058665f, 0.068043f, 0.080041f},
  {-0.063712f, -0.051660f, -0.028649f, -0.013714f, -0.005344f, 0.005541f, 0.029469f, 0.039656f, 0.056040f, 0.068294f, 0.088749f},
  {-0.052739f, -0.030207f, -0.022939f, -0.007095f, 0.008675f, 0.022005f, 0.035369f, 0.048638f, 0.061824f, 0.082100f, 0.091227f},
  {-0.053940f, -0.025732f, -0.016989f, -0.003766f, 0.006203f, 0.016467f, 0.040668f, 0.053314f, 0.067450f, 0.075496f, 0.096127f},
  {-0.039641f, -0.028388f, -0.012503f, 0.003454f, 0.019092f, 0.034224f, 0.047626f, 0.059513f, 0.085093f, 0.093541f, 0.106381f}
};

const float T_PSI[FUZZY_GRID_N] = {
  -4.699511f, -3.647752f, -2.776538f, -2.454664f, -1.362237f, 0.119531f,
  1.156429f, 1.404798f, 2.536859f, 3.887653f, 4.696179f
};

// Fuzzy controller state
float theta_dot_lpf = 0.0f;
float fuzzy_vel_integral = 0.0f;
float fuzzy_current_scale = 0.75f;
float velocity_cmd_mps = 0.0f;

volatile bool ps4_connect_event = false;
volatile bool ps4_disconnect_event = false;
bool ps4_enabled = false;
bool ps4_connected = false;
bool ps4_status_dirty = true;
unsigned long ps4_last_report_ms = 0;
unsigned long ps4_last_status_draw_ms = 0;
int ps4_ly = 0;
float ps4_velocity_cmd_mps = 0.0f;

// Sync Read: Present Velocity + Present Position (addr 128, 8 bytes) x2 motors
const uint16_t ADDR_PRESENT_VELOCITY = 128;
const uint16_t LEN_PRESENT_STATE = 8;
const float VEL_RAW_TO_RPM = 0.229f;
uint8_t sr_recv_buf_L[8];
uint8_t sr_recv_buf_R[8];
DYNAMIXEL::XELInfoSyncRead_t sr_xels[2];
DYNAMIXEL::InfoSyncReadInst_t sr_info;

// Sync Write: Goal Current (addr 102, 2 bytes) x2 motors
const uint16_t ADDR_GOAL_CURRENT = 102;
const uint16_t LEN_GOAL_CURRENT = 2;
uint8_t sw_data_L[2];
uint8_t sw_data_R[2];
DYNAMIXEL::XELInfoSyncWrite_t sw_xels[2];
DYNAMIXEL::InfoSyncWriteInst_t sw_info;

// Telemetry (20Hz output)
const int TELEM_DECIMATION = 10;
int telem_counter = 0;
float telem_vel_L = 0.0f;
float telem_vel_R = 0.0f;
float telem_v_fwd = 0.0f;
float telem_odom_x = 0.0f;
float telem_theta_ref = 0.0f;
float telem_theta_ref_trim = 0.0f;
float telem_ctrl_base = 0.0f;
float telem_ctrl_yaw = 0.0f;
float telem_ctrl_L = 0.0f;
float telem_ctrl_R = 0.0f;
float telem_current_L = 0.0f;
float telem_current_R = 0.0f;
float telem_yaw_rate = 0.0f;
int telem_hold_state = 0;

bool odom_initialized = false;
int32_t pos_origin_L = 0;
int32_t pos_origin_R = 0;
float station_hold_x = 0.0f;
bool station_hold_has_x = false;
float station_hold_dwell_s = 0.0f;

// Kalman filter
Kalman kalman;
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

unsigned long previousTimestamp = 0;

// DYNAMIXEL Params.
const uint8_t DXL_ID_L = 0;
const uint8_t DXL_ID_R = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl;//(DXL_SERIAL);


float getPitch(){
  M5.Imu.getAccelData(&accX, &accY, &accZ);
  float pitch = atan2(accY, accZ) * RAD_TO_DEG; //[deg]
  return pitch;
}


float getPitchDot() {
  M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
  float pitch_dot = gyroX;
  return pitch_dot;  //[deg/sec]
}


float bilinearInterp(
    const float table[FUZZY_GRID_N][FUZZY_GRID_N],
    float x,
    float y,
    float x_min,
    float x_max,
    float y_min,
    float y_max) {
  x = constrain(x, x_min, x_max);
  y = constrain(y, y_min, y_max);

  float i_f = (x - x_min) / (x_max - x_min) * (FUZZY_GRID_N - 1);
  float j_f = (y - y_min) / (y_max - y_min) * (FUZZY_GRID_N - 1);
  int i0 = (int)i_f;
  int j0 = (int)j_f;
  if (i0 > FUZZY_GRID_N - 2) i0 = FUZZY_GRID_N - 2;
  if (j0 > FUZZY_GRID_N - 2) j0 = FUZZY_GRID_N - 2;
  float a = i_f - (float)i0;
  float b = j_f - (float)j0;

  return (1.0f - a) * (1.0f - b) * table[i0][j0]
      + a * (1.0f - b) * table[i0 + 1][j0]
      + (1.0f - a) * b * table[i0][j0 + 1]
      + a * b * table[i0 + 1][j0 + 1];
}


float interp1D(const float table[FUZZY_GRID_N], float x, float x_max) {
  x = constrain(x, -x_max, x_max);
  float i_f = (x + x_max) / (2.0f * x_max) * (FUZZY_GRID_N - 1);
  int i0 = (int)i_f;
  if (i0 > FUZZY_GRID_N - 2) i0 = FUZZY_GRID_N - 2;
  float a = i_f - (float)i0;
  return (1.0f - a) * table[i0] + a * table[i0 + 1];
}


void resetFuzzyState() {
  theta_dot_lpf = 0.0f;
  fuzzy_vel_integral = 0.0f;
  station_hold_has_x = false;
  station_hold_dwell_s = 0.0f;
  telem_theta_ref = 0.0f;
  telem_theta_ref_trim = 0.0f;
  telem_ctrl_base = 0.0f;
  telem_ctrl_yaw = 0.0f;
  telem_ctrl_L = 0.0f;
  telem_ctrl_R = 0.0f;
  telem_yaw_rate = 0.0f;
  telem_hold_state = 0;
}


void handlePS4Connect() {
  ps4_connect_event = true;
}


void handlePS4Disconnect() {
  ps4_disconnect_event = true;
}


uint16_t ps4ButtonMask() {
  uint16_t mask = 0;
  if (PS4.Up()) mask |= 1 << 0;
  if (PS4.Down()) mask |= 1 << 1;
  if (PS4.Left()) mask |= 1 << 2;
  if (PS4.Right()) mask |= 1 << 3;
  if (PS4.Square()) mask |= 1 << 4;
  if (PS4.Cross()) mask |= 1 << 5;
  if (PS4.Circle()) mask |= 1 << 6;
  if (PS4.Triangle()) mask |= 1 << 7;
  if (PS4.L1()) mask |= 1 << 8;
  if (PS4.R1()) mask |= 1 << 9;
  if (PS4.Share()) mask |= 1 << 10;
  if (PS4.Options()) mask |= 1 << 11;
  if (PS4.L3()) mask |= 1 << 12;
  if (PS4.R3()) mask |= 1 << 13;
  if (PS4.PSButton()) mask |= 1 << 14;
  if (PS4.Touchpad()) mask |= 1 << 15;
  return mask;
}


void setupPS4Controller() {
  uint8_t bt_mac[6];
  esp_read_mac(bt_mac, ESP_MAC_BT);
  DEBUG_SERIAL.printf("Bluetooth Mac Address => %02X:%02X:%02X:%02X:%02X:%02X\n",
      bt_mac[0], bt_mac[1], bt_mac[2], bt_mac[3], bt_mac[4], bt_mac[5]);

  PS4.attachOnConnect(handlePS4Connect);
  PS4.attachOnDisconnect(handlePS4Disconnect);
  ps4_enabled = PS4.begin();

  if (ps4_enabled) {
    DEBUG_SERIAL.println("PS4 controller host ready.");
    DEBUG_SERIAL.println("Please pair the gamepad to the Bluetooth MAC above and press HOME.");
  } else {
    DEBUG_SERIAL.println("PS4 controller host init failed.");
  }
}


float velocityCommandFromPS4Ly(int ly) {
  ly = constrain(ly, -127, 127);

  int abs_ly = ly >= 0 ? ly : -ly;
  if (abs_ly <= PS4_LY_DEADBAND) {
    return 0.0f;
  }

  float magnitude = (float)(abs_ly - PS4_LY_DEADBAND) / (float)(127 - PS4_LY_DEADBAND);
  float sign = ly >= 0 ? 1.0f : -1.0f;
  return sign * magnitude * VELOCITY_CMD_LIMIT_MPS;
}


void clearPS4VelocityCommand() {
  ps4_ly = 0;
  ps4_velocity_cmd_mps = 0.0f;
  velocity_cmd_mps = 0.0f;
  ps4_status_dirty = true;
}


void updatePS4VelocityCommand() {
  ps4_ly = constrain((int)PS4.LStickY(), -127, 127);
  ps4_velocity_cmd_mps = velocityCommandFromPS4Ly(ps4_ly);
  velocity_cmd_mps = ps4_velocity_cmd_mps;
}


bool pingDynamixels() {
  bool ping_L = false;
  bool ping_R = false;

  for (int i = 0; i < DXL_PING_RETRIES; ++i) {
    ping_L = dxl.ping(DXL_ID_L);
    ping_R = dxl.ping(DXL_ID_R);

    DEBUG_PRINT("ping try ");
    DEBUG_PRINT(i + 1);
    DEBUG_PRINT(" L: ");
    DEBUG_PRINT(ping_L);
    DEBUG_PRINT(", R: ");
    DEBUG_PRINTLN(ping_R);

    if (ping_L && ping_R) {
      return true;
    }
    delay(DXL_PING_RETRY_DELAY_MS);
  }

  return false;
}


void pollPS4Controller() {
  if (!ps4_enabled) {
    return;
  }

  if (ps4_connect_event) {
    ps4_connect_event = false;
    ps4_connected = true;
    ps4_status_dirty = true;
    clearPS4VelocityCommand();
    DEBUG_SERIAL.println("PS4 controller connected");
    PS4.setLed(0, 32, 64);
    PS4.sendToController();
  }

  if (ps4_disconnect_event) {
    ps4_disconnect_event = false;
    ps4_connected = false;
    clearPS4VelocityCommand();
    DEBUG_SERIAL.println("PS4 controller disconnected");
  }

  bool connected_now = PS4.isConnected();
  if (connected_now != ps4_connected) {
    ps4_connected = connected_now;
    ps4_status_dirty = true;
    if (!ps4_connected) {
      clearPS4VelocityCommand();
    }
    DEBUG_SERIAL.println(ps4_connected ? "PS4 controller connected" : "PS4 controller disconnected");
  }

  if (!ps4_connected) {
    return;
  }

  updatePS4VelocityCommand();

  unsigned long now = millis();
  if (now - ps4_last_report_ms < PS4_REPORT_PERIOD_MS) {
    return;
  }
  ps4_last_report_ms = now;

  DEBUG_SERIAL.printf("PS4,vcmd=%.4f\n", ps4_velocity_cmd_mps);

  // Raw PS4 input dump. Re-enable when diagnosing controller packets.
  // DEBUG_PRINTF(
  //     "PS4,bat=%u,lx=%d,ly=%d,vcmd=%.4f,rx=%d,ry=%d,l2=%u,r2=%u,buttons=0x%04X\n",
  //     PS4.Battery(),
  //     PS4.LStickX(),
  //     ps4_ly,
  //     ps4_velocity_cmd_mps,
  //     PS4.RStickX(),
  //     PS4.RStickY(),
  //     PS4.L2Value(),
  //     PS4.R2Value(),
  //     ps4ButtonMask());
}


float stationHoldTrim(float velocity_cmd, float x, float v, float dt) {
  if (STATION_HOLD_GAIN <= 0.0f) {
    station_hold_has_x = false;
    station_hold_dwell_s = 0.0f;
    telem_hold_state = 0;
    return 0.0f;
  }

  if (!odom_initialized) {
    station_hold_has_x = false;
    station_hold_dwell_s = 0.0f;
    telem_hold_state = 0;
    return 0.0f;
  }

  if (fabsf(velocity_cmd) >= STATION_HOLD_DEADBAND_MPS) {
    station_hold_has_x = false;
    station_hold_dwell_s = 0.0f;
    telem_hold_state = 0;
    return 0.0f;
  }

  if (station_hold_has_x) {
    telem_hold_state = 2;
    return -STATION_HOLD_GAIN * (x - station_hold_x);
  }

  telem_hold_state = 1;
  if (fabsf(v) < STATION_HOLD_SETTLE_MPS) {
    station_hold_dwell_s += dt;
    if (station_hold_dwell_s >= STATION_HOLD_DWELL_S) {
      station_hold_x = x;
      station_hold_has_x = true;
      station_hold_dwell_s = 0.0f;
      telem_hold_state = 2;
    }
  } else {
    station_hold_dwell_s = 0.0f;
  }
  return 0.0f;
}


void driveCurrents(float left_current_A, float right_current_A) {
  left_current_A = constrain(left_current_A, -SOFTWARE_CURRENT_LIMIT_A, SOFTWARE_CURRENT_LIMIT_A);
  right_current_A = constrain(right_current_A, -SOFTWARE_CURRENT_LIMIT_A, SOFTWARE_CURRENT_LIMIT_A);

  int16_t mA_L = (int16_t)lroundf(-left_current_A * 1000.0f);
  int16_t mA_R = (int16_t)lroundf( right_current_A * 1000.0f);
  memcpy(sw_data_L, &mA_L, 2);
  memcpy(sw_data_R, &mA_R, 2);
  telem_current_L = left_current_A;
  telem_current_R = right_current_A;
  dxl.syncWrite(&sw_info);
}


void driveCurrent(float current_A) {
  driveCurrents(current_A, current_A);
}


float motorCtrlToCurrent(float ctrl) {
  float ctrl_limited = constrain(ctrl, -FUZZY_CTRL_LIMIT, FUZZY_CTRL_LIMIT);
  float scale = constrain(fuzzy_current_scale, 0.0f, 1.0f);
  return ctrl_limited * FUZZY_CTRL_TO_CURRENT_A * scale;
}


void driveMotorCtrl(float ctrl_L, float ctrl_R) {
  ctrl_L = constrain(ctrl_L, -FUZZY_CTRL_LIMIT, FUZZY_CTRL_LIMIT);
  ctrl_R = constrain(ctrl_R, -FUZZY_CTRL_LIMIT, FUZZY_CTRL_LIMIT);
  telem_ctrl_L = ctrl_L;
  telem_ctrl_R = ctrl_R;
  driveCurrents(motorCtrlToCurrent(ctrl_L), motorCtrlToCurrent(ctrl_R));
}


void calcFuzzy(){
  unsigned long now = micros();
  float dt = (float)(now - previousTimestamp) / 1000000.0f;
  previousTimestamp = now;

  if (dt <= 0.0f || dt > 0.05f) {
    driveCurrent(0.0f);
    resetFuzzyState();
    return;
  }

  float pitch_kalman = kalman.getAngle(getPitch(), getPitchDot(), dt);
  float pitch_dot_kalman = kalman.getRate();

  float pitch_error_deg = pitch_target - pitch_kalman;

  if (pitch_error_deg < -40.0f || 40.0f < pitch_error_deg) {
    driveCurrent(0.0f);
    resetFuzzyState();
    return;
  }

  static int sync_read_skip = 0;
  if (++sync_read_skip >= SYNC_READ_DECIMATION && dxl.syncRead(&sr_info, 3) == 2) {
    sync_read_skip = 0;
    int32_t raw_vel_L, raw_vel_R;
    int32_t raw_pos_L, raw_pos_R;
    memcpy(&raw_vel_L, sr_recv_buf_L, 4);
    memcpy(&raw_vel_R, sr_recv_buf_R, 4);
    memcpy(&raw_pos_L, sr_recv_buf_L + 4, 4);
    memcpy(&raw_pos_R, sr_recv_buf_R + 4, 4);
    float vel_L = (float)raw_vel_L * VEL_RAW_TO_RPM;
    float vel_R = (float)raw_vel_R * VEL_RAW_TO_RPM;
    telem_vel_L = vel_L;
    telem_vel_R = vel_R;
    if (fabsf(vel_L) > SPEED_HARD_RPM || fabsf(vel_R) > SPEED_HARD_RPM) {
      driveCurrent(0.0f);
      resetFuzzyState();
      return;
    }

    float v_forward_rpm = (-vel_L + vel_R) * 0.5f;
    telem_v_fwd = v_forward_rpm * RPM_TO_MPS;

    if (!odom_initialized) {
      pos_origin_L = raw_pos_L;
      pos_origin_R = raw_pos_R;
      odom_initialized = true;
    }
    float phi_L = -(float)(raw_pos_L - pos_origin_L) * POSITION_RAW_TO_RAD;
    float phi_R =  (float)(raw_pos_R - pos_origin_R) * POSITION_RAW_TO_RAD;
    telem_odom_x = WHEEL_RADIUS_M * (phi_L + phi_R) * 0.5f;
  }

  float theta = pitch_error_deg * DEG_TO_RAD;
  float theta_dot = -pitch_dot_kalman * DEG_TO_RAD;

  float alpha = dt / (D_LPF_TAU_S + dt);
  theta_dot_lpf += alpha * (theta_dot - theta_dot_lpf);

  float v_cmd = constrain(velocity_cmd_mps, -VELOCITY_CMD_LIMIT_MPS, VELOCITY_CMD_LIMIT_MPS);
  float v_err = v_cmd - telem_v_fwd;
  fuzzy_vel_integral = constrain(
      fuzzy_vel_integral + v_err * dt,
      -FUZZY_VEL_INT_MAX_M,
      FUZZY_VEL_INT_MAX_M);

  telem_theta_ref_trim = stationHoldTrim(v_cmd, telem_odom_x, telem_v_fwd, dt);
  float theta_ref = bilinearInterp(
      T_V,
      v_err,
      fuzzy_vel_integral,
      -FUZZY_VEL_ERR_MAX_MPS,
      FUZZY_VEL_ERR_MAX_MPS,
      -FUZZY_VEL_INT_MAX_M,
      FUZZY_VEL_INT_MAX_M);
  theta_ref += telem_theta_ref_trim;
  theta_ref = constrain(theta_ref, -FUZZY_THETA_REF_MAX_RAD, FUZZY_THETA_REF_MAX_RAD);
  telem_theta_ref = theta_ref;

  float theta_err = theta - theta_ref;
  float ctrl_base = bilinearInterp(
      T_THETA,
      theta_err,
      theta_dot_lpf,
      -FUZZY_THETA_ERR_MAX_RAD,
      FUZZY_THETA_ERR_MAX_RAD,
      -FUZZY_THETA_RATE_MAX_RAD_S,
      FUZZY_THETA_RATE_MAX_RAD_S);
  ctrl_base = constrain(ctrl_base, -FUZZY_CTRL_LIMIT, FUZZY_CTRL_LIMIT);
  telem_ctrl_base = ctrl_base;

  telem_yaw_rate = YAW_RATE_SIGN * gyroZ * DEG_TO_RAD;
  telem_ctrl_yaw = 0.5f * interp1D(T_PSI, -telem_yaw_rate, PSI_ERR_MAX_RAD_S);
  float ctrl_L = constrain(ctrl_base - telem_ctrl_yaw, -FUZZY_CTRL_LIMIT, FUZZY_CTRL_LIMIT);
  float ctrl_R = constrain(ctrl_base + telem_ctrl_yaw, -FUZZY_CTRL_LIMIT, FUZZY_CTRL_LIMIT);

  driveMotorCtrl(ctrl_L, ctrl_R);

  if (++telem_counter >= TELEM_DECIMATION) {
    telem_counter = 0;
    DEBUG_PRINTF("T,%lu,%.0f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.1f,%.1f,%d\n",
        millis(), dt * 1000000.0f, pitch_kalman,
        theta, theta_ref, telem_theta_ref_trim, theta_dot_lpf,
        telem_v_fwd, v_cmd, fuzzy_vel_integral, telem_odom_x,
        ctrl_base, telem_ctrl_yaw, telem_ctrl_L, telem_yaw_rate,
        telem_current_L, telem_current_R,
        telem_vel_L, telem_vel_R, telem_hold_state);
  }
}


void drawButton(const char* label, int x, int y, float value) {
  // draw button
  M5.Display.drawRect(x, y, 50, 30, WHITE);
  M5.Display.drawString("-", x+20, y, 2);

  M5.Display.drawRect(x+200, y, 50, 30, WHITE);
  M5.Display.drawString("+", x+200+20, y, 2);

  // draw label and value
  M5.Display.setCursor(x+70, y+5);
  M5.Display.print(label);
  M5.Display.print(": ");
  M5.Display.println(value, 2);
}


void drawPS4Status() {
  M5.Display.fillRect(20, 150, 290, 24, BLACK);
  M5.Display.setCursor(20, 150);

  if (!ps4_enabled) {
    M5.Display.print("PS4: init failed");
  } else if (ps4_connected) {
    M5.Display.printf("PS4: Bat:%u LY:%d", PS4.Battery(), ps4_ly);
  } else {
    M5.Display.print("PS4: waiting HOME");
  }
}


void drawCtrlPanel(){
  drawButton("Ref", 20, 30, pitch_target);
  drawButton("Scale", 20, 70, fuzzy_current_scale);
  drawButton("Vcmd", 20, 110, velocity_cmd_mps);

  drawPS4Status();

  M5.Display.fillRect(20, 190, 290, 24, BLACK);
  M5.Display.setCursor(20, 190);
  int batteryPercentage = M5.Power.getBatteryLevel();
  M5.Display.printf("M5Core2 Battery: %d%%", batteryPercentage);
}


void handleCtrlPanelTouched(){

  auto pos = M5.Touch.getDetail();

  if (pos.y >= 30 && pos.y < 60) {
    if (pos.x >= 15 && pos.x < 120) pitch_target -= 0.2f;
    else if (pos.x >= 220 && pos.x < 270) pitch_target += 0.2f;
    drawButton("Ref", 20, 30, pitch_target);
  }
  else if (pos.y >= 70 && pos.y < 100) {
    if (pos.x >= 15 && pos.x < 120) fuzzy_current_scale -= 0.05f;
    else if (pos.x >= 220 && pos.x < 270) fuzzy_current_scale += 0.05f;
    fuzzy_current_scale = constrain(fuzzy_current_scale, 0.0f, 1.0f);
    drawButton("Scale", 20, 70, fuzzy_current_scale);
  }
  else if (pos.y >= 110 && pos.y < 140) {
    if (pos.x >= 15 && pos.x < 120) velocity_cmd_mps -= 0.005f;
    else if (pos.x >= 220 && pos.x < 270) velocity_cmd_mps += 0.005f;
    velocity_cmd_mps = constrain(velocity_cmd_mps, -VELOCITY_CMD_LIMIT_MPS, VELOCITY_CMD_LIMIT_MPS);
    drawButton("Vcmd", 20, 110, velocity_cmd_mps);
  }
}


void controlLoopTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(true) {
      calcFuzzy();
      vTaskDelayUntil(&xLastWakeTime, xPeriodMs);
    }
}


bool enShowCtrlPanel = false;
void uiLoopTask(void *pvParameters){
  
  TickType_t xLastWakeTime = xTaskGetTickCount();  
  while(true){
    M5.update();
    pollPS4Controller();

    //draw avatar face
    if (M5.BtnA.wasPressed()) {
      avatar.resume();
    }

    // show tuning panel
    if (M5.BtnB.wasPressed()) {
      if (enShowCtrlPanel) {
        enShowCtrlPanel = false;
        avatar.resume();
        M5.Lcd.clear();
      } else {
        enShowCtrlPanel = true;
        avatar.suspend();
        M5.Lcd.clear();
        drawCtrlPanel();
      }
    }
    
    if (enShowCtrlPanel) {
      unsigned long now = millis();
      if (ps4_status_dirty || now - ps4_last_status_draw_ms >= PS4_STATUS_DRAW_PERIOD_MS) {
        drawPS4Status();
        if (ps4_connected) {
          drawButton("Vcmd", 20, 110, velocity_cmd_mps);
        }
        ps4_status_dirty = false;
        ps4_last_status_draw_ms = now;
      }

      bool isReleased = true;
        if (M5.Touch.getCount()>0 && isReleased) {
          isReleased = false;
          handleCtrlPanelTouched();
        }
      }
    vTaskDelayUntil(&xLastWakeTime, xPeriodMs*5);
  }
}


void setup(){

  DEBUG_SERIAL.begin(115200);

  // M5 Settings
  M5.begin();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);
  M5.Imu.init();
  // M5.Imu.setAccelFsr(M5.Imu.AFS_2G);
  // M5.Imu.setGyroFsr(M5.Imu.GFS_250DPS);

  // M5 avatar Settings
  tairinFace = createTairinFace();
  avatar.setFace(tairinFace);
  avatar.init();

  // DYNAMIXEL Settings
  DXL_SERIAL.begin(BAUD_DXL, SERIAL_8N1, PIN_RX_SERVO, PIN_TX_SERVO);
  dxl = Dynamixel2Arduino(DXL_SERIAL);
  dxl.begin(BAUD_DXL);

  DEBUG_PRINTLN("DYNAMIXEL ping Waiting...");

  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  if (!pingDynamixels()) {
    DEBUG_SERIAL.println("DYNAMIXEL ping failed!");
    M5.Lcd.println("DYNAMIXEL ping failed!");
    while (true) {
      pollPS4Controller();
      delay(25);
    }
  }

  DEBUG_PRINTLN("DYNAMIXEL ping OK");

  dxl.torqueOff(DXL_ID_L);
  dxl.torqueOff(DXL_ID_R);
  delay(20);

  dxl.setOperatingMode(DXL_ID_L, OP_CURRENT);
  dxl.setOperatingMode(DXL_ID_R, OP_CURRENT);
  delay(20);

  dxl.setGoalCurrent(DXL_ID_L, 0, UNIT_MILLI_AMPERE);
  dxl.setGoalCurrent(DXL_ID_R, 0, UNIT_MILLI_AMPERE);
  delay(20);

  dxl.torqueOn(DXL_ID_L);
  dxl.torqueOn(DXL_ID_R);
  delay(20);

  dxl.setGoalCurrent(DXL_ID_L, 0, UNIT_MILLI_AMPERE);
  dxl.setGoalCurrent(DXL_ID_R, 0, UNIT_MILLI_AMPERE);

  // Sync Read setup (Present Velocity)
  memset(&sr_info, 0, sizeof(sr_info));
  memset(sr_xels, 0, sizeof(sr_xels));
  sr_xels[0].id = DXL_ID_L;
  sr_xels[0].p_recv_buf = sr_recv_buf_L;
  sr_xels[1].id = DXL_ID_R;
  sr_xels[1].p_recv_buf = sr_recv_buf_R;
  sr_info.p_xels = sr_xels;
  sr_info.xel_count = 2;
  sr_info.addr = ADDR_PRESENT_VELOCITY;
  sr_info.addr_length = LEN_PRESENT_STATE;
  sr_info.is_info_changed = true;

  // Sync Write setup (Goal Current)
  memset(&sw_info, 0, sizeof(sw_info));
  memset(sw_xels, 0, sizeof(sw_xels));
  sw_xels[0].id = DXL_ID_L;
  sw_xels[0].p_data = sw_data_L;
  sw_xels[1].id = DXL_ID_R;
  sw_xels[1].p_data = sw_data_R;
  sw_info.p_xels = sw_xels;
  sw_info.xel_count = 2;
  sw_info.addr = ADDR_GOAL_CURRENT;
  sw_info.addr_length = LEN_GOAL_CURRENT;
  sw_info.is_info_changed = true;

  // Kalman filter Setting
  kalman.setAngle(getPitch());

  // PS4 controller host setup
  setupPS4Controller();

  DEBUG_PRINTLN("# Controller: VEGA fuzzy grid");
  DEBUG_PRINTLN("# Artifact: tables/vega_best_mujoco_teleop_200hz_stationhold.npz");
  DEBUG_PRINTF("# ctrl_limit=%f, ctrl_to_current_A=%f, current_limit_A=%f\n",
      FUZZY_CTRL_LIMIT, FUZZY_CTRL_TO_CURRENT_A, SOFTWARE_CURRENT_LIMIT_A);
  DEBUG_PRINTF("# real_tune: current_scale=%f, theta_err_max=%f, theta_rate_max=%f, d_lpf_tau=%f, station_hold_gain=%f\n",
      fuzzy_current_scale, FUZZY_THETA_ERR_MAX_RAD, FUZZY_THETA_RATE_MAX_RAD_S,
      D_LPF_TAU_S, STATION_HOLD_GAIN);
  DEBUG_PRINTLN("T,t_ms,dt_us,pitch,theta,theta_ref,theta_trim,theta_dot,v_mps,v_cmd,vel_int,odom_x,ctrl_base,ctrl_yaw,ctrl_L,yaw_rate,current_L,current_R,vel_L_rpm,vel_R_rpm,hold_state");
  
  // RTOS Task Settings
  const uint32_t MEMORY_STACK = 8192;
  const UBaseType_t PRIORIRY_CTRL = 5;
  const BaseType_t CORE_CTRL = 1;
  xTaskCreatePinnedToCore(controlLoopTask, "Control Loop Task", MEMORY_STACK, NULL, PRIORIRY_CTRL, NULL, CORE_CTRL);

  const UBaseType_t PRIORIRY_UI = 2;
  const BaseType_t CORE_UI = 0;
  xTaskCreatePinnedToCore(uiLoopTask,      "UI Loop Task",      MEMORY_STACK, NULL, PRIORIRY_UI,   NULL, CORE_UI);
}


void loop(){
  
}
