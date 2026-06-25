#include <Arduino.h>

#include <M5Unified.h>
#include <Avatar.h>

#include "TairinEye.h"
#include "TairinMouth.h"

#include <Kalman.h>
#include <Preferences.h>
#include <Dynamixel2Arduino.h>

HardwareSerial& DXL_SERIAL = Serial1;
#define DEBUG_SERIAL Serial
//#define ENABLE_DEBUG_PRINT


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
float Kp = 0.50f;   // [A/rad]
float Ki = 0.0f;     // [A/(rad*s)]
float Kd = 0.04f;   // [A/(rad/s)]
float pitch_target = 86.0f; // [deg] IMU reference angle for upright

const float SOFTWARE_CURRENT_LIMIT_A = 0.35f;
const float SPEED_HARD_RPM = 350.0f;
const float D_LPF_TAU_S = 0.008f;
const int SYNC_READ_DECIMATION = 2;

// Inner PID state
float z_theta = 0.0f;
float theta_dot_lpf = 0.0f;

// Outer velocity PI params
float Kp_vel = 0.02f;    // [deg/RPM]
float Ki_vel = 0.005f;   // [deg/(RPM*s)]
const float THETA_REF_MAX_DEG = 3.0f;
const int VEL_PI_DECIMATION = 20;

// Outer velocity PI state
float vel_pi_integral = 0.0f;
float vel_pitch_offset_deg = 0.0f;
int vel_pi_counter = 0;
float vel_pi_dt_acc = 0.0f;

// Sync Read: Present Velocity (addr 128, 4 bytes) x2 motors
const uint16_t ADDR_PRESENT_VELOCITY = 128;
const uint16_t LEN_PRESENT_VELOCITY = 4;
const float VEL_RAW_TO_RPM = 0.229f;
uint8_t sr_recv_buf_L[4];
uint8_t sr_recv_buf_R[4];
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


void driveCurrent(float current_A) {
  int16_t mA_L = (int16_t)lroundf(-current_A * 1000.0f);
  int16_t mA_R = (int16_t)lroundf( current_A * 1000.0f);
  memcpy(sw_data_L, &mA_L, 2);
  memcpy(sw_data_R, &mA_R, 2);
  dxl.syncWrite(&sw_info);
}


void calcPID(){
  unsigned long now = micros();
  float dt = (float)(now - previousTimestamp) / 1000000.0f;
  previousTimestamp = now;

  if (dt <= 0.0f || dt > 0.05f) {
    driveCurrent(0.0f);
    return;
  }

  float pitch_kalman = kalman.getAngle(getPitch(), getPitchDot(), dt);
  float pitch_dot_kalman = kalman.getRate();

  float pitch_error_deg = pitch_target - pitch_kalman;

  if (pitch_error_deg < -40.0f || 40.0f < pitch_error_deg) {
    driveCurrent(0.0f);
    z_theta = 0.0f;
    theta_dot_lpf = 0.0f;
    vel_pi_integral = 0.0f;
    vel_pitch_offset_deg = 0.0f;
    vel_pi_dt_acc = 0.0f;
    vel_pi_counter = 0;
    return;
  }

  vel_pi_dt_acc += dt;

  static int sync_read_skip = 0;
  if (++sync_read_skip >= SYNC_READ_DECIMATION && dxl.syncRead(&sr_info, 3) == 2) {
    sync_read_skip = 0;
    int32_t raw_L, raw_R;
    memcpy(&raw_L, sr_recv_buf_L, 4);
    memcpy(&raw_R, sr_recv_buf_R, 4);
    float vel_L = (float)raw_L * VEL_RAW_TO_RPM;
    float vel_R = (float)raw_R * VEL_RAW_TO_RPM;
    telem_vel_L = vel_L;
    telem_vel_R = vel_R;
    if (fabsf(vel_L) > SPEED_HARD_RPM || fabsf(vel_R) > SPEED_HARD_RPM) {
      driveCurrent(0.0f);
      z_theta = 0.0f;
      theta_dot_lpf = 0.0f;
      vel_pi_integral = 0.0f;
      vel_pitch_offset_deg = 0.0f;
      vel_pi_dt_acc = 0.0f;
      vel_pi_counter = 0;
      return;
    }

    if (++vel_pi_counter >= VEL_PI_DECIMATION) {
      vel_pi_counter = 0;
      float dt_vel = vel_pi_dt_acc;
      vel_pi_dt_acc = 0.0f;

      float v_forward_rpm = (-vel_L + vel_R) * 0.5f;
      telem_v_fwd = v_forward_rpm;
      float v_error = -v_forward_rpm;

      vel_pi_integral += v_error * dt_vel;
      float offset_raw = Kp_vel * v_error + Ki_vel * vel_pi_integral;
      vel_pitch_offset_deg = constrain(offset_raw, -THETA_REF_MAX_DEG, THETA_REF_MAX_DEG);

      if (offset_raw != vel_pitch_offset_deg) {
        vel_pi_integral -= v_error * dt_vel;
      }
    }
  }

  float e_theta = (pitch_error_deg + vel_pitch_offset_deg) * DEG_TO_RAD;
  float theta_dot = -pitch_dot_kalman * DEG_TO_RAD;

  float alpha = dt / (D_LPF_TAU_S + dt);
  theta_dot_lpf += alpha * (theta_dot - theta_dot_lpf);

  float i_unsat = Kp * e_theta + Kd * theta_dot_lpf + Ki * z_theta;
  float i_sat = constrain(i_unsat, -SOFTWARE_CURRENT_LIMIT_A, SOFTWARE_CURRENT_LIMIT_A);

  bool sat_hi = i_unsat >  SOFTWARE_CURRENT_LIMIT_A;
  bool sat_lo = i_unsat < -SOFTWARE_CURRENT_LIMIT_A;
  bool drives_out = (sat_hi && e_theta < 0.0f) || (sat_lo && e_theta > 0.0f);
  if ((!sat_hi && !sat_lo) || drives_out) {
    z_theta += e_theta * dt;
  }
  if (fabsf(Ki) > 1e-6f) {
    z_theta += 0.2f * (i_sat - i_unsat) / Ki;
  }

  driveCurrent(i_sat);

  if (++telem_counter >= TELEM_DECIMATION) {
    telem_counter = 0;
    DEBUG_SERIAL.printf("T,%lu,%.0f,%.2f,%.2f,%.2f,%.3f,%.1f,%.1f,%.1f,%.4f,%.4f,%.4f,%.4f\n",
        millis(), dt * 1000000.0f, pitch_kalman, pitch_error_deg,
        vel_pitch_offset_deg, theta_dot_lpf,
        telem_vel_L, telem_vel_R, telem_v_fwd,
        i_unsat, i_sat, z_theta, vel_pi_integral);
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


void drawCtrlPanel(){
  drawButton("Ref", 20, 30, pitch_target);
  drawButton("Kp", 20, 70, Kp);
  drawButton("Ki", 20, 110, Ki);
  drawButton("Kd", 20, 150, Kd);

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
    if (pos.x >= 15 && pos.x < 120) Kp -= 0.02f;
    else if (pos.x >= 220 && pos.x < 270) Kp += 0.02f;
    drawButton("Kp", 20, 70, Kp);
  }
  else if (pos.y >= 110 && pos.y < 140) {
    if (pos.x >= 15 && pos.x < 120) Ki -= 0.1f;
    else if (pos.x >= 220 && pos.x < 270) Ki += 0.1f;
    drawButton("Ki", 20, 110, Ki);
  }
  else if (pos.y >= 150 && pos.y < 180) {
    if (pos.x >= 15 && pos.x < 120) Kd -= 0.02f;
    else if (pos.x >= 220 && pos.x < 270) Kd += 0.02f;
    drawButton("Kd", 20, 150, Kd);
  }
}


void controlLoopTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(true) {
      calcPID();
      vTaskDelayUntil(&xLastWakeTime, xPeriodMs);
    }
}


bool enShowCtrlPanel = false;
void uiLoopTask(void *pvParameters){
  
  TickType_t xLastWakeTime = xTaskGetTickCount();  
  while(true){
    M5.update();

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

  DEBUG_SERIAL.println("DYNAMIXEL ping Waiting...");

  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  DEBUG_SERIAL.print("ping L: ");
  DEBUG_SERIAL.print(dxl.ping(DXL_ID_L));
  DEBUG_SERIAL.print(", ping R: ");
  DEBUG_SERIAL.println(dxl.ping(DXL_ID_R));

  if (!dxl.ping(DXL_ID_L) || !dxl.ping(DXL_ID_R)) {
    M5.Lcd.println("DYNAMIXEL ping failed!");
    while (true) delay(1000);
  }

  DEBUG_SERIAL.println("DYNAMIXEL ping OK");

  dxl.torqueOff(DXL_ID_L);
  dxl.torqueOff(DXL_ID_R);
  delay(20);

  dxl.setOperatingMode(DXL_ID_L, OP_CURRENT);
  dxl.setOperatingMode(DXL_ID_R, OP_CURRENT);
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
  sr_info.addr_length = LEN_PRESENT_VELOCITY;
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

  DEBUG_SERIAL.println("# Gains: Kp,Ki,Kd,Kp_vel,Ki_vel");
  DEBUG_SERIAL.printf("# %f,%f,%f,%f,%f\n", Kp, Ki, Kd, Kp_vel, Ki_vel);
  DEBUG_SERIAL.println("T,t_ms,dt_us,pitch,pitch_err,vel_offset,theta_dot,vel_L,vel_R,v_fwd,i_unsat,i_sat,z_theta,vel_int");
  
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
