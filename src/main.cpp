#include <Arduino.h>

#include <M5Unified.h>

#include <Kalman.h>
#include <Preferences.h>
#include <Dynamixel2Arduino.h>
#include <esp_task_wdt.h>

HardwareSerial& DXL_SERIAL = Serial1;
#define DEBUG_SERIAL Serial
//#define ENABLE_DEBUG_PRINT

// M5Stack Core2 PORT A pin configuration
const uint8_t PIN_RX_SERVO = 33;
const uint8_t PIN_TX_SERVO = 32;

// Contoller Params.
const TickType_t xPeriodMs = 10;  // [milli sec]
float Kp = 50.0;
float Ki = 1.0;
float Kd = 1.0;

float pitch_target = 86.0; //[deg]

float P = 0.0;
float I = 0.0;
float D = 0.0;
float preP = 0.0;

// Kalman filter Params.
Kalman kalman;

// Contoller Values
unsigned long previousTimestamp = 0;
unsigned long currentTimestamp = 0;
unsigned long elapsedTime = 0;

// DYNAMIXEL Params.
const uint8_t DXL_ID_L = 0;
const uint8_t DXL_ID_R = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl;//(DXL_SERIAL);

// I2C bus mutex (IMU on control task vs AXP192/touch on UI task)
SemaphoreHandle_t g_i2c_mutex = nullptr;

struct ImuData { float pitch_deg; float pitch_rate_dps; bool valid; };

ImuData readImuBurst() {
    ImuData r = {0, 0, false};
    if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(2)) != pdTRUE) return r;
    auto mask = M5.Imu.update();
    xSemaphoreGive(g_i2c_mutex);
    if (!mask) return r;
    auto d = M5.Imu.getImuData();
    r.pitch_deg = atan2f(d.accel.y, d.accel.z) * RAD_TO_DEG;
    r.pitch_rate_dps = d.gyro.x;
    r.valid = true;
    return r;
}


void driveTire(int rpm) {

  // Set Goal Velocity using RPM
  dxl.setGoalVelocity(DXL_ID_L, -1*rpm, UNIT_RPM);
  dxl.setGoalVelocity(DXL_ID_R, rpm, UNIT_RPM);

  #ifdef ENABLE_DEBUG_PRINT
  DEBUG_SERIAL.print("Present Velocity(rpm)--L : ");
  DEBUG_SERIAL.print(dxl.getPresentVelocity(DXL_ID_L, UNIT_RPM));
  DEBUG_SERIAL.print(", R : ");
  DEBUG_SERIAL.println(dxl.getPresentVelocity(DXL_ID_R, UNIT_RPM));
  #endif
}


void calcPID(){

  currentTimestamp = micros();
  elapsedTime = currentTimestamp - previousTimestamp;
  previousTimestamp = currentTimestamp;

  float dt = (float)xPeriodMs /1000; // [sec]

  ImuData imu = readImuBurst();
  if (!imu.valid) return;

  float pitch_kalman = kalman.getAngle(imu.pitch_deg, imu.pitch_rate_dps, dt);
  float pitch_dot_kalman = kalman.getRate();

  float pitch_error = pitch_target - pitch_kalman;

  if(pitch_error < -40 || 40 < pitch_error) {
    driveTire(0);
    P = 0;
    I = 0;
    D = 0;
    return;
  }

  P = pitch_target - pitch_kalman;
  I += P * dt;
  D = (P - preP) / dt;
  preP = P;

  // anti windup
  if (200 < abs(I * Ki)) {
    I = 0;
  }

  // Calclate Motor Output
  int rpm_motor = Kp * P + Ki * I + Kd * D;
  rpm_motor = constrain(rpm_motor, -300, 300);
  driveTire(rpm_motor);
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
  M5.Display.println(value, 1);
}


void drawCtrlPanel(){
  drawButton("Ref", 20, 30, pitch_target);
  drawButton("Kp", 20, 70, Kp);
  drawButton("Ki", 20, 110, Ki);
  drawButton("Kd", 20, 150, Kd);

  if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    int batteryPercentage = M5.Power.getBatteryLevel();
    xSemaphoreGive(g_i2c_mutex);
    M5.Display.setCursor(20, 190);
    M5.Display.printf("M5Core2 Battery: %d%%", batteryPercentage);
  }
}


void handleCtrlPanelTouched(){

  auto pos = M5.Touch.getDetail();

  if (pos.y >= 30 && pos.y < 60) { // Target
    if (pos.x >= 15 && pos.x < 120) pitch_target -= 0.2;
      else if (pos.x >= 220 && pos.x < 270) pitch_target += 0.2;
      drawButton("Ref", 20, 30, pitch_target);
  }
  else if (pos.y >= 70 && pos.y < 100) {
    if (pos.x >= 15 && pos.x < 120) Kp -= 0.2;
    else if (pos.x >= 220 && pos.x < 270) Kp += 0.2;
    drawButton("Kp", 20, 70, Kp);
  }
  else if (pos.y >= 110 && pos.y < 140) { // Ki
    if (pos.x >= 15 && pos.x < 120) Ki -= 0.2;
    else if (pos.x >= 220 && pos.x < 270) Ki += 0.2;
    drawButton("Ki", 20, 110, Ki);
  }
  else if (pos.y >= 150 && pos.y < 180) { // Kd
    if (pos.x >= 15 && pos.x < 120) Kd -= 0.2;
    else if (pos.x >= 220 && pos.x < 270) Kd += 0.2;
    drawButton("Kd", 20, 150, Kd);
  }
}


void controlLoopTask(void *pvParameters) {
    esp_task_wdt_add(NULL);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(true) {
      esp_task_wdt_reset();
      calcPID();
      vTaskDelayUntil(&xLastWakeTime, xPeriodMs);
    }
}


bool enShowCtrlPanel = false;
void uiLoopTask(void *pvParameters){

  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(true){
    if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      M5.update();
      xSemaphoreGive(g_i2c_mutex);
    }

    // show tuning panel
    if (M5.BtnB.wasPressed()) {
      if (enShowCtrlPanel) {
        enShowCtrlPanel = false;
        M5.Lcd.clear();
      } else {
        enShowCtrlPanel = true;
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

  // M5 Settings (selective init: disable unused peripherals)
  auto cfg = M5.config();
  cfg.internal_rtc = false;
  cfg.internal_mic = false;
  cfg.internal_spk = false;
  cfg.internal_imu = true;
  M5.begin(cfg);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);

  // Avatar disabled during architecture improvement (Phase 3-8)
  // Will be re-enabled in Phase 9 after all improvements are stable.

  // I2C mutex
  g_i2c_mutex = xSemaphoreCreateMutex();
  configASSERT(g_i2c_mutex != nullptr);

  // DYNAMIXEL Settings
  DXL_SERIAL.begin(1000000, SERIAL_8N1, PIN_RX_SERVO, PIN_TX_SERVO);
  dxl = Dynamixel2Arduino(DXL_SERIAL);
  dxl.begin(1000000);  // DYNAMIXEL baudrate.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(DXL_ID_L);
  dxl.ping(DXL_ID_R);
  dxl.torqueOff(DXL_ID_L);  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID_R);
  dxl.setOperatingMode(DXL_ID_L, OP_VELOCITY);
  dxl.setOperatingMode(DXL_ID_R, OP_VELOCITY);
  dxl.torqueOn(DXL_ID_L);
  dxl.torqueOn(DXL_ID_R);

  // Kalman filter Setting (initial pitch via burst read)
  ImuData imu_init = readImuBurst();
  if (imu_init.valid) {
    kalman.setAngle(imu_init.pitch_deg);
  } else {
    kalman.setAngle(86.0);
  }

  // WDT
  esp_task_wdt_init(1, true);

  // RTOS Task Settings
  const uint32_t MEMORY_STACK = 8192;
  const UBaseType_t PRIORITY_CTRL = 20;
  const BaseType_t CORE_CTRL = 1;
  xTaskCreatePinnedToCore(controlLoopTask, "Control Loop Task", MEMORY_STACK, NULL, PRIORITY_CTRL, NULL, CORE_CTRL);

  const UBaseType_t PRIORITY_UI = 3;
  const BaseType_t CORE_UI = 0;
  xTaskCreatePinnedToCore(uiLoopTask,      "UI Loop Task",      MEMORY_STACK, NULL, PRIORITY_UI,   NULL, CORE_UI);
}


void loop(){

}
