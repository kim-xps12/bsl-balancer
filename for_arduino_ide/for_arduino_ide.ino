#include <M5Unified.h>
#include <Kalman.h>
#include <Preferences.h>
#include <Dynamixel2Arduino.h>

HardwareSerial& DXL_SERIAL = Serial1;
#define DEBUG_SERIAL Serial
#define ENABLE_DEBUG_PRINT

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
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;


// Contoller Values
unsigned long previousTimestamp = 0;
unsigned long currentTimestamp = 0;
unsigned long elapsedTime = 0;


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


void controlLoopTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(true) {
      calcPID();
      vTaskDelayUntil(&xLastWakeTime, xPeriodMs);
    }
}


void uiLoopTask(void *pvParameters){
  
  TickType_t xLastWakeTime = xTaskGetTickCount();  
  while(true){
    M5.update();

    bool isReleased = true;
    if (M5.Touch.getCount() > 0 ) {
      isReleased = false;
      auto pos = M5.Touch.getDetail();

      if(pos.wasPressed()){
        // 各ボタンのタッチ確認
        if (pos.y >= 30 && pos.y < 60) { // Target
          if (pos.x >= 15 && pos.x < 120) pitch_target-=0.2;
          else if (pos.x >= 220 && pos.x < 270) pitch_target+=0.2;
          drawButton("Ref", 20, 30, pitch_target);
        }
        else if (pos.y >= 70 && pos.y < 100) {
          if (pos.x >= 15 && pos.x < 120) Kp-=0.2;
          else if (pos.x >= 220 && pos.x < 270) Kp+=0.2;
          drawButton("Kp", 20, 70, Kp);
        }
        else if (pos.y >= 110 && pos.y < 140) { // Ki
          if (pos.x >= 15 && pos.x < 120) Ki-=0.2;
          else if (pos.x >= 220 && pos.x < 270) Ki+=0.2;
          drawButton("Ki", 20, 110, Ki);
        }
        else if (pos.y >= 150 && pos.y < 180) { // Kd
          if (pos.x >= 15 && pos.x < 120) Kd-=0.2;
          else if (pos.x >= 220 && pos.x < 270) Kd+=0.2;
          drawButton("Kd", 20, 150, Kd);
        }
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriodMs*5);
  }
}

void calcPID(){

  currentTimestamp = micros();
  elapsedTime = currentTimestamp - previousTimestamp;
  previousTimestamp = currentTimestamp;
  
  float dt = (float)xPeriodMs /1000; // [sec]
  float pitch_kalman = kalman.getAngle(getPitch(), getPitchDot(), dt);
  float pitch_dot_kalman = kalman.getRate();

  float pitch_error = pitch_target - pitch_kalman;

  // M5.Lcd.clear();
  // M5.Lcd.setCursor(0, 0);
  // M5.Lcd.println(pitch_kalman);
  // M5.Lcd.println(elapsedTime);

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
  //DEBUG_SERIAL.println(pitch_kalman);

}


void drawButton(const char* label, int x, int y, float value) {
  // ボタンの描画
  M5.Display.drawRect(x, y, 50, 30, WHITE);
  M5.Display.drawString("-", x+20, y, 2);

  M5.Display.drawRect(x+200, y, 50, 30, WHITE);
  M5.Display.drawString("+", x+200+20, y, 2);
  
  // ラベルと値の描画
  M5.Display.setCursor(x+70, y+5);
  M5.Display.print(label);
  M5.Display.print(": ");
  M5.Display.println(value, 1);
}

void setup(){

  DXL_SERIAL.begin(2000000, SERIAL_8N1, 32, 33);
  dxl = Dynamixel2Arduino(DXL_SERIAL);

  // M5 Settings
  M5.begin();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);
  
  drawButton("Ref", 20, 30, pitch_target);
  drawButton("Kp", 20, 70, Kp);
  drawButton("Ki", 20, 110, Ki);
  drawButton("Kd", 20, 150, Kd);
  
  M5.Imu.init();
  // M5.Imu.setAccelFsr(M5.Imu.AFS_2G);
  // M5.Imu.setGyroFsr(M5.Imu.GFS_250DPS);

  DEBUG_SERIAL.begin(115200);

  // DYNAMIXEL Settings
  dxl.begin(2000000);  // DYNAMIXEL baudrate.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(DXL_ID_L);
  dxl.ping(DXL_ID_R);
  dxl.torqueOff(DXL_ID_L);  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID_R); 
  dxl.setOperatingMode(DXL_ID_L, OP_VELOCITY);
  dxl.setOperatingMode(DXL_ID_R, OP_VELOCITY);
  dxl.torqueOn(DXL_ID_L);
  dxl.torqueOn(DXL_ID_R);

  kalman.setAngle(getPitch());
  
  const uint32_t MEMORY_STACK = 8192;
  const UBaseType_t PRIORIRY_SPIN_MAIN = 5;
  //const BaseType_t ID_CORE_CTRL_MAIN = 0;
  xTaskCreatePinnedToCore(controlLoopTask, "Control Loop Task", MEMORY_STACK, NULL, PRIORIRY_SPIN_MAIN, NULL, 0);
  const UBaseType_t PRIORIRY_SPIN_SUB = 2;
  //const BaseType_t ID_CORE_CTRL_SUB = 0;
  xTaskCreatePinnedToCore(uiLoopTask,      "UI Loop Task",      MEMORY_STACK, NULL, PRIORIRY_SPIN_SUB,  NULL, 1);
}


void loop(){
  
}
