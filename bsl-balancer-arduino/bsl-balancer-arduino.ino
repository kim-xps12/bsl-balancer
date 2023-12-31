#include <Dynamixel2Arduino.h>
#include <Kalman.h>
#include <M5Unified.h>

#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial
#define ENABLE_DEBUG_PRINT

// PID Params. 
float Kp = 100.0;
float Ki = 60.0;
float Kd = 5.0;

float pitch_target = 86.5; //[deg]

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
Dynamixel2Arduino dxl(DXL_SERIAL);
using namespace ControlTableItem;


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

void setup(){

  // M5 Settings
  M5.begin();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("I am BSL-Balancer!");
  
  M5.Imu.init();
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
}


void loop(){

  currentTimestamp = micros();
  elapsedTime = currentTimestamp - previousTimestamp;
  previousTimestamp = currentTimestamp;
  
  float dt = (float)elapsedTime / 1000000.0; // [sec]

  float pitch_kalman = kalman.getAngle(getPitch(), getPitchDot(), dt);
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
  DEBUG_SERIAL.println(pitch_kalman);
}
