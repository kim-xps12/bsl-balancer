#include <Arduino.h>

#include <M5Unified.h>
#include <Avatar.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "EspNowProtocol.h"
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


namespace {

constexpr uint8_t ESPNOW_BROADCAST_ADDRESS[ESP_NOW_ETH_ALEN] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
constexpr uint32_t ESPNOW_DISCOVERY_INTERVAL_MS = 1000;

struct ReceivedJoystickPosition {
  espnow_protocol::Packet packet;
  uint8_t sender_mac[ESP_NOW_ETH_ALEN];
};

StaticQueue_t espnow_receive_queue_control;
uint8_t espnow_receive_queue_storage[sizeof(ReceivedJoystickPosition)];
QueueHandle_t espnow_receive_queue = nullptr;
bool espnow_receiver_ready = false;
uint16_t espnow_discovery_sequence = 0;
uint32_t espnow_last_discovery_ms = 0;

void onEspNowDataReceived(const uint8_t* sender_mac, const uint8_t* data,
                          int data_length) {
  if (sender_mac == nullptr || data == nullptr ||
      data_length != sizeof(espnow_protocol::Packet) ||
      espnow_receive_queue == nullptr) {
    return;
  }

  ReceivedJoystickPosition received{};
  memcpy(&received.packet, data, sizeof(received.packet));
  if (!espnow_protocol::isValidPacket(
          received.packet, espnow_protocol::PacketType::JoystickPosition)) {
    return;
  }

  memcpy(received.sender_mac, sender_mac, sizeof(received.sender_mac));
  // The callback runs in the high-priority Wi-Fi task. Keep only the newest
  // sample and do all formatting/printing later in Arduino's loop task.
  xQueueOverwrite(espnow_receive_queue, &received);
}

bool initializeEspNowReceiver() {
  espnow_receive_queue = xQueueCreateStatic(
      1, sizeof(ReceivedJoystickPosition), espnow_receive_queue_storage,
      &espnow_receive_queue_control);
  if (espnow_receive_queue == nullptr) {
    DEBUG_SERIAL.println("ESP-NOW: failed to create receive queue");
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, false);
  delay(20);

  esp_err_t result = esp_wifi_set_ps(WIFI_PS_NONE);
  if (result != ESP_OK) {
    DEBUG_SERIAL.printf("ESP-NOW: disabling Wi-Fi sleep failed: %d\n", result);
    return false;
  }

  result = esp_wifi_set_channel(espnow_protocol::WIFI_CHANNEL,
                                WIFI_SECOND_CHAN_NONE);
  if (result != ESP_OK) {
    DEBUG_SERIAL.printf("ESP-NOW: setting channel failed: %d\n", result);
    return false;
  }

  result = esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_1M_L);
  if (result != ESP_OK) {
    DEBUG_SERIAL.printf("ESP-NOW: setting PHY rate failed: %d\n", result);
    return false;
  }

  result = esp_now_init();
  if (result != ESP_OK) {
    DEBUG_SERIAL.printf("ESP-NOW: initialization failed: %d\n", result);
    return false;
  }

  result = esp_now_register_recv_cb(onEspNowDataReceived);
  if (result != ESP_OK) {
    DEBUG_SERIAL.printf("ESP-NOW: receive callback failed: %d\n", result);
    return false;
  }

  esp_now_peer_info_t broadcast_peer{};
  memcpy(broadcast_peer.peer_addr, ESPNOW_BROADCAST_ADDRESS,
         sizeof(broadcast_peer.peer_addr));
  broadcast_peer.channel = espnow_protocol::WIFI_CHANNEL;
  broadcast_peer.ifidx = WIFI_IF_STA;
  broadcast_peer.encrypt = false;

  result = esp_now_add_peer(&broadcast_peer);
  if (result != ESP_OK && result != ESP_ERR_ESPNOW_EXIST) {
    DEBUG_SERIAL.printf("ESP-NOW: broadcast peer failed: %d\n", result);
    return false;
  }

  uint8_t local_mac[ESP_NOW_ETH_ALEN];
  esp_wifi_get_mac(WIFI_IF_STA, local_mac);
  DEBUG_SERIAL.printf(
      "ESP-NOW receiver ready: %02X:%02X:%02X:%02X:%02X:%02X, channel %u\n",
      local_mac[0], local_mac[1], local_mac[2], local_mac[3], local_mac[4],
      local_mac[5], espnow_protocol::WIFI_CHANNEL);
  return true;
}

void serviceEspNowReceiver() {
  if (!espnow_receiver_ready) {
    return;
  }

  const uint32_t now = millis();
  if (now - espnow_last_discovery_ms >= ESPNOW_DISCOVERY_INTERVAL_MS) {
    const auto packet = espnow_protocol::makePacket(
        espnow_protocol::PacketType::Discovery, espnow_discovery_sequence++,
        now);
    const esp_err_t result = esp_now_send(
        ESPNOW_BROADCAST_ADDRESS, reinterpret_cast<const uint8_t*>(&packet),
        sizeof(packet));
    if (result != ESP_OK) {
      DEBUG_SERIAL.printf("ESP-NOW: discovery send failed: %d\n", result);
    }
    espnow_last_discovery_ms = now;
  }

  ReceivedJoystickPosition received{};
  if (xQueueReceive(espnow_receive_queue, &received, 0) == pdTRUE) {
    DEBUG_SERIAL.printf("ESP-NOW RX PosX=%d, PosY=%d, seq=%u\n",
                        received.packet.pos_x, received.packet.pos_y,
                        received.packet.sequence);
  }
}

}  // namespace


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


void calcPID(){

  currentTimestamp = micros();
  elapsedTime = currentTimestamp - previousTimestamp;
  previousTimestamp = currentTimestamp;
  
  float dt = (float)xPeriodMs /1000; // [sec]
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

  M5.Display.setCursor(20, 190);
  int batteryPercentage = M5.Power.getBatteryLevel();
  M5.Display.printf("M5Core2 Battery: %d%%", batteryPercentage);
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
  espnow_receiver_ready = initializeEspNowReceiver();
  // M5.Imu.setAccelFsr(M5.Imu.AFS_2G);
  // M5.Imu.setGyroFsr(M5.Imu.GFS_250DPS);

  // M5 avatar Settings
  tairinFace = createTairinFace();
  avatar.setFace(tairinFace);
  avatar.init();

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

  // Kalman filter Setting
  kalman.setAngle(getPitch());
  
  // RTOS Task Settings
  const uint32_t MEMORY_STACK = 8192;
  const UBaseType_t PRIORIRY_SPIN_MAIN = 5;
  const BaseType_t ID_CORE_CTRL_MAIN = 0;
  xTaskCreatePinnedToCore(controlLoopTask, "Control Loop Task", MEMORY_STACK, NULL, PRIORIRY_SPIN_MAIN, NULL, ID_CORE_CTRL_MAIN);
  
  const UBaseType_t PRIORIRY_SPIN_SUB = 1;
  const BaseType_t ID_CORE_CTRL_SUB = 1;
  xTaskCreatePinnedToCore(uiLoopTask,      "UI Loop Task",      MEMORY_STACK, NULL, PRIORIRY_SPIN_SUB,  NULL, ID_CORE_CTRL_SUB);
}


void loop(){
  serviceEspNowReceiver();
  delay(1);
}
