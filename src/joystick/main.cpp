#include "M5Unified.h"
#include "M5HatMiniJoyC.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "EspNowProtocol.h"

// Based on the official M5Stack Hat Mini JoyC Arduino tutorial:
// https://docs.m5stack.com/ja/arduino/projects/hat/hat_mini_joyc

// Mini JoyC I2C pins for M5StickC Plus 1.1.
#define MiniJoyC_SDA 0
#define MiniJoyC_SCL 26

constexpr size_t CENTER_CAL_SAMPLES = 100;
constexpr uint16_t MIN_CAL_HALF_RANGE = 500;
constexpr uint32_t POSITION_SAMPLE_INTERVAL_MS = 20;
constexpr uint32_t DISPLAY_UPDATE_INTERVAL_MS = 60;
constexpr uint8_t MAX_CONSECUTIVE_SEND_FAILURES = 10;

M5HatMiniJoyC joyc;

namespace {

portMUX_TYPE espnow_link_mux = portMUX_INITIALIZER_UNLOCKED;
uint8_t espnow_receiver_mac[ESP_NOW_ETH_ALEN]{};
uint8_t espnow_pending_receiver_mac[ESP_NOW_ETH_ALEN]{};
bool espnow_receiver_ready = false;
bool espnow_receiver_pending = false;
bool espnow_peer_reset_pending = false;
bool espnow_send_in_flight = false;
bool espnow_sender_initialized = false;
uint8_t espnow_consecutive_send_failures = 0;
uint16_t espnow_position_sequence = 0;

void recordEspNowSendFailureLocked() {
  if (espnow_consecutive_send_failures < UINT8_MAX) {
    ++espnow_consecutive_send_failures;
  }
  if (espnow_consecutive_send_failures >= MAX_CONSECUTIVE_SEND_FAILURES) {
    espnow_receiver_ready = false;
    espnow_peer_reset_pending = true;
  }
}

void onEspNowDiscoveryReceived(const uint8_t* sender_mac, const uint8_t* data,
                               int data_length) {
  if (sender_mac == nullptr || data == nullptr ||
      data_length != sizeof(espnow_protocol::Packet)) {
    return;
  }

  espnow_protocol::Packet packet{};
  memcpy(&packet, data, sizeof(packet));
  if (!espnow_protocol::isValidPacket(
          packet, espnow_protocol::PacketType::Discovery)) {
    return;
  }

  portENTER_CRITICAL(&espnow_link_mux);
  if (!espnow_receiver_ready && !espnow_receiver_pending) {
    memcpy(espnow_pending_receiver_mac, sender_mac,
           sizeof(espnow_pending_receiver_mac));
    espnow_receiver_pending = true;
  }
  portEXIT_CRITICAL(&espnow_link_mux);
}

void onEspNowPositionSent(const uint8_t* receiver_mac,
                          esp_now_send_status_t status) {
  (void)receiver_mac;
  portENTER_CRITICAL(&espnow_link_mux);
  espnow_send_in_flight = false;
  if (status == ESP_NOW_SEND_SUCCESS) {
    espnow_consecutive_send_failures = 0;
  } else {
    recordEspNowSendFailureLocked();
  }
  portEXIT_CRITICAL(&espnow_link_mux);
}

bool initializeEspNowSender() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, false);
  delay(20);

  esp_err_t result = esp_wifi_set_ps(WIFI_PS_NONE);
  if (result != ESP_OK) {
    Serial.printf("ESP-NOW: disabling Wi-Fi sleep failed: %d\n", result);
    return false;
  }

  result = esp_wifi_set_channel(espnow_protocol::WIFI_CHANNEL,
                                WIFI_SECOND_CHAN_NONE);
  if (result != ESP_OK) {
    Serial.printf("ESP-NOW: setting channel failed: %d\n", result);
    return false;
  }

  result = esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_1M_L);
  if (result != ESP_OK) {
    Serial.printf("ESP-NOW: setting PHY rate failed: %d\n", result);
    return false;
  }

  result = esp_now_init();
  if (result != ESP_OK) {
    Serial.printf("ESP-NOW: initialization failed: %d\n", result);
    return false;
  }

  result = esp_now_register_recv_cb(onEspNowDiscoveryReceived);
  if (result != ESP_OK) {
    Serial.printf("ESP-NOW: receive callback failed: %d\n", result);
    return false;
  }

  result = esp_now_register_send_cb(onEspNowPositionSent);
  if (result != ESP_OK) {
    Serial.printf("ESP-NOW: send callback failed: %d\n", result);
    return false;
  }

  uint8_t local_mac[ESP_NOW_ETH_ALEN];
  esp_wifi_get_mac(WIFI_IF_STA, local_mac);
  Serial.printf(
      "ESP-NOW sender ready: %02X:%02X:%02X:%02X:%02X:%02X, channel %u\n",
      local_mac[0], local_mac[1], local_mac[2], local_mac[3], local_mac[4],
      local_mac[5], espnow_protocol::WIFI_CHANNEL);
  return true;
}

void serviceEspNowPeer() {
  bool reset_peer = false;
  bool add_peer = false;
  uint8_t old_mac[ESP_NOW_ETH_ALEN]{};
  uint8_t new_mac[ESP_NOW_ETH_ALEN]{};

  portENTER_CRITICAL(&espnow_link_mux);
  if (espnow_peer_reset_pending) {
    memcpy(old_mac, espnow_receiver_mac, sizeof(old_mac));
    espnow_peer_reset_pending = false;
    reset_peer = true;
  }
  if (espnow_receiver_pending) {
    memcpy(new_mac, espnow_pending_receiver_mac, sizeof(new_mac));
    espnow_receiver_pending = false;
    add_peer = true;
  }
  portEXIT_CRITICAL(&espnow_link_mux);

  if (reset_peer && esp_now_is_peer_exist(old_mac)) {
    esp_now_del_peer(old_mac);
    Serial.println("ESP-NOW: receiver link lost; waiting for discovery");
  }

  if (!add_peer) {
    return;
  }

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, new_mac, sizeof(peer.peer_addr));
  peer.channel = espnow_protocol::WIFI_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;

  const esp_err_t result = esp_now_add_peer(&peer);
  if (result != ESP_OK && result != ESP_ERR_ESPNOW_EXIST) {
    Serial.printf("ESP-NOW: adding receiver failed: %d\n", result);
    return;
  }

  portENTER_CRITICAL(&espnow_link_mux);
  memcpy(espnow_receiver_mac, new_mac, sizeof(espnow_receiver_mac));
  espnow_receiver_ready = true;
  espnow_consecutive_send_failures = 0;
  portEXIT_CRITICAL(&espnow_link_mux);

  Serial.printf("ESP-NOW receiver found: %02X:%02X:%02X:%02X:%02X:%02X\n",
                new_mac[0], new_mac[1], new_mac[2], new_mac[3], new_mac[4],
                new_mac[5]);
}

void sendJoystickPosition(int8_t pos_x, int8_t pos_y) {
  uint8_t receiver_mac[ESP_NOW_ETH_ALEN];

  portENTER_CRITICAL(&espnow_link_mux);
  if (!espnow_receiver_ready || espnow_send_in_flight) {
    portEXIT_CRITICAL(&espnow_link_mux);
    return;
  }
  memcpy(receiver_mac, espnow_receiver_mac, sizeof(receiver_mac));
  espnow_send_in_flight = true;
  portEXIT_CRITICAL(&espnow_link_mux);

  const auto packet = espnow_protocol::makePacket(
      espnow_protocol::PacketType::JoystickPosition,
      espnow_position_sequence++, millis(), pos_x, pos_y);
  const esp_err_t result =
      esp_now_send(receiver_mac, reinterpret_cast<const uint8_t*>(&packet),
                   sizeof(packet));

  if (result != ESP_OK) {
    portENTER_CRITICAL(&espnow_link_mux);
    espnow_send_in_flight = false;
    recordEspNowSendFailureLocked();
    portEXIT_CRITICAL(&espnow_link_mux);
  }
}

}  // namespace

enum class LinkDisplayStatus {
  Error,
  Waiting,
  Connected,
};

static LinkDisplayStatus getLinkDisplayStatus() {
  if (!espnow_sender_initialized) {
    return LinkDisplayStatus::Error;
  }

  portENTER_CRITICAL(&espnow_link_mux);
  const bool connected = espnow_receiver_ready;
  portEXIT_CRITICAL(&espnow_link_mux);
  return connected ? LinkDisplayStatus::Connected : LinkDisplayStatus::Waiting;
}

static void drawLinkStatus(bool force = false) {
  static LinkDisplayStatus previous_status = LinkDisplayStatus::Error;
  static bool has_previous_status = false;
  const LinkDisplayStatus status = getLinkDisplayStatus();
  if (!force && has_previous_status && status == previous_status) {
    return;
  }

  uint16_t status_color = RED;
  const char* status_text = "ERR";
  switch (status) {
    case LinkDisplayStatus::Connected:
      status_color = GREEN;
      status_text = "OK";
      break;
    case LinkDisplayStatus::Waiting:
      status_color = YELLOW;
      status_text = "WAIT";
      break;
    case LinkDisplayStatus::Error:
      break;
  }

  // Clear the full status field so no pixels from a longer previous state
  // (for example WAIT -> OK) remain visible.
  M5.Display.fillRect(55, 145, 80, 28, BLACK);
  M5.Display.drawLine(0, 135, M5.Display.width() - 1, 135, status_color);
  M5.Display.setTextColor(status_color, BLACK);
  M5.Display.setCursor(55, 165);
  M5.Display.printf("%-4s", status_text);

  previous_status = status;
  has_previous_status = true;
}

static void waitMiniJoyCReady() {
  while (!joyc.begin(&Wire, MiniJoyC_ADDR, MiniJoyC_SDA, MiniJoyC_SCL,
                     100000UL)) {
    delay(100);
  }
}

static void drawStaticUi() {
  M5.Display.fillScreen(BLACK);
  M5.Display.setTextColor(WHITE, BLACK);

  M5.Display.setCursor(0, 20);
  M5.Display.print("POS X:");
  M5.Display.setCursor(0, 50);
  M5.Display.print("POS Y:");

  M5.Display.drawLine(0, 80, M5.Display.width() - 1, 80, DARKGREY);

  M5.Display.setCursor(0, 110);
  M5.Display.print("BtnVal:");

  M5.Display.setCursor(0, 165);
  M5.Display.print("LINK:");
  drawLinkStatus(true);

  M5.Display.setTextColor(YELLOW, BLACK);
  M5.Display.setFont(&fonts::Font2);
  M5.Display.setTextDatum(textdatum_t::middle_center);
  M5.Display.drawString("Button R x2: Cal", M5.Display.width() / 2, 220);
  M5.Display.setTextDatum(textdatum_t::top_left);
  M5.Display.setFont(&fonts::FreeMonoBold9pt7b);
}

static void drawRangeCalibrationUi() {
  M5.Display.fillScreen(BLACK);
  M5.Display.setTextColor(YELLOW, BLACK);
  M5.Display.setCursor(0, 20);
  M5.Display.print("Range Cal");
  M5.Display.setTextColor(WHITE, BLACK);
  M5.Display.setCursor(0, 50);
  M5.Display.print("Rotate stick");
  M5.Display.setCursor(0, 80);
  M5.Display.print("X min:");
  M5.Display.setCursor(0, 105);
  M5.Display.print("X max:");
  M5.Display.setCursor(0, 130);
  M5.Display.print("Y min:");
  M5.Display.setCursor(0, 155);
  M5.Display.print("Y max:");
  M5.Display.drawLine(0, 175, 135, 175, ORANGE);
  M5.Display.setCursor(0, 195);
  M5.Display.print("Press stick");
  M5.Display.setCursor(0, 220);
  M5.Display.print("to save");
}

static void calibrateJoystick() {
  M5.Display.fillScreen(BLACK);
  M5.Display.setTextColor(YELLOW, BLACK);
  M5.Display.setCursor(0, 30);
  M5.Display.print("Center Cal");
  M5.Display.setTextColor(WHITE, BLACK);
  M5.Display.setCursor(0, 70);
  M5.Display.print("Release stick");
  M5.Display.setCursor(0, 100);
  M5.Display.print("Sampling...");

  // Give the user time to release the controls after the double-click.
  delay(500);

  uint32_t sum_x = 0;
  uint32_t sum_y = 0;
  uint16_t min_x = 4095;
  uint16_t min_y = 4095;
  uint16_t max_x = 0;
  uint16_t max_y = 0;

  for (size_t i = 0; i < CENTER_CAL_SAMPLES; ++i) {
    const uint16_t adc_x = joyc.getADCValue(ADC_X);
    const uint16_t adc_y = joyc.getADCValue(ADC_Y);
    sum_x += adc_x;
    sum_y += adc_y;
    min_x = min(min_x, adc_x);
    min_y = min(min_y, adc_y);
    max_x = max(max_x, adc_x);
    max_y = max(max_y, adc_y);
    delay(5);
  }

  // A trimmed mean prevents one noisy sample from shifting the center.
  const uint16_t center_x =
      (sum_x - min_x - max_x) / (CENTER_CAL_SAMPLES - 2);
  const uint16_t center_y =
      (sum_y - min_y - max_y) / (CENTER_CAL_SAMPLES - 2);

  drawRangeCalibrationUi();
  min_x = min_y = 4095;
  max_x = max_y = 0;
  uint32_t last_display_update = 0;

  while (true) {
    M5.update();

    const uint16_t adc_x = joyc.getADCValue(ADC_X);
    const uint16_t adc_y = joyc.getADCValue(ADC_Y);
    min_x = min(min_x, adc_x);
    min_y = min(min_y, adc_y);
    max_x = max(max_x, adc_x);
    max_y = max(max_y, adc_y);

    if (millis() - last_display_update >= 100) {
      M5.Display.setCursor(77, 80);
      M5.Display.printf("%4u", min_x);
      M5.Display.setCursor(77, 105);
      M5.Display.printf("%4u", max_x);
      M5.Display.setCursor(77, 130);
      M5.Display.printf("%4u", min_y);
      M5.Display.setCursor(77, 155);
      M5.Display.printf("%4u", max_y);
      last_display_update = millis();
    }

    if (!joyc.getButtonStatus()) {
      const bool range_is_valid =
          center_x >= min_x + MIN_CAL_HALF_RANGE &&
          max_x >= center_x + MIN_CAL_HALF_RANGE &&
          center_y >= min_y + MIN_CAL_HALF_RANGE &&
          max_y >= center_y + MIN_CAL_HALF_RANGE;

      if (!range_is_valid) {
        M5.Display.fillRect(0, 180, 135, 60, BLACK);
        M5.Display.setTextColor(RED, BLACK);
        M5.Display.setCursor(0, 200);
        M5.Display.print("Range small");
        M5.Display.setCursor(0, 225);
        M5.Display.print("Keep rotating");
        delay(1200);
        M5.Display.setTextColor(WHITE, BLACK);
        M5.Display.fillRect(0, 180, 135, 60, BLACK);
        M5.Display.setCursor(0, 195);
        M5.Display.print("Press stick");
        M5.Display.setCursor(0, 220);
        M5.Display.print("to save");
        while (!joyc.getButtonStatus()) {
          delay(10);
        }
        continue;
      }
      break;
    }

    delay(10);
  }

  uint16_t calibration[6] = {min_x, max_x, min_y,
                             max_y, center_x, center_y};

  M5.Display.fillScreen(BLACK);
  M5.Display.setTextColor(YELLOW, BLACK);
  M5.Display.setCursor(0, 30);
  M5.Display.print("Saving...");
  joyc.setAllCalValue(calibration);

  M5.Display.setTextColor(GREEN, BLACK);
  M5.Display.setCursor(0, 70);
  M5.Display.print("Saved");
  M5.Display.setTextColor(WHITE, BLACK);
  M5.Display.setCursor(0, 110);
  M5.Display.printf("X:%u-%u", min_x, max_x);
  M5.Display.setCursor(0, 140);
  M5.Display.printf("Y:%u-%u", min_y, max_y);
  M5.Display.setCursor(0, 180);
  M5.Display.printf("C:%u,%u", center_x, center_y);
  delay(1500);

  drawStaticUi();
}

void setup() {
  Serial.begin(115200);
  M5.begin();
  waitMiniJoyCReady();
  joyc.setLEDColor(0x000000);
  espnow_sender_initialized = initializeEspNowSender();

  M5.Display.setRotation(0);
  M5.Display.setFont(&fonts::FreeMonoBold9pt7b);
  drawStaticUi();
}

void loop() {
  M5.update();
  serviceEspNowPeer();

  if (M5.BtnB.wasDoubleClicked()) {
    calibrateJoystick();
  }

  static uint32_t last_sample_ms = 0;
  static uint32_t last_display_ms = 0;
  const uint32_t now = millis();
  if (now - last_sample_ms < POSITION_SAMPLE_INTERVAL_MS) {
    delay(1);
    return;
  }
  last_sample_ms = now;

  // Read normalized position (-128~127).
  int8_t pos_x = joyc.getPOSValue(POS_X, _8bit);
  int8_t pos_y = joyc.getPOSValue(POS_Y, _8bit);

  sendJoystickPosition(pos_x, pos_y);

  if (now - last_display_ms >= DISPLAY_UPDATE_INTERVAL_MS) {
    // Redraw only fixed-width value fields. The opaque text background erases
    // the previous value without flashing the whole display.
    M5.Display.setCursor(66, 20);
    M5.Display.printf("%4d", pos_x);
    M5.Display.setCursor(66, 50);
    M5.Display.printf("%4d", pos_y);
    M5.Display.setCursor(77, 110);
    M5.Display.printf("%d", joyc.getButtonStatus());
    drawLinkStatus();
    last_display_ms = now;
  }
}
