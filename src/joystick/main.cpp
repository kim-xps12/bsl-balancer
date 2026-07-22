#include "M5Unified.h"
#include "M5HatMiniJoyC.h"

// Based on the official M5Stack Hat Mini JoyC Arduino tutorial:
// https://docs.m5stack.com/ja/arduino/projects/hat/hat_mini_joyc

// Mini JoyC I2C pins for M5StickC Plus 1.1.
#define MiniJoyC_SDA 0
#define MiniJoyC_SCL 26

constexpr size_t CENTER_CAL_SAMPLES = 100;
constexpr uint16_t MIN_CAL_HALF_RANGE = 500;

M5HatMiniJoyC joyc;

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
  M5.Display.print("ADC X:");
  M5.Display.setCursor(0, 50);
  M5.Display.print("ADC Y:");

  M5.Display.drawLine(0, 80, 135, 80, ORANGE);

  M5.Display.setCursor(0, 100);
  M5.Display.print("POS X:");
  M5.Display.setCursor(0, 130);
  M5.Display.print("POS Y:");

  M5.Display.drawLine(0, 160, 135, 160, ORANGE);
  M5.Display.setCursor(0, 180);
  M5.Display.print("BtnVal:");

  M5.Display.setTextColor(YELLOW, BLACK);
  M5.Display.setCursor(0, 220);
  M5.Display.print("BtnB x2:Cal");
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
  M5.begin();
  waitMiniJoyCReady();
  joyc.setLEDColor(0x000000);

  M5.Display.setRotation(0);
  M5.Display.setFont(&fonts::FreeMonoBold9pt7b);
  drawStaticUi();
}

void loop() {
  M5.update();

  if (M5.BtnB.wasDoubleClicked()) {
    calibrateJoystick();
  }

  // Read raw ADC values (0~4095).
  int16_t adc_x = joyc.getADCValue(ADC_X);
  int16_t adc_y = joyc.getADCValue(ADC_Y);

  // Read normalized position (-128~127).
  int8_t pos_x = joyc.getPOSValue(POS_X, _8bit);
  int8_t pos_y = joyc.getPOSValue(POS_Y, _8bit);

  // Redraw only the fixed-width value fields. The opaque text background erases
  // the previous value without flashing the whole display.
  M5.Display.setCursor(66, 20);
  M5.Display.printf("%4d", adc_x);
  M5.Display.setCursor(66, 50);
  M5.Display.printf("%4d", adc_y);
  M5.Display.setCursor(66, 100);
  M5.Display.printf("%4d", pos_x);
  M5.Display.setCursor(66, 130);
  M5.Display.printf("%4d", pos_y);
  M5.Display.setCursor(77, 180);
  M5.Display.printf("%d", joyc.getButtonStatus());

  delay(30);
}
