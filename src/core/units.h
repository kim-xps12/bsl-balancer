// units.h — SI ⇔ DYNAMIXEL raw 変換 (XL330規範 §6.2)。Arduino 非依存・純関数。
// 負値の 2 の補数は int16_t/int32_t への値変換で扱う (le16/le32 参照)。
#pragma once

#include <cmath>
#include <cstdint>

namespace units {

constexpr float kPi = 3.14159265358979f;

// Goal/Present Current(102/126): 1 raw = 1 mA (公式確認)
inline int16_t currentAToRaw(float a) {
  return static_cast<int16_t>(std::lroundf(a * 1000.0f));
}
inline float rawToCurrentA(int16_t raw) { return static_cast<float>(raw) * 0.001f; }

// Present Velocity(128): 0.229 rpm / raw
constexpr float kVelRadSPerRaw = 0.229f * 2.0f * kPi / 60.0f;
inline float rawToVelRadS(int32_t raw) { return static_cast<float>(raw) * kVelRadSPerRaw; }

// Present Position(132): 4096 pulse / rev
constexpr float kPosRadPerRaw = 2.0f * kPi / 4096.0f;
inline float rawToPosRad(int32_t raw) { return static_cast<float>(raw) * kPosRadPerRaw; }

// リトルエンディアン バイト列 → 符号付き整数 (2 の補数)
inline int16_t le16(const uint8_t* b) {
  return static_cast<int16_t>(static_cast<uint16_t>(b[0]) |
                              (static_cast<uint16_t>(b[1]) << 8));
}
inline int32_t le32(const uint8_t* b) {
  return static_cast<int32_t>(static_cast<uint32_t>(b[0]) |
                              (static_cast<uint32_t>(b[1]) << 8) |
                              (static_cast<uint32_t>(b[2]) << 16) |
                              (static_cast<uint32_t>(b[3]) << 24));
}

inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace units
