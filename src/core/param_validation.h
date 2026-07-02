// param_validation.h — NVS レコード検証 (設計書 §9.1) + コミッショニング認可 (§6)。
// 純関数・fail-closed: 版不一致/範囲外が 1 つでもあれば全体破棄して既定値へ。
#pragma once

#include <cstdint>

#include "../app_config.h"

namespace core {

struct TuningParams {
  uint16_t schema_version = cfg::kParamSchemaVersion;
  float kp = cfg::kPitchKp;
  float ki = cfg::kPitchKi;
  float kd = cfg::kPitchKd;
  float kv = cfg::kVelKv;
  float kvi = cfg::kVelKvi;
  float pitch_eq = cfg::kPitchEqDefaultRad;
  float theta_ref_limit = cfg::kThetaRefLimitRad;
  bool vel_loop_enabled = cfg::kVelLoopEnabledDefault;
};

inline bool inRange(float v, const cfg::ParamRange& r) {
  // NaN は全比較で false → 自動的に破棄側へ落ちる
  return v >= r.min && v <= r.max;
}

// 戻り値: true = record 採用可 / false = 破棄 (呼び出し側は既定値を使う)
inline bool validateParams(const TuningParams& rec) {
  if (rec.schema_version != cfg::kParamSchemaVersion) return false;
  if (!inRange(rec.kp, cfg::kRangeKp)) return false;
  if (!inRange(rec.ki, cfg::kRangeKi)) return false;
  if (!inRange(rec.kd, cfg::kRangeKd)) return false;
  if (!inRange(rec.kv, cfg::kRangeKv)) return false;
  if (!inRange(rec.kvi, cfg::kRangeKvi)) return false;
  if (!inRange(rec.pitch_eq, cfg::kRangePitchEq)) return false;
  if (!inRange(rec.theta_ref_limit, cfg::kRangeThetaRefLimit)) return false;
  return true;
}

// コミッショニング認可レコード (§6): fail-closed。
// sign_axis_checksum はファーム側の符号/軸定数から毎回計算して照合する。
struct CommissioningRecord {
  uint16_t schema_version = 0;
  uint8_t profile = 0;              // cfg::Profile
  uint32_t sign_axis_checksum = 0;  // 符号/軸 config のチェックサム
  uint16_t calib_version = 0;       // pitch_eq 校正レコードの版
  bool user_confirmed = false;
};

// 現在のファーム構成から符号/軸チェックサムを計算 (FNV-1a 32bit)。
// コミッショニング (符号試験合格) が保証するのは車輪符号と IMU 軸/符号の両方
// なので、どちらが変わっても記録を無効化する (§6 fail-closed)。
inline uint32_t signAxisChecksum(float sign_left, float sign_right,
                                 float imu_tilt_sign, float imu_vert_sign,
                                 float imu_gyro_sign) {
  uint32_t h = 2166136261u;
  auto mix = [&h](uint32_t v) {
    for (int i = 0; i < 4; ++i) {
      h ^= (v >> (i * 8)) & 0xFFu;
      h *= 16777619u;
    }
  };
  // 符号は -1/+1 のみ想定 → 整数化して混ぜる (浮動小数のビット差を避ける)
  mix(sign_left < 0.0f ? 0xFFFFFFFFu : 1u);
  mix(sign_right < 0.0f ? 0xFFFFFFFFu : 1u);
  mix(imu_tilt_sign < 0.0f ? 0xFFFFFFFFu : 1u);
  mix(imu_vert_sign < 0.0f ? 0xFFFFFFFFu : 1u);
  mix(imu_gyro_sign < 0.0f ? 0xFFFFFFFFu : 1u);
  return h;
}

// 現行 config 一式からのチェックサム (呼び出し箇所の引数ミスマッチを防ぐ)
inline uint32_t currentSignAxisChecksum() {
  return signAxisChecksum(cfg::kSignLeft, cfg::kSignRight, cfg::kImuAccTiltSign,
                          cfg::kImuAccVertSign, cfg::kImuGyroSign);
}

// true = コミッショニング済みとして auto-arm を許可してよい
inline bool validateCommissioning(const CommissioningRecord& rec,
                                  uint32_t current_sign_axis_checksum,
                                  uint16_t current_calib_version) {
  if (rec.schema_version != cfg::kCommissionSchemaVersion) return false;
  if (rec.profile != static_cast<uint8_t>(cfg::Profile::Normal)) return false;
  if (rec.sign_axis_checksum != current_sign_axis_checksum) return false;
  if (rec.calib_version != current_calib_version) return false;
  if (!rec.user_confirmed) return false;
  return true;
}

}  // namespace core
