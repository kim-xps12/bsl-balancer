// param_validation.h — NVS レコード検証 (設計書 §9.1)。
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

}  // namespace core
