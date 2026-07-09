// pid.h — ピッチ PID (設計書 §2.1)。Arduino 非依存。
// D 項は外部供給レート (derivative on measurement) + PT1 LPF。
// アンチワインドアップ: 積分クランプ + 出力飽和時の条件付き積分。
#pragma once

#include <cmath>

namespace core {

class Pid {
 public:
  struct Params {
    float kp = 0.0f;          // [out/err]
    float ki = 0.0f;          // [out/(err*s)]
    float kd = 0.0f;          // [out/(err/s)]
    float d_lpf_hz = 25.0f;   // D 項 PT1 カットオフ [Hz]。<=0 でフィルタ無効
    float i_limit = 0.0f;     // 積分項の絶対クランプ [out]
    float out_limit = 0.0f;   // 出力の絶対クランプ [out]。<=0 で無制限
  };

  void setParams(const Params& p) { p_ = p; }
  const Params& params() const { return p_; }

  void reset() {
    i_term_ = 0.0f;
    rate_filt_ = 0.0f;
    rate_filt_init_ = false;
  }

  // ref/meas: 同一単位。meas_rate: d(meas)/dt (ジャイロ直読み)。dt: 実測 [s]
  float update(float ref, float meas, float meas_rate, float dt) {
    const float e = ref - meas;

    // D 項 PT1 (meas_rate をフィルタ)
    if (p_.d_lpf_hz > 0.0f && dt > 0.0f) {
      const float rc = 1.0f / (2.0f * 3.14159265f * p_.d_lpf_hz);
      const float alpha = dt / (dt + rc);
      if (!rate_filt_init_) {
        rate_filt_ = meas_rate;
        rate_filt_init_ = true;
      } else {
        rate_filt_ += alpha * (meas_rate - rate_filt_);
      }
    } else {
      rate_filt_ = meas_rate;
      rate_filt_init_ = true;
    }

    const float p_term = p_.kp * e;
    const float d_term = -p_.kd * rate_filt_;

    // 条件付き積分: 出力が飽和していて誤差が飽和を深める向きなら積分停止。
    // ただし |i_term| を減らす向き (巻き戻し) は常に許可 — これが AW の本来目的。
    const float u_pre = p_term + i_term_ + d_term;
    bool deepen = false;
    if (p_.out_limit > 0.0f) {
      if (u_pre > p_.out_limit && e > 0.0f) deepen = true;
      if (u_pre < -p_.out_limit && e < 0.0f) deepen = true;
    }
    const bool unwinds = (i_term_ > 0.0f && e < 0.0f) || (i_term_ < 0.0f && e > 0.0f);
    if (!deepen || unwinds) {
      i_term_ += p_.ki * e * dt;
      if (i_term_ > p_.i_limit) i_term_ = p_.i_limit;
      if (i_term_ < -p_.i_limit) i_term_ = -p_.i_limit;
    }

    float u = p_term + i_term_ + d_term;
    if (p_.out_limit > 0.0f) {
      if (u > p_.out_limit) u = p_.out_limit;
      if (u < -p_.out_limit) u = -p_.out_limit;
    }
    return u;
  }

  float iTerm() const { return i_term_; }
  float rateFilt() const { return rate_filt_; }

 private:
  Params p_;
  float i_term_ = 0.0f;
  float rate_filt_ = 0.0f;
  bool rate_filt_init_ = false;
};

}  // namespace core
