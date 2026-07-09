// attitude_estimator.h — 1D 相補推定器 + バイアス推定 + accel ゲート (設計書 §5.2)。
// Arduino 非依存。出力は絶対傾斜 pitch_abs [rad]（0 中心化は ControlTask の単一変換点で行う）。
// Mahony 1D 形式: pitch' = (gyro - bias) + kp_f*err, bias' = -ki_f*err (臨界減衰 ki_f = kp_f²/4)
#pragma once

#include <cmath>

namespace core {

struct ImuSample {
  float gyro_rate = 0.0f;    // 傾斜軸まわり角速度 [rad/s] (バイアス未補正)
  bool gyro_fresh = false;
  float acc_tilt = 0.0f;     // 傾斜面内の重力成分 (atan2 の第1引数側) [m/s^2]
  float acc_vert = 0.0f;     // 直立時に重力が載る成分 (atan2 の第2引数側) [m/s^2]
  float acc_norm = 0.0f;     // |a| [m/s^2]
  bool accel_fresh = false;
};

class AttitudeEstimator {
 public:
  struct Params {
    float tau_s = 1.0f;        // 相補融合時定数
    float accel_gate_g = 0.3f; // ||a|-1g| > gate で補正停止
    float gravity = 9.80665f;
  };

  void setParams(const Params& p) {
    p_ = p;
    kp_f_ = (p.tau_s > 0.0f) ? (1.0f / p.tau_s) : 0.0f;
    ki_f_ = kp_f_ * kp_f_ * 0.25f;
  }

  // 静止校正の結果で初期化 (pitch0: accel 傾斜平均, bias0: gyro 平均)
  void reset(float pitch0, float bias0) {
    pitch_ = pitch0;
    bias_ = bias0;
  }

  // dt: 実測 [s]。gyro stale 時は前回レートで予測ホールド (設計書 §5.2)
  void update(const ImuSample& s, float dt) {
    if (dt <= 0.0f) return;
    if (s.gyro_fresh) last_gyro_ = s.gyro_rate;

    float err = 0.0f;
    bool correct = false;
    if (s.accel_fresh) {
      const float norm_dev = std::fabs(s.acc_norm - p_.gravity);
      if (norm_dev <= p_.accel_gate_g * p_.gravity) {
        const float tilt_acc = std::atan2(s.acc_tilt, s.acc_vert);
        err = tilt_acc - pitch_;
        correct = true;
      }
    }

    const float rate = (last_gyro_ - bias_) + (correct ? kp_f_ * err : 0.0f);
    pitch_ += rate * dt;
    if (correct) bias_ -= ki_f_ * err * dt;
  }

  float pitchAbs() const { return pitch_; }
  float rate() const { return last_gyro_ - bias_; }  // バイアス補正済みレート
  float bias() const { return bias_; }

 private:
  Params p_;
  float kp_f_ = 1.0f;
  float ki_f_ = 0.25f;
  float pitch_ = 0.0f;
  float bias_ = 0.0f;
  float last_gyro_ = 0.0f;
};

}  // namespace core
