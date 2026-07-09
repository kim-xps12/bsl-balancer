// balance_core.h — カスケード制御パイプライン (設計書 §2.1/§2.3)。Arduino 非依存。
// 入力 θ は 0 中心 (単一変換点は呼び出し側 ControlTask)。出力は符号正規化前の
// I_common/I_yaw ベース電流 [A]。左右符号 (s_L/s_R) の適用は dxl_backend の責務。
#pragma once

#include <cmath>

#include "pid.h"

namespace core {

// I²t 型ピーク電流制限器 (設計書 §2.3。|I| で対称)
class I2tLimiter {
 public:
  struct Params {
    float i_peak = 0.45f;
    float i_cont = 0.30f;
    float peak_duration_s = 0.5f;
  };
  void setParams(const Params& p) {
    p_ = p;
    e_max_ = (p.i_peak * p.i_peak - p.i_cont * p.i_cont) * p.peak_duration_s;
    if (e_max_ < 0.0f) e_max_ = 0.0f;
  }
  void reset() { e_ = 0.0f; }

  // 指令を実効上限でクランプして返し、エネルギー状態を更新する
  float apply(float i_cmd, float dt) {
    const float i_eff = (e_ >= e_max_) ? p_.i_cont : p_.i_peak;
    float out = i_cmd;
    if (out > i_eff) out = i_eff;
    if (out < -i_eff) out = -i_eff;
    // E += (|I|² − I_cont²)·dt (正で蓄積・負で回復)、[0, E_max] にクランプ
    e_ += (out * out - p_.i_cont * p_.i_cont) * dt;
    if (e_ < 0.0f) e_ = 0.0f;
    if (e_ > e_max_) e_ = e_max_;
    return out;
  }
  bool limited() const { return e_ >= e_max_; }
  float energy() const { return e_; }

 private:
  Params p_;
  float e_ = 0.0f;
  float e_max_ = 0.0f;
};

// 速度ガード (XL330規範 §11): 加速方向のみソフト上限から線形縮小。ハード超過は
// フォールト信号を返す (停止処理は上位の FSM)。
inline float applySpeedGuard(float i_cmd, float omega, float omega_soft,
                             float omega_hard, bool* hard_overspeed) {
  const float abs_w = std::fabs(omega);
  if (abs_w >= omega_hard) {
    if (hard_overspeed) *hard_overspeed = true;
    return 0.0f;
  }
  const bool accelerating = (i_cmd * omega) > 0.0f;
  if (!accelerating || abs_w <= omega_soft) return i_cmd;
  float scale = (omega_hard - abs_w) / (omega_hard - omega_soft);
  if (scale < 0.0f) scale = 0.0f;
  if (scale > 1.0f) scale = 1.0f;
  return i_cmd * scale;
}

class BalanceCore {
 public:
  struct Params {
    Pid::Params pitch;            // 内側ピッチ PID
    float kv = 0.10f;             // 外側速度 P [rad/(m/s)]
    float kvi = 0.05f;            // 外側速度 I [rad/m]
    float theta_ref_limit = 0.0524f;  // ±3° [rad]
    bool vel_loop_enabled = true;
    float wheel_speed_soft = 25.0f;   // [rad/s]
    float wheel_speed_hard = 35.0f;   // [rad/s]
    float slew_a_per_s = 5.0f;        // 電流スルーレート [A/s]
    I2tLimiter::Params i2t;
  };

  struct Input {
    float theta = 0.0f;        // 0 中心ピッチ [rad]
    float theta_rate = 0.0f;   // バイアス補正済み [rad/s]
    float v = 0.0f;            // 車体速度 (車輪平均, 前進正) [m/s]
    float omega_left = 0.0f;   // 正規化済み車輪角速度 (前進正) [rad/s]
    float omega_right = 0.0f;
    bool wheel_valid = false;  // false = 当該周期の帰還 stale (§4.2: 外側/ガード凍結)
    float i_yaw = 0.0f;        // v1 では 0 (構造のみ)
    float dt = 0.005f;         // 実測 [s]
  };

  struct Output {
    float i_left = 0.0f;   // 符号正規化前 (前進正) [A]
    float i_right = 0.0f;
    float theta_ref = 0.0f;
    bool hard_overspeed = false;  // → FAULT
    bool saturated = false;
    bool i2t_limited = false;
  };

  void setParams(const Params& p) {
    p_ = p;
    pitch_pid_.setParams(p.pitch);
    i2t_left_.setParams(p.i2t);
    i2t_right_.setParams(p.i2t);
  }
  const Params& params() const { return p_; }

  void reset() {
    pitch_pid_.reset();
    vel_i_ = 0.0f;
    theta_ref_ = 0.0f;
    i2t_left_.reset();
    i2t_right_.reset();
    prev_left_ = 0.0f;
    prev_right_ = 0.0f;
  }

  Output update(const Input& in) {
    Output out;

    // 外側速度 PI → θ_ref (帰還 stale の周期は凍結し前回値を保持)
    if (p_.vel_loop_enabled && in.wheel_valid) {
      const float ev = 0.0f - in.v;  // v_ref = 0 (station keeping)
      vel_i_ += p_.kvi * ev * in.dt;
      const float lim = p_.theta_ref_limit;
      if (vel_i_ > lim) vel_i_ = lim;
      if (vel_i_ < -lim) vel_i_ = -lim;
      float tr = p_.kv * ev + vel_i_;
      if (tr > lim) tr = lim;
      if (tr < -lim) tr = -lim;
      theta_ref_ = tr;
    } else if (!p_.vel_loop_enabled) {
      theta_ref_ = 0.0f;
    }
    out.theta_ref = theta_ref_;

    // 内側ピッチ PID → I_common。倒立の復元則は「倒れる方向へ駆動」
    // τ = Kp·(θ−θ_ref) + Kd·θ̇ (前傾 θ>0 で前進電流が正) なので、
    // 誤差定義 e=ref−meas の PID 出力を負号反転して用いる (I/D も整合)。
    const float i_common_raw =
        -pitch_pid_.update(theta_ref_, in.theta, in.theta_rate, in.dt);
    float i_common = i_common_raw;

    // ミキサ + 倒立優先飽和 (XL330規範 §10.1)。
    // saturated は「共通モードが上限に張り付いている」事実で判定
    // (ピッチ PID 側の out_limit が同値で先にクランプしても検出できるように)
    const float i_max = p_.i2t.i_peak;
    if (i_common > i_max) i_common = i_max;
    if (i_common < -i_max) i_common = -i_max;
    out.saturated = std::fabs(i_common) >= i_max - 1e-6f;
    float headroom = i_max - std::fabs(i_common);
    if (headroom < 0.0f) headroom = 0.0f;
    float i_yaw = in.i_yaw;
    if (i_yaw > headroom) i_yaw = headroom;
    if (i_yaw < -headroom) i_yaw = -headroom;
    float i_l = i_common - i_yaw;
    float i_r = i_common + i_yaw;

    // 速度ガード (per wheel。帰還 stale の周期は更新しない=素通し禁止のため 0 電流は上位判断)
    if (in.wheel_valid) {
      i_l = applySpeedGuard(i_l, in.omega_left, p_.wheel_speed_soft,
                            p_.wheel_speed_hard, &out.hard_overspeed);
      i_r = applySpeedGuard(i_r, in.omega_right, p_.wheel_speed_soft,
                            p_.wheel_speed_hard, &out.hard_overspeed);
    }

    // スルーレート制限 (per wheel)
    const float max_step = p_.slew_a_per_s * in.dt;
    i_l = slew(prev_left_, i_l, max_step);
    i_r = slew(prev_right_, i_r, max_step);

    // I²t (per wheel) + 最終対称クランプ
    i_l = i2t_left_.apply(i_l, in.dt);
    i_r = i2t_right_.apply(i_r, in.dt);
    out.i2t_limited = i2t_left_.limited() || i2t_right_.limited();

    prev_left_ = i_l;
    prev_right_ = i_r;
    out.i_left = i_l;
    out.i_right = i_r;
    return out;
  }

  float velIntegrator() const { return vel_i_; }
  const Pid& pitchPid() const { return pitch_pid_; }

  // 呼び出し側が計算結果と異なる電流を実際に送った場合 (stale コースト等) に
  // スルーレート状態を送信実績へ整合させる (§4.2。I²t は保守側なので保持)
  void overrideOutput(float i_left, float i_right) {
    prev_left_ = i_left;
    prev_right_ = i_right;
  }

 private:
  static float slew(float prev, float target, float max_step) {
    const float d = target - prev;
    if (d > max_step) return prev + max_step;
    if (d < -max_step) return prev - max_step;
    return target;
  }

  Params p_;
  Pid pitch_pid_;
  float vel_i_ = 0.0f;
  float theta_ref_ = 0.0f;
  I2tLimiter i2t_left_;
  I2tLimiter i2t_right_;
  float prev_left_ = 0.0f;
  float prev_right_ = 0.0f;
};

}  // namespace core
