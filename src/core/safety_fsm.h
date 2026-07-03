// safety_fsm.h — 安全状態機械 (設計書 §6)。Arduino 非依存・時刻/入力は注入。
// ハードウェア作用 (enter_balancing, safe_stop 等) は Action として要求を返し、
// 実行と結果報告 (notify*) は ControlTask の責務。
#pragma once

#include <cstdint>

namespace core {

enum class FsmState : uint8_t {
  Initializing = 0,
  Idle,        // Torque OFF・零電流心拍。起立検出で BALANCING 要求
  Balancing,   // Torque ON はこの状態のみ
  Fallen,      // Torque OFF ラッチ。静置検出で Idle 復帰
  Disarmed,    // ユーザ STOP ラッチ。BtnC 長押しでのみ Idle へ
  Fault,       // ラッチ。復帰はリセットのみ
};

enum class FsmAction : uint8_t {
  None = 0,
  EnterBalancing,  // §4.1 enter_balancing() の実行要求
  SafeStop,        // 零電流(検証付き)→Torque OFF の実行要求 (Fallen/Disarmed/Fault 突入)
};

enum class FaultReason : uint8_t {
  None = 0,
  InitFailed,
  ImuStale,
  DxlReadStale,
  DxlWriteUnverified,
  WatchdogTripTorqueOn,
  WatchdogRecoverRepeated,
  HardOverspeed,
  HwErrorStatus,
  OverTemp,
  UnderVoltage,
  LoopOverrun,
  CurrentPlausibility,
  FallEscalation,
  EntryVerifyFailed,
};

class SafetyFsm {
 public:
  struct Params {
    float start_window_rad = 0.0873f;
    float start_rate_max = 0.35f;
    float start_wheel_max = 1.0f;
    float upright_hold_s = 1.0f;
    float fall_threshold_rad = 0.611f;
    int fall_escalation_count = 3;
    float fall_escalation_window_s = 30.0f;
    bool auto_arm = true;      // コミッショニング済みの場合のみ有効
    bool commissioned = false; // 未コミッショニングなら明示アーム必須
  };

  struct Input {
    float theta = 0.0f;        // 0 中心 [rad]
    float theta_rate = 0.0f;   // [rad/s]
    float wheel_speed_max = 0.0f;  // max(|ωL|,|ωR|) [rad/s]
    bool wheel_valid = false;
    bool stop_toggle = false;  // BtnC 長押しのエッジ (UI から)
    bool save_in_progress = false;  // §9.1: 全アーム経路をブロック
    FaultReason fault = FaultReason::None;  // 上位が検出したフォールト
    float now_s = 0.0f;        // 単調時刻 [s]
    float dt = 0.005f;
  };

  struct Result {
    FsmState state;
    FsmAction action;
  };

  void setParams(const Params& p) { p_ = p; }
  const Params& params() const { return p_; }

  FsmState state() const { return state_; }
  FaultReason faultReason() const { return fault_reason_; }
  int fallCount() const { return fall_count_; }

  // INITIALIZING 完了 (自己検査含む) の報告。auto-arm ゲート (§6) を適用。
  void notifyInitDone() {
    if (state_ != FsmState::Initializing) return;
    state_ = (p_.commissioned && p_.auto_arm) ? FsmState::Idle : FsmState::Disarmed;
  }

  // enter_balancing() の実行結果報告
  void notifyBalancingEntered() {
    if (state_ == FsmState::Idle && pending_ == FsmAction::EnterBalancing) {
      state_ = FsmState::Balancing;
      upright_since_valid_ = false;  // 保持タイマを持ち越さない (転倒後の即時再アーム防止)
    }
    pending_ = FsmAction::None;
  }
  void notifyEntryFailed() {
    latchFault(FaultReason::EntryVerifyFailed);
    pending_ = FsmAction::None;
  }
  // SafeStop 実行完了の報告 (Fallen/Disarmed 突入完了)
  void notifySafeStopDone() { pending_ = FsmAction::None; }

  Result update(const Input& in) {
    Result r{state_, FsmAction::None};

    // FAULT は最優先・全状態から (ラッチ)
    if (in.fault != FaultReason::None && state_ != FsmState::Fault) {
      latchFault(in.fault);
      r.state = state_;
      r.action = FsmAction::SafeStop;
      return r;
    }

    switch (state_) {
      case FsmState::Initializing:
        break;  // notifyInitDone() 待ち

      case FsmState::Idle: {
        if (in.stop_toggle) {
          state_ = FsmState::Disarmed;
          // 既に Torque OFF。保存ゲート中はバス送信が拒否され偽 FAULT になる
          // ため SafeStop を発行しない (§9.1)
          r.action = in.save_in_progress ? FsmAction::None : FsmAction::SafeStop;
          break;
        }
        if (in.save_in_progress) {
          upright_since_valid_ = false;  // 保存中はアーム経路全ブロック (§9.1)
          break;
        }
        if (uprightHold(in)) {
          if (pending_ == FsmAction::None) {
            pending_ = FsmAction::EnterBalancing;
            r.action = FsmAction::EnterBalancing;
          }
        }
        break;
      }

      case FsmState::Balancing: {
        if (in.stop_toggle) {
          state_ = FsmState::Disarmed;
          r.action = FsmAction::SafeStop;
          break;
        }
        if (fabsf_(in.theta) > p_.fall_threshold_rad) {
          registerFall(in.now_s);
          upright_since_valid_ = false;  // Fallen の静置検出は新規に 1.0s を要求
          if (fallEscalated(in.now_s)) {
            latchFault(FaultReason::FallEscalation);
          } else {
            state_ = FsmState::Fallen;
          }
          r.action = FsmAction::SafeStop;
        }
        break;
      }

      case FsmState::Fallen: {
        if (in.stop_toggle) {
          state_ = FsmState::Disarmed;
          break;
        }
        if (in.save_in_progress) {
          upright_since_valid_ = false;
          break;
        }
        // 静置検出 (起立姿勢 + 静止 + 車輪停止) → Idle (Torque OFF のまま)
        if (uprightHold(in)) {
          state_ = FsmState::Idle;
          upright_since_valid_ = false;  // Idle 側で改めて 1.0s 要求
        }
        break;
      }

      case FsmState::Disarmed: {
        if (in.stop_toggle) {
          // 明示アーム (BtnC): コミッショニング前でも許可 (ブリングアップ経路)
          if (!in.save_in_progress) {
            state_ = FsmState::Idle;
            upright_since_valid_ = false;
          }
        }
        break;
      }

      case FsmState::Fault:
        break;  // ラッチ (リセットのみ)
    }

    r.state = state_;
    if (r.action == FsmAction::None && pending_ == FsmAction::EnterBalancing &&
        state_ == FsmState::Idle) {
      // 要求発行済み・実行待ち中は再発行しない
    }
    return r;
  }

 private:
  static float fabsf_(float v) { return v < 0.0f ? -v : v; }

  bool uprightHold(const Input& in) {
    const bool ok = fabsf_(in.theta) < p_.start_window_rad &&
                    fabsf_(in.theta_rate) < p_.start_rate_max &&
                    in.wheel_valid && in.wheel_speed_max < p_.start_wheel_max;
    if (!ok) {
      upright_since_valid_ = false;
      return false;
    }
    if (!upright_since_valid_) {
      upright_since_ = in.now_s;
      upright_since_valid_ = true;
      return false;
    }
    return (in.now_s - upright_since_) >= p_.upright_hold_s;
  }

  // 転倒履歴はスライディング窓で数える (先頭基準のリセットだと窓を跨いだ
  // 中間の転倒が脱落し、実際には N 回/窓 でもラッチを逃す)
  void registerFall(float now_s) {
    if (fall_count_ < kMaxFallHistory) {
      fall_ts_[fall_count_++] = now_s;
    } else {
      for (int i = 1; i < kMaxFallHistory; ++i) fall_ts_[i - 1] = fall_ts_[i];
      fall_ts_[kMaxFallHistory - 1] = now_s;
    }
  }

  bool fallEscalated(float now_s) const {
    if (p_.fall_escalation_count <= 0) return false;
    int recent = 0;
    for (int i = 0; i < fall_count_; ++i) {
      if ((now_s - fall_ts_[i]) <= p_.fall_escalation_window_s) ++recent;
    }
    return recent >= p_.fall_escalation_count;
  }

  void latchFault(FaultReason r) {
    state_ = FsmState::Fault;
    if (fault_reason_ == FaultReason::None) fault_reason_ = r;
  }

  static const int kMaxFallHistory = 8;

  Params p_;
  FsmState state_ = FsmState::Initializing;
  FsmAction pending_ = FsmAction::None;
  FaultReason fault_reason_ = FaultReason::None;
  bool upright_since_valid_ = false;
  float upright_since_ = 0.0f;
  int fall_count_ = 0;  // 履歴保持数 (kMaxFallHistory で飽和)
  float fall_ts_[kMaxFallHistory] = {};
};

}  // namespace core
