// shared_state.h — タスク間データ受け渡し (設計書 §3.2)
// 制御→UI: seqlock スナップショット (書き手非ブロッキング)
// UI→制御: パラメータ変更キュー + atomic フラグ (STOP / 保存ゲート)
#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>

#include "../core/param_validation.h"
#include "../core/safety_fsm.h"

namespace shared {

struct Snapshot {
  // 状態
  uint8_t fsm_state = 0;        // core::FsmState
  uint8_t fault_reason = 0;     // core::FaultReason
  bool commissioned = false;
  uint8_t profile = 0;          // cfg::Profile
  // 制御量 (SI)
  float theta = 0.0f;           // 0 中心 [rad]
  float theta_rate = 0.0f;      // [rad/s]
  float theta_ref = 0.0f;
  float v = 0.0f;               // [m/s]
  float omega_left = 0.0f;      // 正規化済み [rad/s]
  float omega_right = 0.0f;
  float i_cmd_left = 0.0f;      // [A]
  float i_cmd_right = 0.0f;
  float i_present_left = 0.0f;
  float i_present_right = 0.0f;
  // タイミング
  float dt_last = 0.0f;         // [s]
  float dt_max = 0.0f;
  uint32_t loop_count = 0;
  // ヘルス
  float voltage = 0.0f;         // [V]
  float temperature = 0.0f;     // [degC]
  bool saturated = false;
  bool i2t_limited = false;
  // パネル表示用のパラメータエコー (実体は ControlTask 所有)
  core::TuningParams params;
};

// パラメータ変更コマンド (UI → 制御)
enum class ParamField : uint8_t {
  Kp = 0, Ki, Kd, PitchEq, VelLoopEnabled,
};
struct ParamCommand {
  ParamField field;
  float value;
};

enum class SaveKind : uint8_t { None = 0, Params, Commission };

class SharedState {
 public:
  // ---- seqlock スナップショット ----
  void publish(const Snapshot& s) {
    const uint32_t s0 = seq_.load(std::memory_order_relaxed);
    seq_.store(s0 + 1, std::memory_order_release);  // 奇数 = 書込中
    std::memcpy(&buf_, &s, sizeof(Snapshot));
    seq_.store(s0 + 2, std::memory_order_release);
  }
  bool read(Snapshot* out) const {
    for (int retry = 0; retry < 4; ++retry) {
      const uint32_t s0 = seq_.load(std::memory_order_acquire);
      if (s0 & 1u) continue;
      std::memcpy(out, &buf_, sizeof(Snapshot));
      std::atomic_thread_fence(std::memory_order_acquire);
      if (seq_.load(std::memory_order_relaxed) == s0) return true;
    }
    return false;
  }

  // ---- パラメータ変更 (単一生産者 UI / 単一消費者 Control の SPSC リング) ----
  bool pushParam(const ParamCommand& c) {
    const uint32_t h = param_head_.load(std::memory_order_relaxed);
    const uint32_t t = param_tail_.load(std::memory_order_acquire);
    if (h - t >= kParamQueueLen) return false;
    param_q_[h % kParamQueueLen] = c;
    param_head_.store(h + 1, std::memory_order_release);
    return true;
  }
  bool popParam(ParamCommand* out) {
    const uint32_t t = param_tail_.load(std::memory_order_relaxed);
    const uint32_t h = param_head_.load(std::memory_order_acquire);
    if (t == h) return false;
    *out = param_q_[t % kParamQueueLen];
    param_tail_.store(t + 1, std::memory_order_release);
    return true;
  }

  // ---- STOP/ARM トグル要求 (UI が set、Control が consume) ----
  void requestStopToggle() { stop_toggle_.store(true, std::memory_order_release); }
  bool consumeStopToggle() { return stop_toggle_.exchange(false, std::memory_order_acq_rel); }

  // ---- NVS 保存ゲート (§9.1: UI要求 → Control が safe-off 検証して許可 → UI が書く) ----
  void requestSave(SaveKind k) { save_request_.store(static_cast<uint8_t>(k), std::memory_order_release); }
  SaveKind saveRequest() const { return static_cast<SaveKind>(save_request_.load(std::memory_order_acquire)); }
  void clearSaveRequest() { save_request_.store(0, std::memory_order_release); }

  void setSaveInProgress(bool v) { save_in_progress_.store(v, std::memory_order_release); }
  bool saveInProgress() const { return save_in_progress_.load(std::memory_order_acquire); }

  void setSaveDone() { save_done_.store(true, std::memory_order_release); }
  bool consumeSaveDone() { return save_done_.exchange(false, std::memory_order_acq_rel); }

 private:
  static constexpr uint32_t kParamQueueLen = 8;

  mutable std::atomic<uint32_t> seq_{0};
  Snapshot buf_{};

  ParamCommand param_q_[kParamQueueLen] = {};
  std::atomic<uint32_t> param_head_{0};
  std::atomic<uint32_t> param_tail_{0};

  std::atomic<bool> stop_toggle_{false};
  std::atomic<uint8_t> save_request_{0};
  std::atomic<bool> save_in_progress_{false};
  std::atomic<bool> save_done_{false};
};

}  // namespace shared
