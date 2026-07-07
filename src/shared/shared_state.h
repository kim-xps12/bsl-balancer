// shared_state.h — タスク間データ受け渡し (設計書 §3.2)
// 制御→UI/telemetry: critical section 保護スナップショット
// UI→制御: パラメータ変更キュー + atomic フラグ (STOP / 保存ゲート)
//
// publish()/read() の相互排他方式 (UDP telemetry Phase1 計画書 §3.1 前提修正):
// 旧実装は「奇数 seq を release store → plain memcpy → 偶数 seq を release store」の
// seqlock だったが、release store は後続のデータ書込みが奇数 store より前に移動する
// ことを禁止しない。弱メモリ順序のデュアルコア環境 (ESP32) では読み手が torn な
// snapshot を受理し得り、plain memcpy の並行アクセスは C++ 規格上データレース (UB) の
// ままである。telemetry がこの snapshot を Wi-Fi 安全ゲート (WIFI_QUIET 判定) に使う
// ため、fence-only seqlock は安全ゲート基盤として不採用とし、publish()/read() の両方を
// portMUX_TYPE spinlock (taskENTER_CRITICAL/taskEXIT_CRITICAL) で置き換える。
// クロスコア相互排他が形式的に保証され、コピーは ~100B で数µs、writer
// (ControlTask) の最悪待ちは読者側 critical section 長 (数µs) に有界となる。
// read() は常に成功する設計になるが、bool 戻り値 API と telemetry 側の
// read-fail/diagnostic セマンティクスは防御的に維持する (期待値ゼロのカウンタとして
// 観測を残す)。native (host test) ビルドでは portMUX/FreeRTOS が存在しないため
// std::mutex で同義の相互排他に差し替える (ARDUINO マクロで分岐)。
#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>

#if defined(ARDUINO)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#else
#include <mutex>
#endif

#include "../core/param_validation.h"
#include "../core/safety_fsm.h"

namespace shared {

namespace detail {

#if defined(ARDUINO)
// ESP32 実機: portMUX スピンロック (クロスコア相互排他)
class CriticalSection {
 public:
  void lock() { taskENTER_CRITICAL(&mux_); }
  void unlock() { taskEXIT_CRITICAL(&mux_); }

 private:
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};
#else
// native (host test): std::mutex で同義の相互排他を提供 (マルチスレッド stress
// テストで実際に競合排他が効くことを検証するため no-op にはしない)
class CriticalSection {
 public:
  void lock() { m_.lock(); }
  void unlock() { m_.unlock(); }

 private:
  std::mutex m_;
};
#endif

class LockGuard {
 public:
  explicit LockGuard(CriticalSection& cs) : cs_(cs) { cs_.lock(); }
  ~LockGuard() { cs_.unlock(); }
  LockGuard(const LockGuard&) = delete;
  LockGuard& operator=(const LockGuard&) = delete;

 private:
  CriticalSection& cs_;
};

}  // namespace detail

struct Snapshot {
  // 状態
  uint8_t fsm_state = 0;        // core::FsmState
  uint8_t fault_reason = 0;     // core::FaultReason
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
  bool wheel_valid = false;     // DXL 帰還の有効性 (起立検出の必要条件)
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

  // ---- UDP telemetry Phase1 追加 (計画書 §3.1): 新設の monotonic 専用カウンタ ----
  // 既存の overrun_count/read_stale_count (LoopState, 連続回数・リセットあり) とは
  // 別物。リセット経路を持たない累積総回数であり、telemetry の 20Hz サンプリングでも
  // 隣接 packet 間の差分を取れば全 200Hz サイクルの dt 分布を漏れなく被覆できる。
  uint32_t dt_hist_total[8] = {};  // 周期比 <1.02x,<1.05x,<1.1x,<1.2x,<1.3x,<1.5x,<2.0x,>=2.0x
  uint32_t overrun_total = 0;      // dt > 1.5x 周期の累積総回数
  uint32_t imu_stale_total = 0;    // IMU (gyro) stale 読みの累積総回数
  // Idle→Balancing 遷移保留 (直立ホールド進行中) 全般。自動アーム・
  // BtnC 手動アーム後の Idle の両方で同一機構 (SafetyFsm::armPending() 参照)。
  bool arm_pending = false;
};

// パラメータ変更コマンド (UI → 制御)
enum class ParamField : uint8_t {
  Kp = 0, Ki, Kd, PitchEq, VelLoopEnabled,
};
struct ParamCommand {
  ParamField field;
  float value;
};

enum class SaveKind : uint8_t { None = 0, Params };

class SharedState {
 public:
  // ---- critical section 保護スナップショット (旧 seqlock からの置換。上記コメント参照) ----
  void publish(const Snapshot& s) {
    detail::LockGuard g(mux_);
    std::memcpy(&buf_, &s, sizeof(Snapshot));
  }
  // 常に true を返す設計 (critical section により torn read が形式的に排除される)。
  // bool 戻り値は API 安定性と telemetry 側の防御的 read-fail セマンティクスのために
  // 維持する (期待値ゼロのカウンタとして観測を残す)。
  bool read(Snapshot* out) const {
    detail::LockGuard g(mux_);
    std::memcpy(out, &buf_, sizeof(Snapshot));
    return true;
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

  mutable detail::CriticalSection mux_;
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
