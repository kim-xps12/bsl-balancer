// watchdog_policy.h — Bus Watchdog 遷移表の判定 + 復旧頻度制限 (設計書 §4.3)。
// Arduino 非依存 (native テスト可能)。I/O (raw98/Torque Enable の読取・
// watchdogRecoverOne) は呼び出し側 (hw/dxl_backend.cpp) に残し、判断のみを
// 抽出する (段階1b。docs/plans/2026-07-06-safety-core-extraction.md §3.2)。
//
// API 形状は逐次リデューサ (ゲート1第1回指摘2対応 — 現行 checkWatchdog は
// 「rawL 読取失敗で即 return (rawR を読まない)」「teL 読取失敗で即 return
// (teR を読まない)」という早期 return を持ち、『両輪分を読んでから一括判定』
// の API では I/O 回数・順序の同一性が破れる。劣化経路でのバス往復・
// タイムアウト遅延の増加は安全上許容しない)。
#pragma once

#include <cstdint>

namespace core {

enum class WatchdogVerdict : uint8_t { Pending, Ok, Fault, ProceedRecover };

// 現行 checkWatchdog の判断列を I/O と同じ逐次順で消費するリデューサ。
// call-site 契約: 読取 I/O は現行と同一順 (raw[L]→raw[R]→(トリップ時のみ)
// te[L]→te[R]) で行い、各 feed が Pending 以外を返した時点で以降の I/O を
// 行わない (早期 return の I/O 回数同一性)。逐語対応は現行 :374-388:
//   feedRaw: 読取失敗 → torque_may_be_on ? Fault : Ok (即時) /
//            両輪 feed 完了かつ非トリップ (いずれも != tripped_value) → Ok /
//            トリップあり → Pending (TE 読取へ)
//   feedTe:  読取失敗 → Fault (即時) / 両輪 feed 完了かつ両輪 0 →
//            ProceedRecover / いずれか非零 → Fault (注: 現行 :388 は両輪
//            読取後に te[0]!=0||te[1]!=0 を判定するため、teL 非零でも teR の
//            読取は行われる。この I/O 回数も保存する — 非零検出は両輪 feed
//            完了時に行う)
//
// 現行挙動の潜在エッジ (ゲート1第2回指摘で発見・本 PR では挙動保存):
// 現行 :374-378 は「トリップ済み raw 値を観測した後に他輪の raw 読取が失敗」
// しても、torque_may_be_on==false なら Ok を返す (既知のトリップ証拠が破棄
// され、復旧も FAULT も発生しない)。本リデューサはこの挙動をそのまま保存
// する (feedRaw の読取失敗判定は先行 raw 値に依存しない)。これはロジック
// 変更ゼロ原則 (§2-1) による意図的判断であり、fail-closed 化 (トリップ証拠
// 観測後の読取失敗を Fault 化) は別課題としてユーザへエスカレーション済み
// (§8 残余リスク参照)。
class WatchdogDecision {
 public:
  explicit WatchdogDecision(bool torque_may_be_on, uint8_t tripped_value)
      : torque_may_be_on_(torque_may_be_on), tripped_value_(tripped_value) {}

  // L→R の順で最大 2 回呼ぶ。Pending 以外を返した時点で以降呼ばない。
  WatchdogVerdict feedRaw(bool read_ok, uint8_t value) {
    ++raw_feed_count_;
    if (!read_ok) {
      // dt 起因検査で raw98 が読めない場合: トルク有効中なら fail-closed
      return torque_may_be_on_ ? WatchdogVerdict::Fault : WatchdogVerdict::Ok;
    }
    if (value == tripped_value_) raw_tripped_ = true;
    if (raw_feed_count_ < 2) return WatchdogVerdict::Pending;  // R 読取待ち
    return raw_tripped_ ? WatchdogVerdict::Pending : WatchdogVerdict::Ok;
  }

  // L→R の順で最大 2 回呼ぶ (トリップ検出後のみ)。Pending 以外を返した時点で
  // 以降呼ばない。
  WatchdogVerdict feedTe(bool read_ok, uint8_t value) {
    ++te_feed_count_;
    if (!read_ok) return WatchdogVerdict::Fault;
    if (value != 0) te_nonzero_ = true;
    if (te_feed_count_ < 2) return WatchdogVerdict::Pending;  // R 読取待ち
    return te_nonzero_ ? WatchdogVerdict::Fault : WatchdogVerdict::ProceedRecover;
  }

 private:
  bool torque_may_be_on_;
  uint8_t tripped_value_;
  int raw_feed_count_ = 0;
  bool raw_tripped_ = false;
  int te_feed_count_ = 0;
  bool te_nonzero_ = false;
};

// 復旧頻度制限 (60s 内 3 回で FAULT)。現行 :390-397 の recover_count_/
// recover_window_start_s_ 状態機械を逐語移動 (リセット条件 count==0 ||
// (now - start) > window、++count >= max で deny)。
class WatchdogRecoverLimiter {
 public:
  // 復旧を許可するなら true。deny (= FAULT) 時も現行同様 count は増加済みの
  // まま。
  bool allow(float now_s, float window_s, int max_count) {
    if (recover_count_ == 0 || (now_s - recover_window_start_s_) > window_s) {
      recover_count_ = 0;
      recover_window_start_s_ = now_s;
    }
    ++recover_count_;
    return recover_count_ < max_count;
  }

 private:
  int recover_count_ = 0;
  float recover_window_start_s_ = 0.0f;
};

}  // namespace core
