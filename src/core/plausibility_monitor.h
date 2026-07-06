// plausibility_monitor.h — 電流妥当性監視 (設計書 §6)。Arduino 非依存
// (内部状態は dwell_ のみ・I/O なし)。既に完全に純粋だったクラス本体を
// tasks/control_task.cpp から逐語移動する (段階2。
// docs/plans/2026-07-06-safety-core-extraction.md §3.3)。
#pragma once

#include <cmath>

#include "../app_config.h"

namespace core {

// 電流妥当性監視 (§6): dwell 付きの単純な監視器
class PlausibilityMonitor {
 public:
  // torque_on 中: |I_pres−I_cmd| 乖離、torque_off: 残留電流を監視
  bool update(bool torque_on, bool fb_valid, float i_cmd, float i_pres, float dt) {
    if (!fb_valid) return false;  // stale 帰還では判定しない
    const float err = std::fabs(i_pres - i_cmd);
    const bool bad = torque_on ? (err > cfg::kCurrentMismatchA)
                               : (std::fabs(i_pres) > cfg::kCurrentResidualA);
    dwell_ = bad ? (dwell_ + dt) : 0.0f;
    return dwell_ > cfg::kCurrentPlausDwellS;
  }
  void reset() { dwell_ = 0.0f; }

 private:
  float dwell_ = 0.0f;
};

}  // namespace core
