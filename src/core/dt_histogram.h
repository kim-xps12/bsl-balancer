// dt_histogram.h — 制御周期 dt の累積ヒストグラム分類 (UDP telemetry Phase1 計画書 §3.1)。
// Arduino 非依存・純関数。ControlTask が毎周期呼び、Snapshot::dt_hist_total /
// overrun_total (新設の monotonic 専用カウンタ) を更新するために使う。
// bin 境界は周期比: <1.02x, <1.05x, <1.1x, <1.2x, <1.3x, <1.5x, <2.0x, >=2.0x
// (微小 jitter の劣化も検出できる分解能。計画書 §5 の受け入れ基準はこの bin 比率で定義)
#pragma once

namespace core {

constexpr int kDtHistogramBins = 8;

// dt/period_s の比率がどの bin に入るかを返す (0..kDtHistogramBins-1)。
inline int classifyDtBin(float dt, float period_s) {
  const float ratio = dt / period_s;
  if (ratio < 1.02f) return 0;
  if (ratio < 1.05f) return 1;
  if (ratio < 1.10f) return 2;
  if (ratio < 1.20f) return 3;
  if (ratio < 1.30f) return 4;
  if (ratio < 1.50f) return 5;
  if (ratio < 2.00f) return 6;
  return 7;
}

// overrun_total の加算条件 (計画書 §3.1: dt > 1.5x 周期の累積総回数)。
inline bool isOverrunDt(float dt, float period_s) {
  return dt > period_s * 1.5f;
}

}  // namespace core
