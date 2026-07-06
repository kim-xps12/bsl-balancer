// dt_stats.h — dt p95 計算コア (設計書 §6「トルク OFF で全経路を ~2s 回して
// dt p95 が予算内であること」の土台)。Arduino 非依存・heap 不使用。段階3
// 新設 (FSM への結線 = ゲート判定への使用は段階4。本 PR では未結線・未使用。
// docs/plans/2026-07-06-safety-core-extraction.md §3.4)。
//
// 既存 dt_histogram.h (telemetry 用 monotonic カウンタ。運用中の粗い bin
// 分布を低コストで可視化する目的) とは目的が異なるため統合しない: 本ヘッダ
// は起動時 soak gate 向けに、量子化なしの厳密な nearest-rank p95 を 1 回
// 抽出する用途。
#pragma once

#include <algorithm>
#include <cstddef>

namespace core {

// 固定容量 (kCapacity=512 > 200Hz×2s=400) の dt サンプル窓と p95 抽出。
// heap 不使用 (float 512 本 = 2048B は使用側が静的に確保する想定。結線は
// 段階4)。API は fail-closed (ゲート1第1回指摘4対応 — 空窓/不足窓/溢れ窓が
// 「予算内」に見える構成を型レベルで作らせない): 段階4 のゲートは
// ready() && p95(&v) && v <= budget の形でのみ合格し得る。
class DtP95Window {
 public:
  static constexpr size_t kCapacity = 512;

  // 容量超過時は記録せず overflowed_ を立てる (既存サンプルは保持したまま)。
  void add(float dt_s) {
    if (size_ < kCapacity) {
      buf_[size_++] = dt_s;
    } else {
      overflowed_ = true;
    }
  }

  size_t size() const { return size_; }
  bool overflowed() const { return overflowed_; }

  // 窓がゲート判定に使える状態か (サンプル数下限を満たし、溢れていない)。
  bool ready(size_t min_samples) const {
    return !overflowed_ && size_ >= min_samples;
  }

  // nearest-rank 法: idx = ceil(0.95*n) - 1。n==0 は false を返し *out 不変
  // (空窓を数値として扱わせない)。作業配列を内部コピーして選択、入力順は
  // 非破壊 (add() を呼出し前後で継続可能)。
  // idx の算出は 0.95 = 19/20 の厳密な整数演算 (19*n+19)/20 - 1 で行い、
  // ceil(0.95*n) を浮動小数点で計算した場合の丸め誤差 (n が大きいほど
  // 境界値で ceil の結果が 1 ずれ得る) を避ける。数学的には
  // ceil(19*n/20) == floor((19*n + 19)/20) (正整数の天井除算の標準変形)。
  bool p95(float* out) const {
    if (size_ == 0) return false;
    const size_t idx = (19 * size_ + 19) / 20 - 1;
    float work[kCapacity];
    std::copy(buf_, buf_ + size_, work);
    std::nth_element(work, work + idx, work + size_);
    *out = work[idx];
    return true;
  }

  void reset() {
    size_ = 0;
    overflowed_ = false;
  }

 private:
  float buf_[kCapacity];
  size_t size_ = 0;
  bool overflowed_ = false;
};

}  // namespace core
