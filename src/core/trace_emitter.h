// trace_emitter.h — Wi-Fi ガード計装 (trace) の grammar v1 行整形 + tick 内
// シーケンシング (計画書 docs/plans/2026-07-06-wifi-guard-trace-bench.md D3/D10/D11)。
// Arduino 非依存 (native テスト・g++ 直接コンパイルの native ハーネス双方で
// 使用可能)。BSL_WIFI_GUARD_TRACE ビルドでのみ telemetry_task.cpp から include
// される (リリース TU には一切現れない)。
//
// 設計方針 (D11):
//   - core::WifiGuard 型にも Arduino/Serial にも一切依存しない。呼び出し側
//     (telemetry_task.cpp の trace 統合コード、または native テスト/フィクスチャ
//     生成ハーネス) が esp_timer_get_time() 相当の時刻・WifiGuard の各種
//     アクセサ値を「値」として都度渡す (関数引数による実質的な依存注入)。
//   - 出力は Serial へ直接書かず、内部の行バッファ (固定サイズ 4096B + drop
//     専用予約 64B) に蓄積するのみ。呼び出し側が bufferData()/bufferedBytes()
//     を読んで任意の sink (実機では Serial.write、native テストでは
//     std::string 等) へ書き出し、resetBuffer() で次 tick に備える
//     (出力 sink も「呼び出し側が選ぶ」という形で注入される)。
//   - 本ヘッダ内の char 配列はすべて「1 回のフォーマット呼び出しの間だけ生存する
//     関数ローカルの整形用スクラッチ」であり、tick を跨いで蓄積する行バッファ
//     (data_, 4096+64B) とは別物。data_ は TraceEmitter インスタンスのメンバで
//     あり、呼び出し側 (telemetry_task.cpp) がインスタンス自体を static 領域に
//     置くことで「行バッファをタスクスタックに置かない」要件 (計画書 D3
//     スタック予算) を満たす。
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace core {
namespace trace {

// ---- grammar v1 レコード種別 (計画書 D3) -----------------------------------

enum class EvKind : uint8_t { GotIp, Disc, Stop, Start, Conn, Scan };
enum class OpKind : uint8_t { Begin, Disc, RadioOff, Bp };

inline const char* evTag(EvKind k) {
  switch (k) {
    case EvKind::GotIp: return "gotip";
    case EvKind::Disc:  return "disc";
    case EvKind::Stop:  return "stop";
    case EvKind::Start: return "start";
    case EvKind::Conn:  return "conn";
    case EvKind::Scan:  return "scan";
  }
  return "?";
}

inline const char* opTag(OpKind k) {
  switch (k) {
    case OpKind::Begin:    return "begin";
    case OpKind::Disc:     return "disc";
    case OpKind::RadioOff: return "radio_off";
    case OpKind::Bp:       return "bp";
  }
  return "?";
}

// ---- D10: trace 専用イベント SPSC リング (容量16。guard 既存キューと同型) ----
// producer = Wi-Fi イベントコールバック文脈 (telemetry_task.cpp の onWifiEvent)。
// guard への pushEvent() の**後**に push される (計画書 D3 ev.i の前提: 単一
// producer が guard キュー → trace リングの順に push するため、i は guard
// キュー内の到着順序と一致する)。consumer = telemetry task 単一 (drainRing())。

struct TraceEvent {
  uint64_t t_us = 0;
  EvKind kind = EvKind::GotIp;
  uint8_t reason = 0;  // 有効なのは kind==Disc のみ (grammar: それ以外は r=-)
  int32_t i = -1;      // guard 対象 (gotip/disc/stop) にのみ振る到着通し番号。
                       // trace 専用購読 (start/conn/scan) は -1 (grammar i=-)

  // 明示コンストラクタ (ESP32 Arduino ツールチェーン (xtensa-esp32-elf-g++) は
  // -std=gnu++11 でビルドされ、既定メンバ初期化子を持つクラスは C++11 では
  // 集成体ではないため `TraceEventRing::push({...})` のような波括弧初期化が
  // 集成体初期化として通らない (native env の gnu++17 では集成体扱いになり
  // 問題が隠れていた)。明示コンストラクタを与えることで両方の標準で
  // 一貫して波括弧初期化 (直接リスト初期化としてこのコンストラクタに解決) が
  // 機能するようにする。
  TraceEvent() = default;
  TraceEvent(uint64_t t_us_, EvKind kind_, uint8_t reason_, int32_t i_)
      : t_us(t_us_), kind(kind_), reason(reason_), i(i_) {}
};

class TraceEventRing {
 public:
  static constexpr uint32_t kCapacity = 16;

  // producer から呼ぶ。満杯時は破棄し、破棄数を atomic カウンタへ計上する
  // (guard 既存イベントキューと同じ fail-closed 方針)。
  bool push(const TraceEvent& e) {
    const uint32_t h = head_.load(std::memory_order_relaxed);
    const uint32_t t = tail_.load(std::memory_order_acquire);
    if (h - t >= kCapacity) {
      drop_count_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    buf_[h % kCapacity] = e;
    head_.store(h + 1, std::memory_order_release);
    return true;
  }

  // consumer (単一, telemetry task) から呼ぶ。
  bool pop(TraceEvent* out) {
    const uint32_t t = tail_.load(std::memory_order_relaxed);
    const uint32_t h = head_.load(std::memory_order_acquire);
    if (t == h) return false;
    *out = buf_[t % kCapacity];
    tail_.store(t + 1, std::memory_order_release);
    return true;
  }

  // 破棄数を読み出し+リセットする (guard の event_overflow_ と同じ exchange 方式)。
  uint32_t takeDropCount() { return drop_count_.exchange(0, std::memory_order_acq_rel); }

 private:
  TraceEvent buf_[kCapacity] = {};
  std::atomic<uint32_t> head_{0};
  std::atomic<uint32_t> tail_{0};
  std::atomic<uint32_t> drop_count_{0};
};

// ---- op=bp 専用の pre/post SharedState 即時再サンプル (計画書 D3 R4 sampled 契約) ----

struct OpResample {
  bool read_ok2 = false;  // bp 直前サンプル (fsm2/arm2)
  uint8_t fsm2 = 0;
  bool arm2 = false;
  bool read_ok3 = false;  // bp 戻り直後サンプル (fsm3/arm3)
  uint8_t fsm3 = 0;
  bool arm3 = false;
};

// ---- st/hb の状態タプル (計画書 D3) -----------------------------------------

struct StateTuple {
  bool read_ok = false;  // fsm/arm の有効性 (false なら両方 "-" 表示)
  uint8_t fsm = 0;
  bool arm = false;
  bool q = false;
  bool conn = false;
  bool cing = false;
  bool udpr = false;
  bool librp = false;
  bool ab = false;
  bool roff = false;
  bool latch = false;
  uint32_t pwf = 0;
  uint32_t sf = 0;
  uint32_t evo = 0;

  // st 行の出力トリガとなる「状態タプル」の等価性判定 (計画書 D3: st は
  // 「状態タプルが前回から変化した時のみ」出力する)。pwf/sf/evo は失敗の度に
  // 増減し得る補助カウンタであり、これらの変化だけで毎 tick st を出すとノイズに
  // なるため比較対象からは除外する (出力する値自体は常に最新の値)。
  bool sameStateAs(const StateTuple& o) const {
    return read_ok == o.read_ok && fsm == o.fsm && arm == o.arm && q == o.q &&
           conn == o.conn && cing == o.cing && udpr == o.udpr &&
           librp == o.librp && ab == o.ab && roff == o.roff && latch == o.latch;
  }
};

// ---- バイト予算 (計画書 D3) --------------------------------------------------

inline constexpr size_t kLineBufferBytes = 4096;
inline constexpr size_t kDropReserveBytes = 64;
inline constexpr size_t kUsableBufferBytes = kLineBufferBytes - kDropReserveBytes;

// レコード種別ごとの最大行長 (native テストが実際の snprintf 出力長を計測して
// これを超えないことを assert する。計画書 D3 の宣言バースト計算
// (tk≈50B, ev≈60B, op≈120B, st≈130B, hb≈170B) に安全マージンを載せた宣言値)。
inline constexpr size_t kMaxLineBoot = 96;
inline constexpr size_t kMaxLineEv = 80;
inline constexpr size_t kMaxLineTk = 80;
inline constexpr size_t kMaxLineOp = 224;
inline constexpr size_t kMaxLineSt = 224;
inline constexpr size_t kMaxLineHb = 320;
inline constexpr size_t kMaxLineDrop = 48;

// 宣言最悪バースト (1 tk + 16 ev + 4 op + st + hb) が非予約領域に収まることを
// コンパイル時に固定する (drop は予約 64B 側で別枠保証されるためここには含めない)。
inline constexpr size_t kWorstCaseBurstBytes =
    kMaxLineTk + TraceEventRing::kCapacity * kMaxLineEv + 4 * kMaxLineOp +
    kMaxLineSt + kMaxLineHb;
static_assert(kWorstCaseBurstBytes <= kUsableBufferBytes,
              "trace worst-case burst exceeds line buffer budget (D3)");
static_assert(kMaxLineDrop <= kDropReserveBytes,
              "drop marker must fit in the reserved slot (D3)");

// ---- TraceEmitter: 行整形 + tick 内シーケンシング (D11) ---------------------
//
// 呼び出し順序契約 (1 tick あたり、telemetry_task.cpp / native ハーネス共通):
//   emitBoot()            … task 起動直後に 1 回だけ
//   beginTick()           … tick 冒頭、guard.tick() 呼び出し **前**
//   drainRing()           … tk の直後、guard.tick() 呼び出し **前**
//   (guard.tick()/trySend() 実行。Ops ラッパが recordOp() を呼ぶ)
//   endTick()             … tick 末尾 (st 差分)
//   maybeHeartbeat()       … tick 末尾 (20 tick ごとに hb)
//   maybeEmitDrop()        … tick 末尾 (evdrop 増加時のみ drop 行)
//   (呼び出し側が bufferData()/bufferedBytes() を sink へ書き出し、
//    recordFlushDuration() で所要時間を記録、resetBuffer() で次 tick へ)
class TraceEmitter {
 public:
  TraceEmitter() = default;

  // ---- 出力バッファ (呼び出し側が static 領域に本インスタンスを置くことで
  // 「タスクスタック禁止」要件 (計画書 D3 スタック予算) を満たす) ----
  const char* bufferData() const { return data_; }
  size_t bufferedBytes() const { return used_; }
  void resetBuffer() { used_ = 0; }

  uint32_t evdropTotal() const { return evdrop_total_; }

  // boot 行 (telemetry task 起動直後、一切の guard tick / Wi-Fi 操作より前に
  // 1 回だけ呼ぶ。計画書 D3)。
  void emitBoot(uint64_t t_us, const char* fw, const char* dev) {
    char line[kMaxLineBoot];
    const int n = std::snprintf(line, sizeof(line),
                                "[WG1] s=%u t=%llu boot v=1 fw=%s dev=%s\n",
                                static_cast<unsigned>(nextSeq()),
                                static_cast<unsigned long long>(t_us), fw, dev);
    appendLine(line, n, sizeof(line));
  }

  // tick 冒頭 (guard.tick() 呼び出し前): tk 行を出力し、この tick の snapshot
  // 文脈 (以降の op/st 行が参照する) を保持する。ep は呼び出し側が **この tick の
  // guard.tick() 呼び出し直前** に読んだ drainedEpoch() でなければならない
  // (D11 の pre-drain 契約。呼び出し側が「読む→即座にここへ渡す→即座に
  // guard.tick() を呼ぶ」の順を守る限り、単一スレッド上ではこの値は常に
  // pre-drain である)。
  void beginTick(uint64_t t_us, bool read_ok, uint8_t fsm, bool arm, uint32_t ep) {
    tick_read_ok_ = read_ok;
    tick_fsm_ = fsm;
    tick_arm_ = arm;

    char fsm_tok[8];
    char arm_tok[8];
    fmtOptFsm(fsm_tok, sizeof(fsm_tok), read_ok, fsm);
    fmtOptBool(arm_tok, sizeof(arm_tok), read_ok, arm);

    char line[kMaxLineTk];
    const int n = std::snprintf(line, sizeof(line),
                                "[WG1] s=%u t=%llu tk fsm=%s arm=%s ep=%u\n",
                                static_cast<unsigned>(nextSeq()),
                                static_cast<unsigned long long>(t_us), fsm_tok,
                                arm_tok, static_cast<unsigned>(ep));
    appendLine(line, n, sizeof(line));
  }

  // trace 専用リングを空になるまで drain し ev 行化する (tk の直後、
  // guard.tick() 呼び出し前に呼ぶ。D11 の tick 内シーケンシング)。
  void drainRing(TraceEventRing& ring) {
    TraceEvent ev;
    while (ring.pop(&ev)) {
      char r_tok[8];
      if (ev.kind == EvKind::Disc) {
        std::snprintf(r_tok, sizeof(r_tok), "%u", static_cast<unsigned>(ev.reason));
      } else {
        std::snprintf(r_tok, sizeof(r_tok), "-");
      }
      char i_tok[16];
      if (ev.i >= 0) {
        std::snprintf(i_tok, sizeof(i_tok), "%d", static_cast<int>(ev.i));
      } else {
        std::snprintf(i_tok, sizeof(i_tok), "-");
      }
      char line[kMaxLineEv];
      const int n = std::snprintf(line, sizeof(line),
                                  "[WG1] s=%u t=%llu ev=%s r=%s i=%s\n",
                                  static_cast<unsigned>(nextSeq()),
                                  static_cast<unsigned long long>(ev.t_us),
                                  evTag(ev.kind), r_tok, i_tok);
      appendLine(line, n, sizeof(line));
    }
    const uint32_t dropped = ring.takeDropCount();
    if (dropped) evdrop_total_ += dropped;
  }

  // Ops ラッパ (D4) から呼ぶ。res_valid=false なら res 列は "-"。resample は
  // kind==Bp のときのみ参照する (grammar: bp 行だけ fsm2/arm2/fsm3/arm3 必須)。
  void recordOp(OpKind kind, uint64_t t_us, uint64_t dur_us, bool res_valid, int res,
               bool q, bool udpr, const OpResample& resample = OpResample{}) {
    char fsm_tok[8], arm_tok[8], res_tok[16];
    fmtOptFsm(fsm_tok, sizeof(fsm_tok), tick_read_ok_, tick_fsm_);
    fmtOptBool(arm_tok, sizeof(arm_tok), tick_read_ok_, tick_arm_);
    if (res_valid) std::snprintf(res_tok, sizeof(res_tok), "%d", res);
    else std::snprintf(res_tok, sizeof(res_tok), "-");

    char line[kMaxLineOp];
    int n;
    if (kind == OpKind::Bp) {
      char fsm2_tok[8], arm2_tok[8], fsm3_tok[8], arm3_tok[8];
      fmtOptFsm(fsm2_tok, sizeof(fsm2_tok), resample.read_ok2, resample.fsm2);
      fmtOptBool(arm2_tok, sizeof(arm2_tok), resample.read_ok2, resample.arm2);
      fmtOptFsm(fsm3_tok, sizeof(fsm3_tok), resample.read_ok3, resample.fsm3);
      fmtOptBool(arm3_tok, sizeof(arm3_tok), resample.read_ok3, resample.arm3);
      n = std::snprintf(line, sizeof(line),
          "[WG1] s=%u t=%llu op=%s fsm=%s arm=%s q=%d udpr=%d res=%s dur=%llu "
          "fsm2=%s arm2=%s fsm3=%s arm3=%s\n",
          static_cast<unsigned>(nextSeq()), static_cast<unsigned long long>(t_us),
          opTag(kind), fsm_tok, arm_tok, q ? 1 : 0, udpr ? 1 : 0, res_tok,
          static_cast<unsigned long long>(dur_us), fsm2_tok, arm2_tok, fsm3_tok,
          arm3_tok);
    } else {
      n = std::snprintf(line, sizeof(line),
          "[WG1] s=%u t=%llu op=%s fsm=%s arm=%s q=%d udpr=%d res=%s dur=%llu\n",
          static_cast<unsigned>(nextSeq()), static_cast<unsigned long long>(t_us),
          opTag(kind), fsm_tok, arm_tok, q ? 1 : 0, udpr ? 1 : 0, res_tok,
          static_cast<unsigned long long>(dur_us));
    }
    appendLine(line, n, sizeof(line));
  }

  // tick 末尾: 状態タプルが前回から変化していれば st 行を出す。
  void endTick(uint64_t t_us, const StateTuple& current) {
    if (!have_prev_tuple_ || !current.sameStateAs(prev_tuple_)) {
      emitStateLine(t_us, current);
    }
    prev_tuple_ = current;
    have_prev_tuple_ = true;
  }

  // 20 tick (=1s @50ms 周期) ごとに呼ぶ。heap/heapmin/stkmin は呼び出し側が
  // 実測 (ESP.getFreeHeap() 等) して渡す値をそのまま印字する。
  void maybeHeartbeat(uint64_t t_us, uint32_t tick_number, const StateTuple& current,
                      uint32_t heap, uint32_t heapmin, uint32_t stkmin) {
    if (tick_number == 0 || tick_number % kHeartbeatPeriodTicks != 0) return;
    emitHeartbeatLine(t_us, current, heap, heapmin, stkmin, tick_number);
  }

  // このtick中にリング/行バッファの破棄が起きていれば (evdrop_total_ が前回
  // 報告値から増えていれば) 予約枠へ drop 行を出す (計画書 D3: 1Hzのhbを
  // 待たずにそのtick中に自己申告する。予約枠を使うため必ず出力できる)。
  void maybeEmitDrop(uint64_t t_us) {
    if (evdrop_total_ == evdrop_reported_) return;
    evdrop_reported_ = evdrop_total_;
    char line[kMaxLineDrop];
    const int n = std::snprintf(line, sizeof(line), "[WG1] s=%u t=%llu drop n=%u\n",
                                static_cast<unsigned>(nextSeq()),
                                static_cast<unsigned long long>(t_us),
                                static_cast<unsigned>(evdrop_total_));
    appendDropReserved(line, n, sizeof(line));
  }

  // フラッシュ (Serial.write 等) の前後で計測した所要時間を記録する
  // (hb の flmax = 前回 hb 以降の最大フラッシュ所要時間、計画書 D3/D9)。
  void recordFlushDuration(uint64_t dur_us) {
    if (dur_us > flush_max_us_since_hb_) flush_max_us_since_hb_ = dur_us;
  }

 private:
  static constexpr uint32_t kHeartbeatPeriodTicks = 20;

  uint32_t nextSeq() const { return seq_ + 1; }

  static void fmtOptFsm(char* out, size_t outsz, bool valid, uint8_t fsm) {
    if (!valid) std::snprintf(out, outsz, "-");
    else std::snprintf(out, outsz, "%u", static_cast<unsigned>(fsm));
  }
  static void fmtOptBool(char* out, size_t outsz, bool valid, bool v) {
    if (!valid) std::snprintf(out, outsz, "-");
    else std::snprintf(out, outsz, "%d", v ? 1 : 0);
  }

  void emitStateLine(uint64_t t_us, const StateTuple& s) {
    char fsm_tok[8], arm_tok[8];
    fmtOptFsm(fsm_tok, sizeof(fsm_tok), s.read_ok, s.fsm);
    fmtOptBool(arm_tok, sizeof(arm_tok), s.read_ok, s.arm);
    char line[kMaxLineSt];
    const int n = std::snprintf(line, sizeof(line),
        "[WG1] s=%u t=%llu st fsm=%s arm=%s q=%d conn=%d cing=%d udpr=%d "
        "librp=%d ab=%d roff=%d latch=%d pwf=%u sf=%u evo=%u\n",
        static_cast<unsigned>(nextSeq()), static_cast<unsigned long long>(t_us),
        fsm_tok, arm_tok, s.q ? 1 : 0, s.conn ? 1 : 0, s.cing ? 1 : 0,
        s.udpr ? 1 : 0, s.librp ? 1 : 0, s.ab ? 1 : 0, s.roff ? 1 : 0,
        s.latch ? 1 : 0, static_cast<unsigned>(s.pwf), static_cast<unsigned>(s.sf),
        static_cast<unsigned>(s.evo));
    appendLine(line, n, sizeof(line));
  }

  void emitHeartbeatLine(uint64_t t_us, const StateTuple& s, uint32_t heap,
                        uint32_t heapmin, uint32_t stkmin, uint32_t tick_number) {
    char fsm_tok[8], arm_tok[8];
    fmtOptFsm(fsm_tok, sizeof(fsm_tok), s.read_ok, s.fsm);
    fmtOptBool(arm_tok, sizeof(arm_tok), s.read_ok, s.arm);
    char line[kMaxLineHb];
    const int n = std::snprintf(line, sizeof(line),
        "[WG1] s=%u t=%llu hb fsm=%s arm=%s q=%d conn=%d cing=%d udpr=%d "
        "librp=%d ab=%d roff=%d latch=%d pwf=%u sf=%u evo=%u heap=%u "
        "heapmin=%u evdrop=%u flmax=%llu stkmin=%u tick=%u\n",
        static_cast<unsigned>(nextSeq()), static_cast<unsigned long long>(t_us),
        fsm_tok, arm_tok, s.q ? 1 : 0, s.conn ? 1 : 0, s.cing ? 1 : 0,
        s.udpr ? 1 : 0, s.librp ? 1 : 0, s.ab ? 1 : 0, s.roff ? 1 : 0,
        s.latch ? 1 : 0, static_cast<unsigned>(s.pwf), static_cast<unsigned>(s.sf),
        static_cast<unsigned>(s.evo), static_cast<unsigned>(heap),
        static_cast<unsigned>(heapmin), static_cast<unsigned>(evdrop_total_),
        static_cast<unsigned long long>(flush_max_us_since_hb_),
        static_cast<unsigned>(stkmin), static_cast<unsigned>(tick_number));
    appendLine(line, n, sizeof(line));
    flush_max_us_since_hb_ = 0;  // 次の hb 区間用にリセット (計画書 D3)
  }

  // 通常行の追加: 非予約領域 (kUsableBufferBytes) に収まる場合のみ書き込み、
  // 実際に書き込めた行だけ s (seq_) を進める (欠番を作らない。ゲート1第16回
  // 指摘1・第15回指摘1対応: s の連続性を構造的に保証する)。収まらない場合は
  // 行を破棄し evdrop に計上する (黙って欠落させない。計画書 D3)。
  //
  // scratch_capacity = 呼び出し側の整形用スクラッチバッファ (line) の実サイズ
  // (= sizeof(line))。snprintf は出力が収まりきらない場合、「実際に格納できた
  // 長さ」ではなく「省略なしなら本来必要だった長さ」を返す (C99/C++11 の
  // snprintf 契約)。そのため fw/dev 等の可変長文字列 (emitBoot) がスクラッチ
  // バッファを超えると n がその長さより大きくなり得る。旧実装はこの n を
  // そのまま memcpy(dst, line, n) に渡していたため、line (スクラッチバッファ)
  // の境界を超えて読み出す (スタック過読) 上に、切り詰められた grammar 不整合
  // な行が trace 出力に混入し得た (ゲート2レビュー指摘対応)。n が
  // scratch_capacity 以上 (= 切り詰め発生) の場合はその行を一切出力せず
  // evdrop に計上し、drop マーカー経由で自己申告する。
  void appendLine(const char* line, int n, size_t scratch_capacity) {
    if (n < 0 || static_cast<size_t>(n) >= scratch_capacity) {
      ++evdrop_total_;  // 符号化異常、または切り詰め (整形失敗として扱う)
      return;
    }
    const size_t len = static_cast<size_t>(n);
    if (used_ + len > kUsableBufferBytes) {
      ++evdrop_total_;
      return;
    }
    std::memcpy(data_ + used_, line, len);
    used_ += len;
    ++seq_;
  }

  // drop マーカー専用の予約枠 (kDropReserveBytes) への書き込み。予約枠のため
  // 通常行と競合せず、kMaxLineDrop <= kDropReserveBytes (static_assert 済み) に
  // より理論上必ず収まる。drop 行のフィールドはすべて固定範囲の数値のため
  // 実運用では切り詰めは起こらないが、appendLine と同じ理由 (snprintf 契約) で
  // scratch_capacity 超過時は memcpy せず捨てる (スタック過読の防止。
  // ゲート2レビュー指摘と同一パターンの防御を一貫して適用する)。
  void appendDropReserved(const char* line, int n, size_t scratch_capacity) {
    if (n < 0 || static_cast<size_t>(n) >= scratch_capacity) return;
    const size_t len = static_cast<size_t>(n);
    if (used_ + len > kLineBufferBytes) return;  // 理論上到達しない防御的分岐
    std::memcpy(data_ + used_, line, len);
    used_ += len;
    ++seq_;
  }

  char data_[kLineBufferBytes] = {};
  size_t used_ = 0;
  uint32_t seq_ = 0;

  // この tick の snapshot 文脈 (beginTick() で設定、op 行が参照する)
  bool tick_read_ok_ = false;
  uint8_t tick_fsm_ = 0;
  bool tick_arm_ = false;

  // st 差分判定用
  bool have_prev_tuple_ = false;
  StateTuple prev_tuple_{};

  // evdrop 集計 (D3: リング/行バッファ破棄の累積。drop 行は増加分のみ報告)
  uint32_t evdrop_total_ = 0;
  uint32_t evdrop_reported_ = 0;

  // flmax (D3: 前回 hb 以降の最大フラッシュ所要時間)
  uint64_t flush_max_us_since_hb_ = 0;
};

}  // namespace trace
}  // namespace core
