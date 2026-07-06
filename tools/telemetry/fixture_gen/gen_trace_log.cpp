// gen_trace_log.cpp — Wi-Fi ガード計装 (trace) の代表シナリオログを
// src/core/trace_emitter.h + src/core/wifi_guard.h から決定的に生成する
// native ハーネス (wifi-guard-trace 計画書 §4 D11)。
//
// core/ は Arduino 非依存のため g++ 直接コンパイルできる
// (scripts/gen_trace_fixtures.sh からビルド・実行される)。
//
// 時刻はすべて固定シード (単調増加の固定刻み) で進めるため、同じ引数で
// 再実行しても出力は完全に決定的 (diff ゼロ) である。壁時計・乱数は
// 一切使用しない。
//
// 使い方: gen_trace_log <scenario>
//   scenario ∈ {t1, t2, t3, t3b, t5, common,
//               t2_immediate_abort, t3_race_predrain_forbidden_op}
//
// 出力は標準出力へ WG1 grammar v1 の行のみを書く (scripts/gen_trace_fixtures.sh
// がファイルへリダイレクトして tools/telemetry/tests/fixtures/ に保存する)。
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "../../../src/core/safety_fsm.h"
#include "../../../src/core/trace_emitter.h"
#include "../../../src/core/wifi_guard.h"

namespace trace = core::trace;
using core::WifiGuard;

namespace {

// ---- WifiGuard 用フェイク Ops + trace 記録 (telemetry_task.cpp の
// TraceOps* ラッパと同じ役割を native で再現する) ----
struct Ctx {
  // フェイク Wi-Fi/UDP 状態
  bool disconnect_return = true;
  bool radio_off_status = false;
  int begin_packet_return = 1;

  trace::TraceEmitter emitter;
  trace::TraceEventRing ring;
  WifiGuard* guard = nullptr;
  uint32_t guard_event_seq = 0;
  uint64_t clock_us = 1000;
  uint32_t tick_num = 0;
  uint8_t last_fsm = 0;
  bool last_arm = false;
  std::string log;

  // Wi-Fi イベント到着を模擬する (guard へ push → trace ring へ push の順。
  // 計画書 D3 ev.i の前提となる単一 producer の順序と同じ)。
  void injectEvent(WifiGuard::EventKind kind, uint8_t reason, trace::EvKind tk) {
    guard->pushEvent(kind, reason);
    ring.push(trace::TraceEvent{clock_us, tk, reason,
                                static_cast<int32_t>(++guard_event_seq)});
  }

  // trace 専用の追加購読 (start/conn/scan, i=-)。
  void injectExtraEvent(trace::EvKind tk) {
    ring.push(trace::TraceEvent{clock_us, tk, 0, -1});
  }

  uint32_t beginTick(uint8_t fsm, bool arm, uint64_t dt_us = 50000) {
    ++tick_num;
    last_fsm = fsm;
    last_arm = arm;
    clock_us += dt_us;
    const uint32_t ep = guard->drainedEpoch();
    emitter.beginTick(clock_us, true, fsm, arm, ep);
    emitter.drainRing(ring);
    return ep;
  }

  trace::StateTuple snapshot() const {
    trace::StateTuple s;
    s.read_ok = true;
    s.fsm = last_fsm;
    s.arm = last_arm;
    s.q = guard->lastWifiQuiet();
    s.conn = guard->connected();
    s.cing = guard->connecting();
    s.udpr = guard->udpReady();
    s.librp = guard->libReconnectPending();
    s.ab = guard->aborting();
    s.roff = guard->radioOffPending();
    s.latch = guard->wifiAbortFailed();
    s.pwf = guard->prewarmFailTotal();
    s.sf = guard->sendFailTotal();
    s.evo = guard->eventOverflowTotal();
    return s;
  }

  void endTick() {
    clock_us += 200;
    emitter.endTick(clock_us, snapshot());
    emitter.maybeHeartbeat(clock_us, tick_num, snapshot(), /*heap=*/180000,
                           /*heapmin=*/170000, /*stkmin=*/4096);
    emitter.maybeEmitDrop(clock_us);
  }

  // 1 tick 分を flush して log へ蓄積し、次 tick に備えてバッファをリセットする
  // (本番の Serial.write + resetBuffer() と同じ「1 tick 分ずつ書き出す」動作)。
  void flush() {
    log.append(emitter.bufferData(), emitter.bufferedBytes());
    emitter.resetBuffer();
  }

  // telemetry_task.cpp の 1 tick 分 (tk → drain → guard.tick()/trySend() →
  // st/hb/drop → flush) を丸ごと再現する。
  void runTick(uint8_t fsm, bool arm, uint32_t loop_count) {
    beginTick(fsm, arm);
    const bool balancing = fsm == static_cast<uint8_t>(core::FsmState::Balancing);
    guard->tick(true, loop_count, balancing, arm);
    if (guard->readyToAttempt()) {
      const uint8_t payload[4] = {1, 2, 3, 4};
      guard->trySend(payload, sizeof(payload));
    }
    endTick();
    flush();
  }

  // 末尾の完全性要件 (計画書 D3 共通妥当性: (b) hb が2本以上・(c) 時系列最後の
  // [WG1] 行が hb・(d) 最後の ev/op の後に少なくとも1本の hb) を満たすため、
  // 変化なしのアイドル tick を最低 min_extra_ticks 回し、さらに tick_num が
  // 20 の倍数 (= 直近 tick が必ず hb を出す) になるまで追加で回す。これにより
  // どのシナリオも「最後の [WG1] 行が hb」で終わることを構造的に保証する。
  void settleTail(uint8_t fsm, bool arm, uint32_t loop_count_start,
                  int min_extra_ticks = 45) {
    uint32_t lc = loop_count_start;
    for (int i = 0; i < min_extra_ticks; ++i) runTick(fsm, arm, lc++);
    while (tick_num % 20 != 0) runTick(fsm, arm, lc++);
  }
};

void OpsBegin(void* ctxv) {
  auto* c = static_cast<Ctx*>(ctxv);
  const uint64_t t0 = c->clock_us;
  c->clock_us += 300;
  c->emitter.recordOp(trace::OpKind::Begin, t0, c->clock_us - t0,
                      /*res_valid=*/false, 0, c->guard->lastWifiQuiet(),
                      c->guard->udpReady());
}
bool OpsDisconnect(void* ctxv) {
  auto* c = static_cast<Ctx*>(ctxv);
  const uint64_t t0 = c->clock_us;
  c->clock_us += 300;
  const bool res = c->disconnect_return;
  c->emitter.recordOp(trace::OpKind::Disc, t0, c->clock_us - t0,
                      /*res_valid=*/true, res ? 1 : 0, c->guard->lastWifiQuiet(),
                      c->guard->udpReady());
  return res;
}
void OpsSetRadioOff(void* ctxv) {
  auto* c = static_cast<Ctx*>(ctxv);
  const uint64_t t0 = c->clock_us;
  c->clock_us += 300;
  c->emitter.recordOp(trace::OpKind::RadioOff, t0, c->clock_us - t0,
                      /*res_valid=*/false, 0, c->guard->lastWifiQuiet(),
                      c->guard->udpReady());
}
bool OpsIsRadioOff(void* ctxv) { return static_cast<Ctx*>(ctxv)->radio_off_status; }
int OpsBeginPacket(void* ctxv) {
  auto* c = static_cast<Ctx*>(ctxv);
  trace::OpResample resample;
  resample.read_ok2 = true;
  resample.fsm2 = c->last_fsm;
  resample.arm2 = c->last_arm;
  const uint64_t t0 = c->clock_us;
  c->clock_us += 300;
  const int res = c->begin_packet_return;
  resample.read_ok3 = true;
  resample.fsm3 = c->last_fsm;
  resample.arm3 = c->last_arm;
  c->emitter.recordOp(trace::OpKind::Bp, t0, c->clock_us - t0, /*res_valid=*/true,
                      res, c->guard->lastWifiQuiet(), c->guard->udpReady(), resample);
  return res;
}
int OpsWritePacket(void*, const uint8_t*, size_t len) { return static_cast<int>(len); }
int OpsEndPacket(void*) { return 1; }

core::WifiOps makeOps(Ctx* c) {
  core::WifiOps o;
  o.ctx = c;
  o.begin = &OpsBegin;
  o.disconnect = &OpsDisconnect;
  o.setRadioOff = &OpsSetRadioOff;
  o.isRadioOff = &OpsIsRadioOff;
  o.beginPacket = &OpsBeginPacket;
  o.writePacket = &OpsWritePacket;
  o.endPacket = &OpsEndPacket;
  return o;
}

constexpr uint8_t kIdle = static_cast<uint8_t>(core::FsmState::Idle);
constexpr uint8_t kBalancing = static_cast<uint8_t>(core::FsmState::Balancing);

void emitBootAndFlush(Ctx& c, const char* fw, const char* dev) {
  c.emitter.emitBoot(c.clock_us, fw, dev);
  c.flush();
}

// 接続確立 + prewarm 完了までの共通シーケンス (T1/T2/T3/T3b/T5/common で共通の
// 前段として使う): tick1 fresh未確定 → tick2 begin → tick3 GOT_IP + prewarm。
uint32_t establishConnection(Ctx& c, uint32_t loop_count) {
  c.runTick(kIdle, false, loop_count++);
  c.runTick(kIdle, false, loop_count++);
  c.injectEvent(WifiGuard::EventKind::GotIp, 0, trace::EvKind::GotIp);
  c.runTick(kIdle, false, loop_count++);
  return loop_count;
}

// ---- T1 (C1): AP 停止中の Balancing (非 CONNECTING) で Wi-Fi 操作が発行
// されない ----
void scenarioT1(Ctx& c) {
  emitBootAndFlush(c, "t1fixture", "core2-t1");
  uint32_t loop = 1;
  // tick1: fresh 未確定 (quiet)
  c.runTick(kIdle, false, loop++);
  // tick2: fresh 確立・非quiet → begin 発行 (AP 不在のため接続は成立しない)
  c.runTick(kIdle, false, loop++);
  // AP 不在による接続失敗 (非自発的切断、reason=201=NO_AP_FOUND 相当) →
  // ライブラリ first_connect one-shot が発火 (lib_reconnect_pending_=true)
  c.injectEvent(WifiGuard::EventKind::StaDisconnected, 201, trace::EvKind::Disc);
  c.runTick(kIdle, false, loop++);
  // one-shot も AP 不在で失敗 (2 回目の非自発的切断) → in-flight 終了とみなされ
  // lib_reconnect_pending_ が clear される (guard 既存ロジック、計画書 §3.1 (b))
  c.injectEvent(WifiGuard::EventKind::StaDisconnected, 201, trace::EvKind::Disc);
  c.runTick(kIdle, false, loop++);
  // backoff 消化を待つ (cing=0 の settled 状態を st で確認できるまで)
  for (int i = 0; i < 8; ++i) c.runTick(kIdle, false, loop++);
  // 直立保持で Balancing へ (窓開始時点で cing=0 ∧ librp=0 ∧ ab=0)
  for (int i = 0; i < 6; ++i) c.runTick(kBalancing, false, loop++);
  c.settleTail(kBalancing, false, loop, 45);
}

// ---- T2 (C2): CONNECTING 中の Balancing 遷移でちょうど1回の abort disconnect
// のみ ----
void scenarioT2(Ctx& c, bool immediate_abort_variant) {
  emitBootAndFlush(c, "t2fixture", "core2-t2");
  uint32_t loop = 1;
  c.runTick(kIdle, false, loop++);
  c.runTick(kIdle, false, loop++);  // begin 発行 → connecting_
  // CONNECTING 窓のうちに直立保持で arm → Balancing (同一 tick で即座に abort
  // 発行されるケース。op が st より先に記録されるのが正常 = PASS の代表例)
  c.runTick(kBalancing, false, loop++);
  (void)immediate_abort_variant;  // このシナリオ自体が既に「即時 abort」の例
  // 確認イベント (自発的切断 = ASSOC_LEAVE) 到着 → abort 完了
  c.injectEvent(WifiGuard::EventKind::StaDisconnected, WifiGuard::kReasonAssocLeave,
               trace::EvKind::Disc);
  c.runTick(kBalancing, false, loop++);
  c.settleTail(kBalancing, false, loop, 45);
}

// 共通の episode 1 (T3 本体): GOT_IP 後の AP 喪失 × Balancing 中、one-shot
// 再接続を有界時間でキャンセルする (計画書 §6 T3 前半)。
uint32_t runEpisode1OneShotCancel(Ctx& c, uint32_t loop) {
  c.runTick(kBalancing, false, loop++);
  // AP 電源断 (非自発的切断, reason=200) → one-shot 発火
  c.injectEvent(WifiGuard::EventKind::StaDisconnected, 200, trace::EvKind::Disc);
  c.runTick(kBalancing, false, loop++);  // 有界キャンセル (abort disc) 発行
  // 確認 (自発的切断 = ASSOC_LEAVE) 到着 → abort 完了 (R3 エピソード終了)
  c.injectEvent(WifiGuard::EventKind::StaDisconnected, WifiGuard::kReasonAssocLeave,
               trace::EvKind::Disc);
  c.runTick(kBalancing, false, loop++);
  return loop;
}

// ---- T3 (C3 前半単体): episode 1 のみで完結するフォーカス版フィクスチャ。
// trace_check.py の R3 判定 (`_evaluate_r3`) はログ中の「conn=1∧fsm=2∧
// ev=disc(r!=8)」に一致する**全エピソード**を走査するため (実装確認済み)、
// T3+T3b を単一ログにまとめると 2 個目の episode (burnt 後の再喪失。本来
// one-shot が再発火せず abort 自体が不要というのが正しい挙動 —
// scenarioT3AndT3b のコメント参照) が R3 側からは「未完了の abort」に見えて
// FAIL する (R3b 側は逆にこの episode が静粛であることを要求しており、両立
// しない)。よって `--scenario t3` 用にはこの episode 1 単体版を、
// `--scenario t3b` 用には下記の 2 episode 通し版を、それぞれ独立した
// フィクスチャとして提供する (計画書の「単一の連続キャプチャ」要件は実機
// ベンチ手順のものであり、本 native フィクスチャ生成はチェッカの各ルールを
// 単体で試験する目的のため、ルールごとに焦点を絞ったログを用意する)。
void scenarioT3Only(Ctx& c) {
  emitBootAndFlush(c, "t3fixture", "core2-t3");
  uint32_t loop = 1;
  loop = establishConnection(c, loop);  // Idle で GOT_IP・prewarm 完了
  c.settleTail(kIdle, false, loop, 3);
  loop += 3;
  loop = runEpisode1OneShotCancel(c, loop);
  c.settleTail(kBalancing, false, loop, 45);
}

// ---- T3+T3b (C3 全体, R3b 用): GOT_IP 後の AP 喪失 × Balancing 中の
// one-shot 有界キャンセル (episode 1) → AP 復電・再接続 → 2 度目の喪失
// (burnt 状態、episode 2) が有界キャンセル不要で静粛のままであることを検証
// する連続キャプチャ (計画書 §6 T3+T3b、R3b ヒット条件 (iii))。
// episode 2 で **op も ev も一切出さない** のが正しい挙動である根拠:
// WifiGeneric.cpp の first_connect one-shot は生涯で一度きり
// (lib_first_connect_consumed_)。episode 1 で既に消費済みのため、episode 2
// の非自発的切断は lib_reconnect_pending_ を再セットしない
// (drainEvents() の該当分岐は `!lib_first_connect_consumed_` または
// `lib_reconnect_pending_` のいずれかを要求し、両方 false ならどちらの
// 分岐も実行されない)。in-flight な再接続が存在しない以上、guard 側にも
// キャンセルすべき対象がなく startAbort() は発行されない
// (tick() の abort トリガ条件 `connecting_ || lib_reconnect_pending_ ||
// (connection_established_in_drain && connected_)` がすべて false のため)。
void scenarioT3AndT3b(Ctx& c) {
  emitBootAndFlush(c, "t3fixture", "core2-t3");
  uint32_t loop = 1;
  loop = establishConnection(c, loop);  // Idle で GOT_IP・prewarm 完了
  c.settleTail(kIdle, false, loop, 3);
  loop += 3;

  loop = runEpisode1OneShotCancel(c, loop);

  // AP 復電・Idle に戻す → 再接続 + prewarm 完了 (R3b ヒット条件 (ii))
  c.runTick(kIdle, false, loop++);
  c.runTick(kIdle, false, loop++);  // begin 再発行
  c.injectEvent(WifiGuard::EventKind::GotIp, 0, trace::EvKind::GotIp);
  c.runTick(kIdle, false, loop++);  // GOT_IP + prewarm 完了 (conn=1 ∧ udpr=1)
  c.settleTail(kIdle, false, loop, 3);
  loop += 3;

  // 再び Balancing へ → 2 度目の AP 電源断 (R3b ヒット条件 (iii): burnt 状態
  // での AP 喪失)。lib_first_connect_consumed_ は既に true・lib_reconnect_
  // pending_ は false のため、この disc(r=200) は one-shot を再発火せず、
  // guard は abort を発行しない (上記コメント参照) —— op も追加の ev も
  // 一切出ないのが正しい挙動 (R3b (b) の要求そのもの)。
  c.runTick(kBalancing, false, loop++);
  c.injectEvent(WifiGuard::EventKind::StaDisconnected, 200, trace::EvKind::Disc);
  c.runTick(kBalancing, false, loop++);
  // R3b 観測窓下限 (20s 以上、lib reconnect timeout 15s + マージン) を満たす
  // だけの tick 数を Balancing 継続のまま追加する (50ms/tick 換算で 20s ≈
  // 400 tick。フィクスチャの決定性・生成コストを優先し、tick 数はそのままに
  // t (時刻) 側で 20s 以上経過したことが分かるよう明示的に大きく進める)。
  for (int i = 0; i < 30; ++i) {
    c.clock_us += 700000;  // 1 tick あたりの経過時間を水増しし t 差分で 20s 超を表現
    c.runTick(kBalancing, false, loop++);
  }
  // 末尾の完全性要件 (最後の [WG1] 行が hb) を満たすまで tick_num を 20 の
  // 倍数へ揃える (settleTail と同じ整合ロジック)。
  while (c.tick_num % 20 != 0) {
    c.clock_us += 700000;
    c.runTick(kBalancing, false, loop++);
  }
}

// ---- T5 (C5): arm_pending (直立ホールド) 中の AP 喪失で one-shot が発火して
// も 1 回の abort でホールド中に中断・フラグクリア ----
void scenarioT5(Ctx& c) {
  emitBootAndFlush(c, "t5fixture", "core2-t5");
  uint32_t loop = 1;
  loop = establishConnection(c, loop);
  c.settleTail(kIdle, false, loop, 3);
  loop += 3;

  // 直立ホールド (arm_pending) 中に AP 喪失
  c.runTick(kIdle, true, loop++);
  c.injectEvent(WifiGuard::EventKind::StaDisconnected, 200, trace::EvKind::Disc);
  c.runTick(kIdle, true, loop++);  // 有界キャンセル (abort disc) 発行
  c.injectEvent(WifiGuard::EventKind::StaDisconnected, WifiGuard::kReasonAssocLeave,
               trace::EvKind::Disc);
  c.runTick(kIdle, true, loop++);  // abort 完了・フラグクリア
  c.settleTail(kIdle, true, loop, 45);
}

// ---- common: 全シナリオ共通の妥当性検査 (boot 配置・s 連続性・hb 完全性等)
// を単体で確認するための素朴な基準セッション。Wi-Fi 活動は一切なし。----
void scenarioCommon(Ctx& c) {
  emitBootAndFlush(c, "commonfix", "core2-cm");
  uint32_t loop = 1;
  // 接続確立済みの健全な状態から Balancing へ (R1 の禁止窓に触れない構成。
  // このシナリオの目的はシナリオ固有の R1〜R5 判定ではなく、boot 配置・s 連続
  // 性・hb 完全性等の共通妥当性検査を単体で確認することにある)。
  loop = establishConnection(c, loop);
  c.settleTail(kIdle, false, loop, 3);
  loop += 3;
  c.settleTail(kBalancing, false, loop, 40);
}

// ---- 特殊 fixture (1): 「tk 記録後・guard drain 前にイベント到着」の
// D11 相互検証 fixture (ゲート1第7回・第9回指摘)。R3 の禁止窓が開いた直後
// (同一 tick 内) に、禁止されるはずの start-class 相当の op (begin) を
// **意図的に**注入し、trace_check が禁止窓内の余分な op としてこれを FAIL
// 報告することを検証するための合成ログ。実際の WifiGuard 実装はこの余分な
// begin を発行しない (readyToAttempt/aborting_ ガードにより自然には起こらない
// ため、trace_check の窓判定ロジックを試験する目的で emitter API を直接叩いて
// 意図的に不正なログを作る)。
void scenarioT3RaceForbiddenOp(Ctx& c) {
  emitBootAndFlush(c, "t3racefix", "core2-t3race");
  uint32_t loop = 1;
  loop = establishConnection(c, loop);
  c.settleTail(kIdle, false, loop, 3);
  loop += 3;

  c.runTick(kBalancing, false, loop++);

  // 「tk 記録後・guard drain 前」の到着を再現する: このtickのtkは、まだ
  // 到着していないイベントの ep (pre-drain) で出力される。
  const uint32_t ep_before = c.beginTick(kBalancing, false);
  c.injectEvent(WifiGuard::EventKind::StaDisconnected, 200, trace::EvKind::Disc);
  c.guard->tick(true, loop++, true, false);  // このtick内でdrainされ abort disc発行
  (void)ep_before;
  // 禁止窓が開いた直後 (この tick 内、trySend の前) に、本来禁止されるはずの
  // start-class 相当の op (begin) を意図的に注入する (実装はこれを発行しない。
  // trace_check の禁止窓検査を試験するための不正合成)。
  c.last_fsm = kBalancing;
  c.last_arm = false;
  const uint64_t t0 = c.clock_us;
  c.clock_us += 300;
  c.emitter.recordOp(trace::OpKind::Begin, t0, c.clock_us - t0, false, 0,
                     c.guard->lastWifiQuiet(), c.guard->udpReady());
  c.endTick();
  c.flush();

  // 確認イベント到着 → abort 完了 (エピソード自体は完結させる)
  c.injectEvent(WifiGuard::EventKind::StaDisconnected, WifiGuard::kReasonAssocLeave,
               trace::EvKind::Disc);
  c.runTick(kBalancing, false, loop++);
  c.settleTail(kBalancing, false, loop, 45);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: gen_trace_log <scenario>\n");
    return 2;
  }
  const std::string scenario = argv[1];

  Ctx c;
  core::WifiOps ops = makeOps(&c);
  WifiGuard::Params p;
  p.abort_confirm_ticks = 2;
  p.abort_max_retries = 2;
  p.lib_reconnect_timeout_ticks = 5;
  // T1 は「AP 停止中の Balancing (非 CONNECTING)」の観測を目的とするため、
  // 2 回の接続失敗後に guard 自身の retry begin() が settle 期間中に再発行
  // されてしまわないよう、backoff を意図的に長く取る (T1 以外は再接続速度
  // 重視で短い backoff=3 を使う)。
  p.reconnect_backoff_ticks = (scenario == "t1") ? 1000 : 3;
  WifiGuard guard(ops, p);
  c.guard = &guard;

  if (scenario == "t1") {
    scenarioT1(c);
  } else if (scenario == "t2") {
    scenarioT2(c, /*immediate_abort_variant=*/false);
  } else if (scenario == "t2_immediate_abort") {
    scenarioT2(c, /*immediate_abort_variant=*/true);
  } else if (scenario == "t3") {
    scenarioT3Only(c);
  } else if (scenario == "t3b") {
    scenarioT3AndT3b(c);
  } else if (scenario == "t5") {
    scenarioT5(c);
  } else if (scenario == "common") {
    scenarioCommon(c);
  } else if (scenario == "t3_race_predrain_forbidden_op") {
    scenarioT3RaceForbiddenOp(c);
  } else {
    std::fprintf(stderr, "unknown scenario: %s\n", scenario.c_str());
    return 2;
  }

  std::fwrite(c.log.data(), 1, c.log.size(), stdout);
  return 0;
}
