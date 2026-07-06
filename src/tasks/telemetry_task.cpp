// telemetry_task.cpp — UDP telemetry タスク実装 (計画書 §3.1)。
// __has_include("wifi_secrets.h") の分岐はこのファイル内部に閉じ込める
// (telemetry_task.h は secrets の有無によらず安定 API を提供する)。
#include "telemetry_task.h"

#if __has_include("wifi_secrets.h")
#define BSL_TELEMETRY_SECRETS_AVAILABLE 1
#include "wifi_secrets.h"
#else
#define BSL_TELEMETRY_SECRETS_AVAILABLE 0
#endif

#ifndef BSL_TELEMETRY_UDP_PORT
#define BSL_TELEMETRY_UDP_PORT 45678
#endif
#ifndef BSL_FW_GIT
#define BSL_FW_GIT "unknown"  // platformio.ini extra_scripts が通常は注入する (§3.1)
#endif
// Wi-Fi ガード計装 (trace) ビルドフラグ (wifi-guard-trace 計画書 §2 設計原則1)。
// 未定義 (通常の m5stack-core2 env) では常に 0 = 本 PR 適用前とコード的に不変。
#ifndef BSL_WIFI_GUARD_TRACE
#define BSL_WIFI_GUARD_TRACE 0
#endif

#if BSL_TELEMETRY_SECRETS_AVAILABLE

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <cstdio>

#include "../app_config.h"
#include "../core/dt_histogram.h"
#include "../core/safety_fsm.h"
#include "../core/telemetry_format.h"
#include "../core/wifi_guard.h"

#if BSL_WIFI_GUARD_TRACE
// Wi-Fi ガード計装 (trace) ビルド専用 (wifi-guard-trace 計画書 §3/D1)。
// BSL_TELEMETRY_SECRETS_AVAILABLE 有効領域の**内側**にのみ現れるため、
// trace ∧ ¬secrets ≡ ¬secrets (no-op) が構造的に保証される (D1/D6)。
#include "../core/trace_emitter.h"
#endif

#endif  // BSL_TELEMETRY_SECRETS_AVAILABLE

namespace tasks {

#if BSL_TELEMETRY_SECRETS_AVAILABLE

namespace {

WiFiUDP g_udp;
IPAddress g_host_ip;
core::WifiGuard* g_guard = nullptr;
shared::SharedState* g_shared = nullptr;
char g_dev_id[16] = "core2-????";

#if BSL_WIFI_GUARD_TRACE
// ---- Wi-Fi ガード計装 (trace) 状態 (wifi-guard-trace 計画書 §4 D1) ----
// telemetry task 単一スレッドで読み書き (trace リングのみ SPSC)。すべて
// static 領域 (グローバル/名前空間スコープの静的ストレージ) に確保するため、
// TraceEmitter 内蔵の 4096+64B 行バッファがタスクスタック (8192B) を圧迫
// することはない (計画書 D3 スタック予算)。
core::trace::TraceEmitter g_trace_emitter;
core::trace::TraceEventRing g_trace_ring;
// guard 対象イベント (gotip/disc/stop) にのみ振る到着通し番号 (計画書 D3 ev.i)。
// コールバックは guard pushEvent → trace ring の順に単一タスク文脈 (Wi-Fi
// イベントタスク) で push するため、この番号は guard キュー内の順序と一致する。
uint32_t g_trace_guard_event_seq = 0;
#endif

// ---- WifiOps 実装: 実際の WiFi/WiFiUDP API 呼び出し (計画書 §3.1) ----
void OpsBegin(void*) {
  // ライブラリ既定動作の無効化 (計画書 §3.1): flash-backed config 更新と定常
  // 自動再接続を無効化する。ただし WiFiGeneric.cpp の first_connect one-shot
  // (非自発的切断の初回で無条件 disconnect();begin();) はこれとは独立に発火し得る
  // ため、WifiGuard 側のイベント駆動ガードで有界キャンセルする。
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.begin(BSL_WIFI_SSID, BSL_WIFI_PASS);
}
bool OpsDisconnect(void*) { return WiFi.disconnect(); }
void OpsSetRadioOff(void*) { WiFi.mode(WIFI_OFF); }
bool OpsIsRadioOff(void*) { return WiFi.status() == WL_NO_SHIELD; }
int OpsBeginPacket(void*) { return g_udp.beginPacket(g_host_ip, BSL_TELEMETRY_UDP_PORT); }
int OpsWritePacket(void*, const uint8_t* buf, size_t len) {
  return static_cast<int>(g_udp.write(buf, len));
}
int OpsEndPacket(void*) { return g_udp.endPacket(); }

#if BSL_WIFI_GUARD_TRACE
// ---- Trace 版 WifiOps ラッパ (計画書 D4): 実 Ops への委譲 + 行バッファへの
// 記録のみ。呼び出し順序・回数・引数は実 Ops (OpsBegin 等) に対して完全に不変。
// Serial は一切呼ばない (D3 印字タイミング: フラッシュは tick 末尾 1 箇所のみ)。
void TraceOpsBegin(void* ctx) {
  const uint64_t t0 = static_cast<uint64_t>(esp_timer_get_time());
  OpsBegin(ctx);
  const uint64_t dur = static_cast<uint64_t>(esp_timer_get_time()) - t0;
  g_trace_emitter.recordOp(core::trace::OpKind::Begin, t0, dur, /*res_valid=*/false,
                           0, g_guard->lastWifiQuiet(), g_guard->udpReady());
}
bool TraceOpsDisconnect(void* ctx) {
  const uint64_t t0 = static_cast<uint64_t>(esp_timer_get_time());
  const bool res = OpsDisconnect(ctx);
  const uint64_t dur = static_cast<uint64_t>(esp_timer_get_time()) - t0;
  g_trace_emitter.recordOp(core::trace::OpKind::Disc, t0, dur, /*res_valid=*/true,
                           res ? 1 : 0, g_guard->lastWifiQuiet(), g_guard->udpReady());
  return res;
}
void TraceOpsSetRadioOff(void* ctx) {
  const uint64_t t0 = static_cast<uint64_t>(esp_timer_get_time());
  OpsSetRadioOff(ctx);
  const uint64_t dur = static_cast<uint64_t>(esp_timer_get_time()) - t0;
  g_trace_emitter.recordOp(core::trace::OpKind::RadioOff, t0, dur,
                           /*res_valid=*/false, 0, g_guard->lastWifiQuiet(),
                           g_guard->udpReady());
}
int TraceOpsBeginPacket(void* ctx) {
  // op=bp: beginPacket 実行の直前/戻り直後で SharedState を即時再サンプルする
  // (計画書 D3 R4 sampled 契約: 観測のみでガード判断には一切不使用。malloc/
  // socket 確保は dur 区間内で起きるため pre のみでは確保区間を挟めない)。
  core::trace::OpResample resample;
  shared::Snapshot pre;
  resample.read_ok2 = g_shared->read(&pre);
  if (resample.read_ok2) {
    resample.fsm2 = pre.fsm_state;
    resample.arm2 = pre.arm_pending;
  }
  const uint64_t t0 = static_cast<uint64_t>(esp_timer_get_time());
  const int res = OpsBeginPacket(ctx);
  const uint64_t dur = static_cast<uint64_t>(esp_timer_get_time()) - t0;
  shared::Snapshot post;
  resample.read_ok3 = g_shared->read(&post);
  if (resample.read_ok3) {
    resample.fsm3 = post.fsm_state;
    resample.arm3 = post.arm_pending;
  }
  g_trace_emitter.recordOp(core::trace::OpKind::Bp, t0, dur, /*res_valid=*/true, res,
                           g_guard->lastWifiQuiet(), g_guard->udpReady(), resample);
  return res;
}
#endif  // BSL_WIFI_GUARD_TRACE

// Wi-Fi イベントコールバック: atomic なイベントキューへの push のみ (計画書 §3.1。
// Wi-Fi API はここでは一切呼ばない)。
void onWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (!g_guard) return;
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      g_guard->pushEvent(core::WifiGuard::EventKind::GotIp);
#if BSL_WIFI_GUARD_TRACE
      g_trace_ring.push({static_cast<uint64_t>(esp_timer_get_time()),
                         core::trace::EvKind::GotIp, 0,
                         static_cast<int32_t>(++g_trace_guard_event_seq)});
#endif
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      g_guard->pushEvent(core::WifiGuard::EventKind::StaDisconnected,
                         info.wifi_sta_disconnected.reason);
#if BSL_WIFI_GUARD_TRACE
      g_trace_ring.push({static_cast<uint64_t>(esp_timer_get_time()),
                         core::trace::EvKind::Disc,
                         info.wifi_sta_disconnected.reason,
                         static_cast<int32_t>(++g_trace_guard_event_seq)});
#endif
      break;
    case ARDUINO_EVENT_WIFI_STA_STOP:
      g_guard->pushEvent(core::WifiGuard::EventKind::StaStop);
#if BSL_WIFI_GUARD_TRACE
      g_trace_ring.push({static_cast<uint64_t>(esp_timer_get_time()),
                         core::trace::EvKind::Stop, 0,
                         static_cast<int32_t>(++g_trace_guard_event_seq)});
#endif
      break;
#if BSL_WIFI_GUARD_TRACE
    // trace 専用の追加購読 (計画書 D3 ev 行): guard への pushEvent 対象は上記
    // 3 種から変更しない。ライブラリ内部の再接続活動 (telemetry の Ops を
    // 経由しない disconnect();begin();) を op 行の不在に頼らずイベント面で
    // 可視化するための観測専用購読 (i=- で guard キュー順序とは無関係)。
    case ARDUINO_EVENT_WIFI_STA_START:
      g_trace_ring.push({static_cast<uint64_t>(esp_timer_get_time()),
                         core::trace::EvKind::Start, 0, -1});
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      g_trace_ring.push({static_cast<uint64_t>(esp_timer_get_time()),
                         core::trace::EvKind::Conn, 0, -1});
      break;
    case ARDUINO_EVENT_WIFI_SCAN_DONE:
      g_trace_ring.push({static_cast<uint64_t>(esp_timer_get_time()),
                         core::trace::EvKind::Scan, 0, -1});
      break;
#endif
    default:
      break;
  }
}

#if BSL_WIFI_GUARD_TRACE
// tick 末尾 (guard tick()/trySend() 完了後) の st 差分/hb/drop 出力 + 一括
// フラッシュ (計画書 D11 シーケンシング・D3 印字タイミング)。guard 処理・
// 送信完了後の 1 箇所のみで呼ぶため、被測定経路 (abort 発行等) を遅延させない。
void flushTrace(uint32_t tick, bool read_ok, uint8_t fsm, bool arm_pending) {
  core::trace::StateTuple stt;
  stt.read_ok = read_ok;
  stt.fsm = fsm;
  stt.arm = arm_pending;
  stt.q = g_guard->lastWifiQuiet();
  stt.conn = g_guard->connected();
  stt.cing = g_guard->connecting();
  stt.udpr = g_guard->udpReady();
  stt.librp = g_guard->libReconnectPending();
  stt.ab = g_guard->aborting();
  stt.roff = g_guard->radioOffPending();
  stt.latch = g_guard->wifiAbortFailed();
  stt.pwf = g_guard->prewarmFailTotal();
  stt.sf = g_guard->sendFailTotal();
  stt.evo = g_guard->eventOverflowTotal();

  const uint64_t t_us = static_cast<uint64_t>(esp_timer_get_time());
  g_trace_emitter.endTick(t_us, stt);

  // stkmin: uxTaskGetStackHighWaterMark() は ESP32 port (StackType_t=uint8_t)
  // では既にバイト単位だが、移植性のため sizeof(StackType_t) を明示乗算して
  // 「換算バイト」にする (計画書 D3)。ESP.getFreeHeap()/getMinFreeHeap() は
  // esp_get_free_heap_size()/esp_get_minimum_free_heap_size() の Arduino
  // ラッパ (計画書 D2 補助証拠。参考情報であり合否条件にはしない)。
  const uint32_t stkmin_bytes = static_cast<uint32_t>(
      uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t));
  g_trace_emitter.maybeHeartbeat(t_us, tick, stt,
                                 static_cast<uint32_t>(ESP.getFreeHeap()),
                                 static_cast<uint32_t>(ESP.getMinFreeHeap()),
                                 stkmin_bytes);
  g_trace_emitter.maybeEmitDrop(t_us);

  const uint64_t f0 = static_cast<uint64_t>(esp_timer_get_time());
  if (g_trace_emitter.bufferedBytes() > 0) {
    Serial.write(reinterpret_cast<const uint8_t*>(g_trace_emitter.bufferData()),
                g_trace_emitter.bufferedBytes());
  }
  const uint64_t fdur = static_cast<uint64_t>(esp_timer_get_time()) - f0;
  g_trace_emitter.recordFlushDuration(fdur);
  g_trace_emitter.resetBuffer();
}
#endif  // BSL_WIFI_GUARD_TRACE

void telemetryTaskEntry(void*) {
  uint32_t seq = 0;
  uint32_t tick = 0;
  uint32_t read_fail_total = 0;
  uint32_t trunc_total = 0;
  char buf[core::kTelemetryBufferBytes];

#if BSL_WIFI_GUARD_TRACE
  // trace ビルド専用: フラッシュ予算保証のため 921600 baud へ切替 (計画書 D3/D9。
  // main.cpp の Serial.begin(115200) は不変)。boot 行は一切の guard tick /
  // Wi-Fi 操作より前に単独でフラッシュする (計画書 D3: 先頭欠落の機械的排除)。
  Serial.updateBaudRate(921600);
  g_trace_emitter.emitBoot(static_cast<uint64_t>(esp_timer_get_time()), BSL_FW_GIT,
                           g_dev_id);
  {
    const uint64_t f0 = static_cast<uint64_t>(esp_timer_get_time());
    Serial.write(reinterpret_cast<const uint8_t*>(g_trace_emitter.bufferData()),
                g_trace_emitter.bufferedBytes());
    const uint64_t fdur = static_cast<uint64_t>(esp_timer_get_time()) - f0;
    g_trace_emitter.recordFlushDuration(fdur);
    g_trace_emitter.resetBuffer();
  }
#endif

  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(cfg::kTelemetryPeriodMs));
    ++tick;  // tick は read 失敗でも進む (計画書 §3.1)

    shared::Snapshot snap;
    const bool read_ok = g_shared->read(&snap);
    if (!read_ok) ++read_fail_total;  // 実装上は常に true (§3.1 防御的カウンタ)

    const bool balancing =
        read_ok && snap.fsm_state == static_cast<uint8_t>(core::FsmState::Balancing);
    const bool arm_pending = read_ok && snap.arm_pending;

#if BSL_WIFI_GUARD_TRACE
    // tick 冒頭 (guard.tick() 呼び出し前) の tk 行 (計画書 D11 の tick 内
    // シーケンシング: tk (ep=drain前) → trace リング drain (ev 行化) →
    // guard tick/trySend (op 記録) → st 差分 → hb/drop → フラッシュ)。
    // ep は drainedEpoch() を guard.tick() 呼び出し直前に読むことで pre-drain
    // 契約を満たす (同一スレッド上でこの間に割り込みは入らない)。
    const uint64_t trace_tick_t_us = static_cast<uint64_t>(esp_timer_get_time());
    const uint8_t trace_fsm = read_ok ? snap.fsm_state : 0;
    const uint32_t trace_ep = g_guard->drainedEpoch();
    g_trace_emitter.beginTick(trace_tick_t_us, read_ok, trace_fsm, arm_pending,
                              trace_ep);
    g_trace_emitter.drainRing(g_trace_ring);
#endif

    g_guard->tick(read_ok, read_ok ? snap.loop_count : 0, balancing, arm_pending);

    if (!g_guard->readyToAttempt()) {
#if BSL_WIFI_GUARD_TRACE
      flushTrace(tick, read_ok, trace_fsm, arm_pending);
#endif
      continue;
    }

    // seq は「送信 datagram の通し番号」(計画書 §3.1) であり、trySend が実際に
    // datagram を送出できた場合のみ消費されなければならない。ここではまだ
    // "候補" (seq_candidate = seq + 1) として packet を整形するだけに留め、
    // 確定 (seq への反映) は下記 trySend の戻り値が Sent の場合のみ行う
    // (ゲート2レビュー(5回目)指摘1対応)。trySend が失敗 (beginPacket/write/
    // endPacket のいずれかが失敗) した場合は datagram が出ていないため、次回
    // tick で同じ候補値が再利用されても重複にはならない。
    const uint32_t seq_candidate = seq + 1;
    size_t len = 0;
    if (!read_ok) {
      core::TelemetryDiagFields df;
      df.seq = seq_candidate;
      df.tick = tick;
      df.t_us = esp_timer_get_time();
      df.dev = g_dev_id;
      df.fw = BSL_FW_GIT;
      df.reason = core::DiagReason::ReadFail;
      df.read_fail = read_fail_total;
      df.trunc = trunc_total;
      len = core::formatDiagPacket(buf, sizeof(buf), df);
    } else {
      core::TelemetryFullFields ff;
      ff.seq = seq_candidate;
      ff.tick = tick;
      ff.t_us = esp_timer_get_time();
      ff.dev = g_dev_id;
      ff.fw = BSL_FW_GIT;
      ff.fsm = snap.fsm_state;
      ff.fault = snap.fault_reason;
      ff.theta = snap.theta;
      ff.theta_dot = snap.theta_rate;
      ff.theta_ref = snap.theta_ref;
      ff.v_body = snap.v;
      ff.om_l = snap.omega_left;
      ff.om_r = snap.omega_right;
      ff.i_cmd_l = snap.i_cmd_left;
      ff.i_cmd_r = snap.i_cmd_right;
      ff.i_mea_l = snap.i_present_left;
      ff.i_mea_r = snap.i_present_right;
      ff.dt_us = static_cast<uint32_t>(snap.dt_last * 1.0e6f);
      ff.dt_max_us = static_cast<uint32_t>(snap.dt_max * 1.0e6f);
      ff.loop = snap.loop_count;
      for (int i = 0; i < core::kDtHistogramBins; ++i) ff.dt_h[i] = snap.dt_hist_total[i];
      ff.ovr = snap.overrun_total;
      ff.stale = snap.imu_stale_total;
      ff.read_fail = read_fail_total;
      ff.trunc = trunc_total;
      ff.volt = snap.voltage;
      ff.temp = snap.temperature;
      ff.sat = snap.saturated;
      ff.i2t = snap.i2t_limited;
      ff.kp = snap.params.kp;
      ff.ki = snap.params.ki;
      ff.kd = snap.params.kd;
      ff.eq = snap.params.pitch_eq;
      ff.rssi = static_cast<int16_t>(WiFi.RSSI());
      len = core::formatFullPacket(buf, sizeof(buf), ff);
      if (len == 0) {
        // truncation: 送信せず診断 datagram で報告する (計画書 §3.1。不正 JSON を送らない)
        ++trunc_total;
        core::TelemetryDiagFields df;
        df.seq = seq_candidate;
        df.tick = tick;
        df.t_us = esp_timer_get_time();
        df.dev = g_dev_id;
        df.fw = BSL_FW_GIT;
        df.reason = core::DiagReason::Trunc;
        df.read_fail = read_fail_total;
        df.trunc = trunc_total;
        len = core::formatDiagPacket(buf, sizeof(buf), df);
      }
    }

    if (len > 0) {
      const core::WifiGuard::SendOutcome outcome =
          g_guard->trySend(reinterpret_cast<const uint8_t*>(buf), len);
      // seq の確定は datagram が実際に送出できた場合のみ (計画書 §3.1)。
      // 失敗時は seq を据え置き、次 tick で同じ candidate 値を再試行する
      // (datagram が出ていないため重複にはならない)。
      if (outcome == core::WifiGuard::SendOutcome::Sent) {
        seq = seq_candidate;
      }
    }

#if BSL_WIFI_GUARD_TRACE
    flushTrace(tick, read_ok, trace_fsm, arm_pending);
#endif
  }
}

}  // namespace

void startTelemetryTask(shared::SharedState& shared_state) {
  g_shared = &shared_state;

  const uint64_t mac = ESP.getEfuseMac();
  std::snprintf(g_dev_id, sizeof(g_dev_id), "core2-%04x",
               static_cast<unsigned>(mac & 0xFFFFu));
  g_host_ip.fromString(BSL_TELEMETRY_HOST_IP);

  static core::WifiOps ops = [] {
    core::WifiOps o;
    o.ctx = nullptr;
    o.begin = &OpsBegin;
    o.disconnect = &OpsDisconnect;
    o.setRadioOff = &OpsSetRadioOff;
    o.isRadioOff = &OpsIsRadioOff;
    o.beginPacket = &OpsBeginPacket;
    o.writePacket = &OpsWritePacket;
    o.endPacket = &OpsEndPacket;
#if BSL_WIFI_GUARD_TRACE
    // trace 版に差し替え (計画書 D4)。実 API の呼び出し順序・回数・引数は
    // 完全に不変 (各 TraceOps* が実 Ops へ委譲するのみ)。write/endPacket は
    // 確保を伴わないため素の Ops のまま (計画書 D2)。
    o.begin = &TraceOpsBegin;
    o.disconnect = &TraceOpsDisconnect;
    o.setRadioOff = &TraceOpsSetRadioOff;
    o.beginPacket = &TraceOpsBeginPacket;
#endif
    return o;
  }();
  static core::WifiGuard::Params params = [] {
    core::WifiGuard::Params p;
    p.abort_confirm_ticks = cfg::kWifiAbortConfirmTicks;
    p.abort_max_retries = cfg::kWifiAbortMaxRetries;
    p.lib_reconnect_timeout_ticks = cfg::kWifiLibReconnectTimeoutTicks;
    p.reconnect_backoff_ticks = cfg::kWifiReconnectBackoffTicks;
    return p;
  }();
  static core::WifiGuard guard(ops, params);
  g_guard = &guard;

  WiFi.onEvent(onWifiEvent);

  xTaskCreatePinnedToCore(telemetryTaskEntry, "TelemetryTask", 8192, nullptr, 1,
                          nullptr, 1);
}

bool telemetryEnabled() { return true; }

#else  // !BSL_TELEMETRY_SECRETS_AVAILABLE

// secrets 不在時は no-op (計画書 §3.1: main.cpp は無条件に startTelemetryTask() を
// 呼ぶため、呼び出し側に #ifdef を置かない)。
void startTelemetryTask(shared::SharedState&) {}
bool telemetryEnabled() { return false; }

#endif  // BSL_TELEMETRY_SECRETS_AVAILABLE

}  // namespace tasks
