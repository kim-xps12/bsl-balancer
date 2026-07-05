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

#endif  // BSL_TELEMETRY_SECRETS_AVAILABLE

namespace tasks {

#if BSL_TELEMETRY_SECRETS_AVAILABLE

namespace {

WiFiUDP g_udp;
IPAddress g_host_ip;
core::WifiGuard* g_guard = nullptr;
shared::SharedState* g_shared = nullptr;
char g_dev_id[16] = "core2-????";

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

// Wi-Fi イベントコールバック: atomic なイベントキューへの push のみ (計画書 §3.1。
// Wi-Fi API はここでは一切呼ばない)。
void onWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (!g_guard) return;
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      g_guard->pushEvent(core::WifiGuard::EventKind::GotIp);
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      g_guard->pushEvent(core::WifiGuard::EventKind::StaDisconnected,
                         info.wifi_sta_disconnected.reason);
      break;
    case ARDUINO_EVENT_WIFI_STA_STOP:
      g_guard->pushEvent(core::WifiGuard::EventKind::StaStop);
      break;
    default:
      break;
  }
}

void telemetryTaskEntry(void*) {
  uint32_t seq = 0;
  uint32_t tick = 0;
  uint32_t read_fail_total = 0;
  uint32_t trunc_total = 0;
  char buf[core::kTelemetryBufferBytes];

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

    g_guard->tick(read_ok, read_ok ? snap.loop_count : 0, balancing, arm_pending);

    if (!g_guard->readyToAttempt()) continue;

    ++seq;  // seq = 送信 datagram の通し番号 (計画書 §3.1)
    size_t len = 0;
    if (!read_ok) {
      core::TelemetryDiagFields df;
      df.seq = seq;
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
      ff.seq = seq;
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
        df.seq = seq;
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
      g_guard->trySend(reinterpret_cast<const uint8_t*>(buf), len);
    }
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
