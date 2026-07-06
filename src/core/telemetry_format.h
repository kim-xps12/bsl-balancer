// telemetry_format.h — UDP telemetry packet schema v1 の JSON 生成
// (UDP telemetry Phase1 計画書 §3.1)。Arduino 非依存・host テスト可能。
// snprintf で固定長バッファに 1 行 JSON を生成する。heap 確保・String は使わない。
// 全フィールドの float 精度・整数幅を明示指定し (既定 %f 精度は冗長すぎるため)、
// 呼び出し側は kTelemetryBufferBytes (1024B) のバッファを用意すること。
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace core {

// snprintf の戻り値検査で truncation を検出した場合や buf_size 不足の場合は
// formatFullPacket()/formatDiagPacket() は 0 を返す (呼び出し側は送信してはならない)。
constexpr size_t kTelemetryBufferBytes = 1024;

struct TelemetryFullFields {
  uint32_t seq = 0;    // 送信 datagram の通し番号 (telemetry task が付与)
  uint32_t tick = 0;   // telemetry task の周期実行回数 (attempt count)
  int64_t t_us = 0;    // esp_timer_get_time()
  const char* dev = "";  // eFuse MAC 由来のデバイス ID (例 "core2-a4cf")
  const char* fw = "";   // ビルド時 git 短縮ハッシュ
  uint8_t fsm = 0;     // core::FsmState
  uint8_t fault = 0;   // core::FaultReason
  float theta = 0.0f;
  float theta_dot = 0.0f;
  float theta_ref = 0.0f;
  float v_body = 0.0f;
  float om_l = 0.0f;
  float om_r = 0.0f;
  float i_cmd_l = 0.0f;
  float i_cmd_r = 0.0f;
  float i_mea_l = 0.0f;
  float i_mea_r = 0.0f;
  uint32_t dt_us = 0;
  uint32_t dt_max_us = 0;
  uint32_t loop = 0;
  uint32_t dt_h[8] = {};  // 累積 dt ヒストグラム (Snapshot::dt_hist_total)
  uint32_t ovr = 0;       // Snapshot::overrun_total
  uint32_t stale = 0;     // Snapshot::imu_stale_total
  uint32_t read_fail = 0; // telemetry task ローカルの累積 SharedState::read() 失敗数
  uint32_t trunc = 0;     // telemetry task ローカルの累積 truncation 回数
  float volt = 0.0f;
  float temp = 0.0f;
  bool sat = false;
  bool i2t = false;
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float eq = 0.0f;
  int16_t rssi = 0;
};

enum class DiagReason : uint8_t { ReadFail, Trunc };

struct TelemetryDiagFields {
  uint32_t seq = 0;
  uint32_t tick = 0;
  int64_t t_us = 0;
  const char* dev = "";
  const char* fw = "";
  DiagReason reason = DiagReason::ReadFail;
  uint32_t read_fail = 0;
  uint32_t trunc = 0;
};

inline const char* diagReasonStr(DiagReason r) {
  return r == DiagReason::ReadFail ? "read_fail" : "trunc";
}

// 成功時: 生成した JSON 行の長さ (終端 '\0' を含まない) を返す。
// truncation (buf_size 以上必要) またはエンコードエラー時は 0 を返す (送信禁止)。
inline size_t formatFullPacket(char* buf, size_t buf_size,
                               const TelemetryFullFields& f) {
  const int n = std::snprintf(
      buf, buf_size,
      "{\"v\":1,\"seq\":%u,\"tick\":%u,\"snap_valid\":true,\"t_us\":%lld,"
      "\"dev\":\"%s\",\"fw\":\"%s\",\"fsm\":%u,\"fault\":%u,"
      "\"theta\":%.4f,\"theta_dot\":%.4f,\"theta_ref\":%.4f,"
      "\"v_body\":%.4f,\"om_l\":%.4f,\"om_r\":%.4f,"
      "\"i_cmd_l\":%.4f,\"i_cmd_r\":%.4f,\"i_mea_l\":%.4f,\"i_mea_r\":%.4f,"
      "\"dt_us\":%u,\"dt_max_us\":%u,\"loop\":%u,"
      "\"dt_h\":[%u,%u,%u,%u,%u,%u,%u,%u],"
      "\"ovr\":%u,\"stale\":%u,\"read_fail\":%u,\"trunc\":%u,"
      "\"volt\":%.3f,\"temp\":%.2f,\"sat\":%s,\"i2t\":%s,"
      "\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f,\"eq\":%.4f,\"rssi\":%d}",
      static_cast<unsigned>(f.seq), static_cast<unsigned>(f.tick),
      static_cast<long long>(f.t_us), f.dev, f.fw,
      static_cast<unsigned>(f.fsm), static_cast<unsigned>(f.fault), f.theta,
      f.theta_dot, f.theta_ref, f.v_body, f.om_l, f.om_r, f.i_cmd_l,
      f.i_cmd_r, f.i_mea_l, f.i_mea_r, static_cast<unsigned>(f.dt_us),
      static_cast<unsigned>(f.dt_max_us), static_cast<unsigned>(f.loop),
      static_cast<unsigned>(f.dt_h[0]), static_cast<unsigned>(f.dt_h[1]),
      static_cast<unsigned>(f.dt_h[2]), static_cast<unsigned>(f.dt_h[3]),
      static_cast<unsigned>(f.dt_h[4]), static_cast<unsigned>(f.dt_h[5]),
      static_cast<unsigned>(f.dt_h[6]), static_cast<unsigned>(f.dt_h[7]),
      static_cast<unsigned>(f.ovr), static_cast<unsigned>(f.stale),
      static_cast<unsigned>(f.read_fail), static_cast<unsigned>(f.trunc),
      f.volt, f.temp, f.sat ? "true" : "false", f.i2t ? "true" : "false",
      f.kp, f.ki, f.kd, f.eq, static_cast<int>(f.rssi));
  if (n < 0 || static_cast<size_t>(n) >= buf_size) return 0;
  return static_cast<size_t>(n);
}

inline size_t formatDiagPacket(char* buf, size_t buf_size,
                               const TelemetryDiagFields& f) {
  const int n = std::snprintf(
      buf, buf_size,
      "{\"v\":1,\"seq\":%u,\"tick\":%u,\"t_us\":%lld,"
      "\"dev\":\"%s\",\"fw\":\"%s\",\"snap_valid\":false,\"reason\":\"%s\","
      "\"read_fail\":%u,\"trunc\":%u}",
      static_cast<unsigned>(f.seq), static_cast<unsigned>(f.tick),
      static_cast<long long>(f.t_us), f.dev, f.fw, diagReasonStr(f.reason),
      static_cast<unsigned>(f.read_fail), static_cast<unsigned>(f.trunc));
  if (n < 0 || static_cast<size_t>(n) >= buf_size) return 0;
  return static_cast<size_t>(n);
}

}  // namespace core
