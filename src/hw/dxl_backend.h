// dxl_backend.h — DYNAMIXEL バックエンド (設計書 §4)
// - 初期化シーケンス §4.1 (プロファイル値の読み戻し厳密検証)
// - verified_write: RX ドレーン + Status の ID/エラーバイト検証 (§4.2)
// - Bus Watchdog: 生アドレス 1 バイトアクセス必須 (ライブラリテーブルは 2B 誤定義)
// - バス検疫: 最優先送信ゲート (検疫 > save_in_progress > 心拍 > その他)
#pragma once

#include <Dynamixel2Arduino.h>
#include <cstdint>

#include "../app_config.h"

namespace hw {

// Master の protected 生プロトコル API (txInstPacket/rxStatusPacket) を用いて、
// Status 応答の「送信元 ID 一致 ∧ エラーバイト==0」を検証する read/write を提供する
// 薄いラッパ (§4.2 verified_write の下回り。ライブラリの bool は ALERT でも true を
// 返すため信用しない)。
class DxlWithRxInfo : public Dynamixel2Arduino {
 public:
  using Dynamixel2Arduino::Dynamixel2Arduino;

  // 検証付き WRITE。成功 = Status 受信 ∧ rx.id==id ∧ rx.err_idx==0
  bool writeVerified(uint8_t id, uint16_t addr, const uint8_t* data,
                     uint16_t len, uint32_t timeout_ms);
  // 検証付き READ。成功時 buf に len バイト格納。ID/エラーバイト/長さを検証
  bool readVerified(uint8_t id, uint16_t addr, uint16_t len, uint8_t* buf,
                    uint32_t timeout_ms);
};

struct WheelFeedback {
  bool valid = false;      // 今周期の読取成功 (stale 値は制御に使わない §4.2)
  float i_left = 0.0f;     // 正規化済み Present Current [A] (前進正)
  float i_right = 0.0f;
  float omega_left = 0.0f; // 正規化済み Present Velocity [rad/s] (前進正)
  float omega_right = 0.0f;
  float pos_left = 0.0f;   // [rad]
  float pos_right = 0.0f;
};

struct HealthInfo {
  float voltage = 0.0f;      // [V]
  float temperature = 0.0f;  // [degC]
  uint8_t hw_error = 0;      // Hardware Error Status(70) の OR
  bool watchdog_tripped = false;
};

enum class WatchdogCheck : uint8_t {
  Ok = 0,        // トリップなし
  Recovered,     // トルク OFF 確認済みで自動復旧した
  Fault,         // トルク ON 中のトリップ / 読取失敗 / 非対称 → ラッチ FAULT
};

class DxlBackend {
 public:
  explicit DxlBackend(HardwareSerial& serial) : dxl_(serial) {}

  // §4.1 初期化シーケンス。失敗時 false (トルク ON しない)。
  bool init(cfg::Profile profile);

  // ---- 送信ゲート (§4.2 優先順位: 検疫 > save > 心拍 > その他) ----
  void engageQuarantine();               // 検疫開始 (バス全体沈黙)
  bool quarantineActive() const;
  void setSaveGate(bool active) { save_gate_ = active; }

  // ---- 周期 I/O ----
  // 零電流心拍 (トルク OFF 状態用、syncWrite)。ゲート中は何もしない。
  bool writeZeroHeartbeat();
  // BALANCING 用: 検証付き個別 WRITE (§4.2 verified_write)。
  // 戻り値 false = 配達未検証 (呼び出し側が零書込→FAULT 系列を実施)
  bool writeGoalCurrentsVerified(float i_left_a, float i_right_a);
  // 検証付き零書込 (安全遷移用)。false = 未検証 → 呼び出し側は検疫へ
  bool writeZeroVerified();
  // 周期ブロック読取 (126..135, 10B)。符号正規化済み。
  WheelFeedback readFeedback();

  // ---- 安全シーケンス ----
  // enter_balancing() §4.1: raw98/HWエラー確認 → 零書込検証(トルクOFFのまま)
  // → Torque ON → 読み戻し → 零再確認。false = 検証失敗 (FSM は FAULT へ)。
  // トルクOFF中の潜在Watchdogトリップは §4.3 の復旧経路を先に実行する。
  bool enterBalancing(float now_s);
  // 零書込(検証)→Torque OFF(読み戻し)。false = 未検証 → 内部で検疫を発動済み
  bool safeStop();
  // §4.3 Watchdog 状態別遷移表 (torque_may_be_on: 呼び出し側の想定)
  WatchdogCheck checkWatchdog(bool torque_may_be_on, float now_s);
  // 検証済み safe-off か (保存ゲート用: 両輪 Torque Enable==0 && Goal==0)
  bool verifySafeOff();

  // ---- 低頻度ヘルス (ラウンドロビン 1 項目/呼び出し) ----
  void pollHealth(HealthInfo* out);

  uint32_t txFailCount() const { return tx_fail_count_; }

 private:
  bool verifiedWrite(uint8_t id, uint16_t addr, const uint8_t* data, uint16_t len);
  bool readRaw(uint8_t id, uint16_t addr, uint16_t len, uint8_t* buf);
  bool writeRaw1(uint8_t id, uint16_t addr, uint8_t value);
  bool verifyByte(uint8_t id, uint16_t addr, uint8_t expect);
  void drainRx();
  bool txAllowed() const;
  bool watchdogRecoverOne(uint8_t id);

  DxlWithRxInfo dxl_;
  cfg::Profile profile_ = cfg::Profile::Bringup;
  volatile int64_t quarantine_until_us_ = 0;
  bool save_gate_ = false;
  uint8_t health_phase_ = 0;
  HealthInfo health_{};
  uint32_t tx_fail_count_ = 0;
  // Watchdog 自動復旧の頻度制限 (§4.3: 60s 内 3 回で FAULT)
  int recover_count_ = 0;
  float recover_window_start_s_ = 0.0f;
};

}  // namespace hw
