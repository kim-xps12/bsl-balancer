#include "dxl_backend.h"

#include <Arduino.h>
#include <cmath>

#include "../core/units.h"

namespace hw {
namespace {

// XL330 制御テーブル (XL330規範 §6.1)
constexpr uint16_t kAddrModelNumber = 0;
constexpr uint16_t kAddrReturnDelay = 9;
constexpr uint16_t kAddrOperatingMode = 11;
constexpr uint16_t kAddrCurrentLimit = 38;
constexpr uint16_t kAddrShutdown = 63;
constexpr uint16_t kAddrTorqueEnable = 64;
constexpr uint16_t kAddrStatusReturnLevel = 68;
constexpr uint16_t kAddrHwErrorStatus = 70;
constexpr uint16_t kAddrBusWatchdog = 98;  // ★実レジスタ 1B。ライブラリ共通
                                           // テーブルは {98,2} 誤定義のため
                                           // 生アドレス 1B アクセス必須 (§4.3)
constexpr uint16_t kAddrGoalCurrent = 102;
constexpr uint16_t kAddrPresentBlock = 126;  // Current(2)+Velocity(4)+Position(4)
constexpr uint16_t kAddrPresentVoltage = 144;
constexpr uint16_t kAddrPresentTemp = 146;
constexpr uint16_t kAddrPwmLimit = 36;
constexpr uint16_t kPwmLimitDefault = 885;
constexpr uint8_t kWatchdogTripped = 0xFF;  // raw==0xFF (-1 の 1B 表現) = トリップ

constexpr uint8_t kIds[2] = {cfg::kDxlIdLeft, cfg::kDxlIdRight};

int16_t goalRawFor(float amps) { return units::currentAToRaw(amps); }

}  // namespace

// ---------------- DxlWithRxInfo (生プロトコル検証 I/O) ----------------

bool DxlWithRxInfo::writeVerified(uint8_t id, uint16_t addr, const uint8_t* data,
                                  uint16_t len, uint32_t timeout_ms) {
  uint8_t param[2 + 8];
  if (len > 8) return false;
  param[0] = static_cast<uint8_t>(addr & 0xFF);
  param[1] = static_cast<uint8_t>((addr >> 8) & 0xFF);
  for (uint16_t i = 0; i < len; ++i) param[2 + i] = data[i];
  if (!txInstPacket(id, DXL_INST_WRITE, param, static_cast<uint16_t>(2 + len))) {
    return false;
  }
  uint8_t rxbuf[32];
  const InfoToParseDXLPacket_t* rx = rxStatusPacket(rxbuf, sizeof(rxbuf), timeout_ms);
  if (rx == nullptr) return false;
  if (rx->id != id) return false;       // 他 ID/残留応答の誤消費を拒否 (§4.2)
  if (rx->err_idx != 0) return false;   // ALERT(0x80) 含む非零は安全側へ
  return true;
}

bool DxlWithRxInfo::readVerified(uint8_t id, uint16_t addr, uint16_t len,
                                 uint8_t* buf, uint32_t timeout_ms) {
  uint8_t param[4];
  param[0] = static_cast<uint8_t>(addr & 0xFF);
  param[1] = static_cast<uint8_t>((addr >> 8) & 0xFF);
  param[2] = static_cast<uint8_t>(len & 0xFF);
  param[3] = static_cast<uint8_t>((len >> 8) & 0xFF);
  if (!txInstPacket(id, DXL_INST_READ, param, 4)) return false;
  uint8_t rxbuf[32];
  const InfoToParseDXLPacket_t* rx = rxStatusPacket(rxbuf, sizeof(rxbuf), timeout_ms);
  if (rx == nullptr) return false;
  if (rx->id != id) return false;
  if (rx->recv_param_len != len) return false;
  // READ の ALERT はデータ有効 (HW エラー通知はヘルス側で扱う) — err の
  // ALERT ビット以外 (Instruction/CRC 等の Result Fail) は失敗扱い
  if ((rx->err_idx & 0x7F) != 0) return false;
  for (uint16_t i = 0; i < len; ++i) buf[i] = rx->p_param_buf[i];
  return true;
}

// ---------------- 低レベルヘルパ ----------------

void DxlBackend::drainRx() {
  // verified_write の前提 (1): stale バイトのドレーン (§4.2)
  DXLPortHandler* port = dxl_.getPort();
  if (!port) return;
  while (port->available() > 0) (void)port->read();
}

bool DxlBackend::txAllowed() const {
  if (quarantineActive()) return false;  // 検疫 (最優先)
  if (save_gate_) return false;          // save_in_progress
  return true;
}

bool DxlBackend::verifiedWrite(uint8_t id, uint16_t addr, const uint8_t* data,
                               uint16_t len) {
  // 成功条件 (§4.2): drain 済み ∧ Status の送信元 ID 一致 ∧ エラーバイト==0
  if (!txAllowed()) return false;
  drainRx();
  if (!dxl_.writeVerified(id, addr, data, len, cfg::kDxlIoTimeoutMs)) {
    ++tx_fail_count_;
    return false;
  }
  return true;
}

bool DxlBackend::readRaw(uint8_t id, uint16_t addr, uint16_t len, uint8_t* buf) {
  if (!txAllowed()) return false;
  drainRx();
  return dxl_.readVerified(id, addr, len, buf, cfg::kDxlIoTimeoutMs);
}

bool DxlBackend::writeRaw1(uint8_t id, uint16_t addr, uint8_t value) {
  return verifiedWrite(id, addr, &value, 1);
}

bool DxlBackend::verifyByte(uint8_t id, uint16_t addr, uint8_t expect) {
  uint8_t v = 0;
  if (!readRaw(id, addr, 1, &v)) return false;
  return v == expect;
}

// ---------------- 検疫 ----------------

void DxlBackend::engageQuarantine() {
  quarantine_until_us_ = esp_timer_get_time() +
                         static_cast<int64_t>(cfg::kQuarantineSilenceS * 1e6f);
}

bool DxlBackend::quarantineActive() const {
  return esp_timer_get_time() < quarantine_until_us_;
}

// ---------------- 初期化 (§4.1) ----------------

bool DxlBackend::init(cfg::Profile profile) {
  profile_ = profile;
  dxl_.begin(cfg::kDxlBaud);
  dxl_.setPortProtocolVersion(cfg::kDxlProtocol);

  const uint16_t current_limit_raw =
      static_cast<uint16_t>(units::currentAToRaw(cfg::currentLimitFor(profile)));

  for (uint8_t id : kIds) {
    // ping + Model Number == 1190 検証
    if (!dxl_.ping(id)) return false;
    uint8_t mn[2];
    if (!readRaw(id, kAddrModelNumber, 2, mn)) return false;
    if (units::le16(mn) != static_cast<int16_t>(cfg::kDxlModelNumber)) return false;

    // Torque OFF (EEPROM 書換のため)
    if (!writeRaw1(id, kAddrTorqueEnable, 0)) return false;

    // Operating Mode = 0 (Current Control)
    if (!writeRaw1(id, kAddrOperatingMode, 0)) return false;

    // Current Limit (デフォルト 1750=事実上無制限のため必ず設定)
    {
      const uint8_t d[2] = {static_cast<uint8_t>(current_limit_raw & 0xFF),
                            static_cast<uint8_t>(current_limit_raw >> 8)};
      if (!verifiedWrite(id, kAddrCurrentLimit, d, 2)) return false;
    }

    // Return Delay Time = 0 / Status Return Level = 2 (配達検証の成立前提)
    if (!writeRaw1(id, kAddrReturnDelay, 0)) return false;
    if (!writeRaw1(id, kAddrStatusReturnLevel, 2)) return false;

    // 読み戻し厳密検証 (プロファイル不一致 = FAULT §4.1)
    if (!verifyByte(id, kAddrOperatingMode, 0)) return false;
    if (!verifyByte(id, kAddrReturnDelay, 0)) return false;
    if (!verifyByte(id, kAddrStatusReturnLevel, 2)) return false;
    {
      uint8_t v[2];
      if (!readRaw(id, kAddrCurrentLimit, 2, v)) return false;
      if (units::le16(v) != static_cast<int16_t>(current_limit_raw)) return false;
      // PWM Limit(36) は全モード共通の出力上限 → 885(100%) を検証
      if (!readRaw(id, kAddrPwmLimit, 2, v)) return false;
      if (units::le16(v) != static_cast<int16_t>(kPwmLimitDefault)) return false;
      // Shutdown(63) は既定値を読取記録 (変更しない)
      uint8_t sd = 0;
      if (!readRaw(id, kAddrShutdown, 1, &sd)) return false;
    }

    // 前回稼働の Watchdog トリップ残留を先に解除する (トリップ中は Goal 値が
    // read-only になり零書込が失敗するため、零書込より前に行う)
    uint8_t wd = 0;
    if (!readRaw(id, kAddrBusWatchdog, 1, &wd)) return false;
    if (wd == kWatchdogTripped) {
      if (!writeRaw1(id, kAddrBusWatchdog, 0)) return false;
    }

    // ★Goal Current=0 (モード変更で Current Limit 値へ自動セットされるため必須)
    {
      const uint8_t z[2] = {0, 0};
      if (!verifiedWrite(id, kAddrGoalCurrent, z, 2)) return false;
    }

    // Bus Watchdog 有効化 (raw 1 = 20ms)
    if (!writeRaw1(id, kAddrBusWatchdog, cfg::kBusWatchdogRaw)) return false;
  }
  // Torque は OFF のまま (Torque ON は enter_balancing() のみ §4.1)
  return true;
}

// ---------------- 周期 I/O ----------------

bool DxlBackend::writeZeroHeartbeat() {
  if (!txAllowed()) return false;
  ParamForSyncWriteInst_t sw;
  sw.addr = kAddrGoalCurrent;
  sw.length = 2;
  sw.id_count = 2;
  for (int i = 0; i < 2; ++i) {
    sw.xel[i].id = kIds[i];
    sw.xel[i].data[0] = 0;
    sw.xel[i].data[1] = 0;
  }
  return dxl_.syncWrite(sw);
}

bool DxlBackend::writeGoalCurrentsVerified(float i_left_a, float i_right_a) {
  // 符号正規化: 指令にも s_L/s_R を適用 (§4.2)
  const int16_t l = goalRawFor(i_left_a * cfg::kSignLeft);
  const int16_t r = goalRawFor(i_right_a * cfg::kSignRight);
  const uint8_t dl[2] = {static_cast<uint8_t>(l & 0xFF), static_cast<uint8_t>((l >> 8) & 0xFF)};
  const uint8_t dr[2] = {static_cast<uint8_t>(r & 0xFF), static_cast<uint8_t>((r >> 8) & 0xFF)};
  bool ok = verifiedWrite(cfg::kDxlIdLeft, kAddrGoalCurrent, dl, 2);
  ok &= verifiedWrite(cfg::kDxlIdRight, kAddrGoalCurrent, dr, 2);
  return ok;
}

bool DxlBackend::writeZeroVerified() {
  const uint8_t z[2] = {0, 0};
  bool ok = verifiedWrite(cfg::kDxlIdLeft, kAddrGoalCurrent, z, 2);
  ok &= verifiedWrite(cfg::kDxlIdRight, kAddrGoalCurrent, z, 2);
  if (ok) {
    // 読み戻しで零を確認 (安全遷移では syncWrite を信用しない §4.2)
    uint8_t v[2];
    ok = readRaw(cfg::kDxlIdLeft, kAddrGoalCurrent, 2, v) && units::le16(v) == 0;
    ok = ok && readRaw(cfg::kDxlIdRight, kAddrGoalCurrent, 2, v) && units::le16(v) == 0;
  }
  return ok;
}

WheelFeedback DxlBackend::readFeedback() {
  WheelFeedback fb;
  if (!txAllowed()) return fb;

  ParamForSyncReadInst_t sr;
  sr.addr = kAddrPresentBlock;
  sr.length = 10;
  sr.id_count = 2;
  sr.xel[0].id = cfg::kDxlIdLeft;
  sr.xel[1].id = cfg::kDxlIdRight;
  RecvInfoFromStatusInst_t rx;
  drainRx();
  if (!dxl_.syncRead(sr, rx, cfg::kDxlIoTimeoutMs)) return fb;
  if (rx.id_count != 2) return fb;

  float i[2] = {0, 0}, w[2] = {0, 0}, pos[2] = {0, 0};
  bool seen[2] = {false, false};
  for (int k = 0; k < 2; ++k) {
    const auto& x = rx.xel[k];
    int idx = -1;
    if (x.id == cfg::kDxlIdLeft) idx = 0;
    if (x.id == cfg::kDxlIdRight) idx = 1;
    if (idx < 0 || x.length < 10) return fb;  // ID 不一致/短小応答は無効 (§4.2)
    i[idx] = units::rawToCurrentA(units::le16(&x.data[0]));
    w[idx] = units::rawToVelRadS(units::le32(&x.data[2]));
    pos[idx] = units::rawToPosRad(units::le32(&x.data[6]));
    seen[idx] = true;
  }
  if (!seen[0] || !seen[1]) return fb;

  // 帰還にも符号正規化を適用: 上位層は「前進 = 正」のみを見る (§4.2)
  fb.i_left = i[0] * cfg::kSignLeft;
  fb.i_right = i[1] * cfg::kSignRight;
  fb.omega_left = w[0] * cfg::kSignLeft;
  fb.omega_right = w[1] * cfg::kSignRight;
  fb.pos_left = pos[0] * cfg::kSignLeft;
  fb.pos_right = pos[1] * cfg::kSignRight;
  fb.valid = true;
  return fb;
}

// ---------------- 安全シーケンス ----------------

bool DxlBackend::enterBalancing() {
  // §4.1 enter_balancing(): 唯一の突入契約
  for (uint8_t id : kIds) {
    uint8_t hw_err = 0xFF;
    if (!readRaw(id, kAddrHwErrorStatus, 1, &hw_err) || hw_err != 0) return false;
    uint8_t wd = 0;
    if (!readRaw(id, kAddrBusWatchdog, 1, &wd)) return false;
    if (wd == kWatchdogTripped) return false;  // 潜在トリップのまま Torque ON 禁止
    if (wd != cfg::kBusWatchdogRaw) {
      // 復旧途中失敗等で無効(0)のまま残った場合は再有効化してから進む
      // (Watchdog なしで Torque ON しない — 最終防御の欠落を許さない)
      if (!writeRaw1(id, kAddrBusWatchdog, cfg::kBusWatchdogRaw)) return false;
      if (!verifyByte(id, kAddrBusWatchdog, cfg::kBusWatchdogRaw)) return false;
    }
  }
  // Torque Enable==0 のまま Goal Current=0 を検証付き書込 (非零パルス遮断)
  if (!writeZeroVerified()) return false;
  for (uint8_t id : kIds) {
    if (!writeRaw1(id, kAddrTorqueEnable, 1)) return false;
    if (!verifyByte(id, kAddrTorqueEnable, 1)) return false;
  }
  // 冗長チェック: Torque ON 後も Goal==0
  uint8_t v[2];
  if (!readRaw(cfg::kDxlIdLeft, kAddrGoalCurrent, 2, v) || units::le16(v) != 0) return false;
  if (!readRaw(cfg::kDxlIdRight, kAddrGoalCurrent, 2, v) || units::le16(v) != 0) return false;
  return true;
}

bool DxlBackend::safeStop() {
  // 零書込(検証) → Torque OFF(読み戻し)。未検証なら検疫 (§4.2 fail-stop)
  bool ok = writeZeroVerified();
  if (ok) {
    for (uint8_t id : kIds) {
      ok &= writeRaw1(id, kAddrTorqueEnable, 0);
      ok &= verifyByte(id, kAddrTorqueEnable, 0);
    }
  }
  if (!ok) engageQuarantine();
  return ok;
}

bool DxlBackend::watchdogRecoverOne(uint8_t id) {
  if (!writeRaw1(id, kAddrBusWatchdog, 0)) return false;  // エラー解除
  uint8_t v[2];
  if (!readRaw(id, kAddrGoalCurrent, 2, v) || units::le16(v) != 0) return false;
  return writeRaw1(id, kAddrBusWatchdog, cfg::kBusWatchdogRaw);  // 再有効化
}

WatchdogCheck DxlBackend::checkWatchdog(bool torque_may_be_on, float now_s) {
  // §4.3: 判定根拠は両輪の実測 (raw98 + Torque Enable 読み戻し)
  uint8_t wd[2];
  for (int k = 0; k < 2; ++k) {
    if (!readRaw(kIds[k], kAddrBusWatchdog, 1, &wd[k])) {
      // dt 起因検査で raw98 が読めない場合: トルク有効中なら fail-closed
      return torque_may_be_on ? WatchdogCheck::Fault : WatchdogCheck::Ok;
    }
  }
  const bool tripped = (wd[0] == kWatchdogTripped) || (wd[1] == kWatchdogTripped);
  if (!tripped) return WatchdogCheck::Ok;

  // Torque Enable 読み戻し (集約規則: 両輪成功かつ両輪 0 のみ自動復旧)
  uint8_t te[2];
  for (int k = 0; k < 2; ++k) {
    if (!readRaw(kIds[k], kAddrTorqueEnable, 1, &te[k])) return WatchdogCheck::Fault;
  }
  if (te[0] != 0 || te[1] != 0) return WatchdogCheck::Fault;

  // 復旧頻度制限 (60s 内 3 回で FAULT)
  if (recover_count_ == 0 ||
      (now_s - recover_window_start_s_) > cfg::kWatchdogRecoverWindowS) {
    recover_count_ = 0;
    recover_window_start_s_ = now_s;
  }
  if (++recover_count_ > cfg::kWatchdogRecoverMaxCount) return WatchdogCheck::Fault;

  for (uint8_t id : kIds) {
    if (!watchdogRecoverOne(id)) return WatchdogCheck::Fault;
  }
  return WatchdogCheck::Recovered;
}

bool DxlBackend::verifySafeOff() {
  // 保存ゲートの前提 (§9.1): 両輪 Torque Enable==0 かつ 両輪 Goal Current==0
  for (uint8_t id : kIds) {
    if (!verifyByte(id, kAddrTorqueEnable, 0)) return false;
    uint8_t v[2];
    if (!readRaw(id, kAddrGoalCurrent, 2, v) || units::le16(v) != 0) return false;
  }
  return true;
}

void DxlBackend::pollHealth(HealthInfo* out) {
  // 低頻度ラウンドロビン: 1 呼び出し 1 トランザクション (§8.2 + raw98)
  const uint8_t id = kIds[health_phase_ & 1];
  switch ((health_phase_ >> 1) & 3) {
    case 0: {
      uint8_t v[2];
      if (readRaw(id, kAddrPresentVoltage, 2, v)) {
        health_.voltage = units::le16(v) * 0.1f;
      }
      break;
    }
    case 1: {
      uint8_t t = 0;
      if (readRaw(id, kAddrPresentTemp, 1, &t)) {
        health_.temperature = static_cast<float>(t);
      }
      break;
    }
    case 2: {
      uint8_t e = 0;
      if (readRaw(id, kAddrHwErrorStatus, 1, &e)) {
        health_.hw_error = (id == cfg::kDxlIdLeft)
                               ? ((health_.hw_error & 0x0F) | (e ? 0x10 : 0))
                               : ((health_.hw_error & 0xF0) | (e ? 0x01 : 0));
        if (e != 0) health_.hw_error |= 0x80;
      }
      break;
    }
    case 3: {
      uint8_t wd = 0;
      if (readRaw(id, kAddrBusWatchdog, 1, &wd)) {
        if (wd == kWatchdogTripped) health_.watchdog_tripped = true;
      }
      break;
    }
  }
  ++health_phase_;
  *out = health_;
}

}  // namespace hw
