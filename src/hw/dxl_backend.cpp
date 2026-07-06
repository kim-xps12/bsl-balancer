#include "dxl_backend.h"

#include <Arduino.h>
#include <cmath>

#include "../core/dxl_verify.h"
#include "../core/units.h"
#include "../core/watchdog_policy.h"

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
constexpr uint8_t kShutdownDefault = 53;    // XL330 既定 (0b00110101、e-manual 確認)

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
  // view 構築 (§3.1 call-site 契約: rx==nullptr 時にフィールドへ一切触れない。
  // 引数評価は関数呼出し前に行われるため rx->field を引数式に書かない)
  core::DxlStatusView st;
  if (rx != nullptr) {
    st.ok = true;
    st.id = rx->id;
    st.err_idx = rx->err_idx;
    st.recv_param_len = rx->recv_param_len;
  }
  // 受理判定は core::isWriteStatusVerified へ抽出済み (段階1a。他 ID/残留
  // 応答の誤消費・ALERT(0x80) 含む非零・前回 READ の遅延応答誤受理を拒否)
  return core::isWriteStatusVerified(st, id);
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
  // view 構築 (§3.1 call-site 契約: rx==nullptr 時にフィールドへ一切触れない)
  core::DxlStatusView st;
  if (rx != nullptr) {
    st.ok = true;
    st.id = rx->id;
    st.err_idx = rx->err_idx;
    st.recv_param_len = rx->recv_param_len;
  }
  // 受理判定は core::isReadStatusVerified へ抽出済み (段階1a。READ の ALERT
  // はデータ有効 — err の ALERT ビット以外 (Instruction/CRC 等の Result
  // Fail) は失敗扱い)
  if (!core::isReadStatusVerified(st, id, len)) return false;
  // データコピーは成功時のみ実行 (現行と同順序。st.ok==true が保証されて
  // いるためこの時点で rx は非 nullptr)
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
      // Shutdown(63) は既定値 53 (過熱/過負荷/電圧/ショック保護有効) を要求。
      // 過去に無効化されたまま残っていたら安全既定へ復元して検証する
      uint8_t sd = 0;
      if (!readRaw(id, kAddrShutdown, 1, &sd)) return false;
      if (sd != kShutdownDefault) {
        if (!writeRaw1(id, kAddrShutdown, kShutdownDefault)) return false;
        if (!verifyByte(id, kAddrShutdown, kShutdownDefault)) return false;
      }
    }

    // 前回稼働の Watchdog 状態を先に無効化する。トリップ(0xFF)中は Goal 値が
    // read-only になるため零書込より前に解除が必要。加えて ESP32 のみ再起動
    // した場合は武装(raw=1)のまま残り、以降の初期化トランザクションが 20ms
    // 窓を超えるとトリップするため、非零なら一律 0 (無効) へ落とす
    uint8_t wd = 0;
    if (!readRaw(id, kAddrBusWatchdog, 1, &wd)) return false;
    if (wd != 0) {
      if (!writeRaw1(id, kAddrBusWatchdog, 0)) return false;
    }

    // ★Goal Current=0 (モード変更で Current Limit 値へ自動セットされるため必須)
    {
      const uint8_t z[2] = {0, 0};
      if (!verifiedWrite(id, kAddrGoalCurrent, z, 2)) return false;
    }
  }
  // Bus Watchdog 有効化は全サーボの設定完了後にまとめて行う (片側だけ先に
  // 武装すると残りの初期化トランザクションが 20ms 窓を超えた場合に潜在
  // トリップする)。有効化直後から心拍 (零書込) が 5ms 周期で走る前提。
  for (uint8_t id : kIds) {
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
    if ((x.error & 0x7F) != 0) return fb;     // 非 ALERT エラーの応答は採用しない
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

bool DxlBackend::enterBalancing(float now_s) {
  // §4.1 enter_balancing(): 唯一の突入契約。
  // トルク OFF 中の潜在トリップ (起動/校正中の心拍ギャップ等) はここで §4.3 の
  // 安全復旧を実行してから進む — 復旧可能な状態で EntryVerifyFailed に落とさない
  {
    uint8_t wd0 = 0, wd1 = 0;
    const bool r0 = readRaw(kIds[0], kAddrBusWatchdog, 1, &wd0);
    const bool r1 = readRaw(kIds[1], kAddrBusWatchdog, 1, &wd1);
    if (!r0 || !r1) return false;
    if (wd0 == kWatchdogTripped || wd1 == kWatchdogTripped) {
      if (checkWatchdog(/*torque_may_be_on=*/false, now_s) !=
          WatchdogCheck::Recovered) {
        return false;
      }
    }
  }
  for (uint8_t id : kIds) {
    uint8_t hw_err = 0xFF;
    if (!readRaw(id, kAddrHwErrorStatus, 1, &hw_err) || hw_err != 0) return false;
    uint8_t wd = 0;
    if (!readRaw(id, kAddrBusWatchdog, 1, &wd)) return false;
    if (wd == kWatchdogTripped) return false;  // 復旧後も残るなら Torque ON 禁止
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
  // 零書込(検証) → Torque OFF(読み戻し)。**サーボ毎に独立してベストエフォート**
  // で実行する — 片側が無応答でも健常側の Torque OFF を必ず試みる (部分故障で
  // 応答する側を駆動されたまま放置しない)。全段検証成功のみ true、
  // 失敗があれば検疫 (§4.2 fail-stop)。
  bool all_ok = true;
  for (uint8_t id : kIds) {
    const uint8_t z[2] = {0, 0};
    bool zero_ok = verifiedWrite(id, kAddrGoalCurrent, z, 2);
    if (zero_ok) {
      uint8_t v[2];
      zero_ok = readRaw(id, kAddrGoalCurrent, 2, v) && units::le16(v) == 0;
    }
    // 零が未検証でも Torque OFF は独立に試みる (OFF 自体が無通電化)
    bool off_ok = writeRaw1(id, kAddrTorqueEnable, 0);
    off_ok = off_ok && verifyByte(id, kAddrTorqueEnable, 0);
    all_ok = all_ok && zero_ok && off_ok;
  }
  if (!all_ok) engageQuarantine();
  return all_ok;
}

bool DxlBackend::watchdogRecoverOne(uint8_t id) {
  if (!writeRaw1(id, kAddrBusWatchdog, 0)) return false;  // エラー解除
  uint8_t v[2];
  if (!readRaw(id, kAddrGoalCurrent, 2, v) || units::le16(v) != 0) return false;
  return writeRaw1(id, kAddrBusWatchdog, cfg::kBusWatchdogRaw);  // 再有効化
}

WatchdogCheck DxlBackend::checkWatchdog(bool torque_may_be_on, float now_s) {
  // §4.3: 判定根拠は両輪の実測 (raw98 + Torque Enable 読み戻し)。判断は
  // core::WatchdogDecision (逐次リデューサ) へ抽出済みで、ここは I/O の
  // 位置・回数・順序を現行と同一に保つだけ (段階1b。
  // docs/plans/2026-07-06-safety-core-extraction.md §3.2)。
  core::WatchdogDecision decision(torque_may_be_on, kWatchdogTripped);

  // raw98 読取ループ: L→R の順、Pending 以外を返した時点で以降を読まない
  // (早期 return の I/O 回数同一性)
  core::WatchdogVerdict verdict = core::WatchdogVerdict::Pending;
  for (int k = 0; k < 2 && verdict == core::WatchdogVerdict::Pending; ++k) {
    uint8_t wd = 0;
    const bool ok = readRaw(kIds[k], kAddrBusWatchdog, 1, &wd);
    verdict = decision.feedRaw(ok, wd);
  }
  if (verdict == core::WatchdogVerdict::Ok) return WatchdogCheck::Ok;
  if (verdict == core::WatchdogVerdict::Fault) return WatchdogCheck::Fault;

  // ここまで到達 = トリップ検出 (Pending)。Torque Enable 読み戻しへ
  // (集約規則: 両輪成功かつ両輪 0 のみ自動復旧)。同様に L→R・早期打切り。
  for (int k = 0; k < 2 && verdict == core::WatchdogVerdict::Pending; ++k) {
    uint8_t te = 0;
    const bool ok = readRaw(kIds[k], kAddrTorqueEnable, 1, &te);
    verdict = decision.feedTe(ok, te);
  }
  if (verdict == core::WatchdogVerdict::Fault) return WatchdogCheck::Fault;

  // verdict == ProceedRecover: 復旧頻度制限 (60s 内 3 回で FAULT)
  if (!limiter_.allow(now_s, cfg::kWatchdogRecoverWindowS,
                       cfg::kWatchdogRecoverMaxCount)) {
    return WatchdogCheck::Fault;
  }

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
        // 現在値で更新 (復旧後に stale なトリップ表示を残さない)
        health_.watchdog_tripped = (wd == kWatchdogTripped);
      }
      break;
    }
  }
  ++health_phase_;
  *out = health_;
}

}  // namespace hw
