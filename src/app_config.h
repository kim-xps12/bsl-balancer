// app_config.h — 全定数の単一情報源 (SSOT)
// 設計書: docs/plans/2026-07-02-current-mode-freertos-redesign.md
// 単位は SI (rad, rad/s, m, m/s, A, s)。TUNE = 実機調整前提の初期値。
#pragma once

#include <cstdint>

namespace cfg {

// ---- 制御周期 (§3.3) ----
constexpr uint32_t kControlPeriodMs = 5;    // 200 Hz
constexpr float    kControlPeriodS  = 0.005f;
constexpr float    kDtFaultFactor   = 1.5f; // dt > 1.5x周期 が連続で FAULT
constexpr int      kDtFaultConsecutive = 5;

// ---- ハードウェア (PORT A / DYNAMIXEL) ----
constexpr uint8_t  kPinRxServo = 33;
constexpr uint8_t  kPinTxServo = 32;
constexpr uint32_t kDxlBaud    = 1000000;  // サーボ EEPROM 設定値と一致必須 (README 手順で 1M 化済み)
constexpr uint8_t  kDxlIdLeft  = 0;
constexpr uint8_t  kDxlIdRight = 1;
constexpr uint16_t kDxlModelNumber = 1190; // XL330-M077
constexpr float    kDxlProtocol = 2.0f;

// ---- 機体諸元 ----
constexpr float kWheelRadiusM = 0.029f;  // TAMIYA ナロータイヤ 58mm 径

// ---- 符号規約 (§4.2/§7 実機符号試験で確定。指令と帰還の両方に適用) ----
constexpr float kSignLeft  = -1.0f; // 現行実装準拠 (L に -指令で前進)
constexpr float kSignRight = +1.0f;

// ---- IMU 軸マップ (§5.1 縦置き: 直立で重力≈+Y、傾斜で Z に射影) ----
// tilt = atan2(sign_tilt*accZ, sign_vert*accY) が直立≈0・前傾で正になるよう
// 符号試験 (§7) で確定する。gyro は atan2(Z,Y) の時間微分と同符号にする
// (旧 atan2(Y,Z) 基準とは微分符号が反転するため既定 -1)。
constexpr float kImuAccTiltSign = +1.0f;
constexpr float kImuAccVertSign = +1.0f;
constexpr float kImuGyroSign    = -1.0f;

// ---- IMU / 姿勢推定 (§5) ----
constexpr float kEstimatorTauS      = 1.0f;   // 相補融合時定数 TUNE
constexpr float kAccelGateG         = 0.3f;   // ||a|-1g| > 0.3g で補正停止
constexpr float kGravity            = 9.80665f;
constexpr float kGyroCalibDurationS = 1.5f;   // 起動時静止校正
constexpr int   kImuStaleFaultCycles = 5;     // gyro stale 連続 25ms で FAULT

// ---- DXL 通信規律 (§4.2) ----
constexpr uint32_t kDxlIoTimeoutMs        = 2;   // 全トランザクション明示タイムアウト
constexpr float    kDxlCycleBudgetS       = 0.003f; // 1周期内 DXL 総予算 3ms
constexpr int      kDxlReadStaleFaultCycles = 40; // 読取失敗 連続 200ms で FAULT
// (実バスは読取が単発で落ちる。失敗周期はコースト(§4.2)なので 200ms まで許容)
constexpr float    kUnverifiedTorqueMaxS  = 0.150f; // 未検証トルク窓 (壁時計)
// (実測バーストは50msでも不足する場合がある: 3分連続倒立の末に50ms超の
//  バス断でラッチした実績。窓内は毎周期の検証付き零書込で回復を試み続け、
//  バス完全断は XL330 の Bus Watchdog(20ms) が自律的にトルク遮断する)
constexpr uint8_t  kBusWatchdogRaw        = 1;   // 20ms (raw 1 = 最小)
constexpr float    kBusWatchdogWindowS    = 0.020f;
constexpr float    kQuarantineSilenceS    = 0.040f; // 検疫沈黙窓 (Watchdog窓+余裕)
constexpr int      kWatchdogRecoverMaxCount = 3;  // 60s 内 3 回で FAULT
constexpr float    kWatchdogRecoverWindowS  = 60.0f;

// ---- 電流制限 3 層 + I²t (§2.3) ----
constexpr float kCurrentLimitA        = 0.900f; // EEPROM Current Limit(38) TUNE
constexpr float kCurrentPeakA         = 0.450f; // ソフトピーク TUNE
constexpr float kCurrentContA         = 0.300f; // ソフト連続 TUNE
constexpr float kPeakDurationS        = 0.5f;   // I²t: E_max=(Ip²-Ic²)*Tpeak TUNE
constexpr float kCurrentSlewAPerS     = 5.0f;   // スルーレート TUNE

// ---- 電流妥当性監視 (§6。Present Current は電源側電流のため寛大に) ----
constexpr float kCurrentMismatchA      = 0.300f; // |I_present-I_cmd| 閾値 TUNE
constexpr float kCurrentResidualA      = 0.100f; // 零指令中の残留閾値 TUNE
constexpr float kCurrentPlausDwellS    = 0.200f;

// ---- 速度ガード (§2.1 / XL330規範 §11。無負荷 383rpm≈40.1rad/s) ----
constexpr float kWheelSpeedSoftRadS = 25.0f; // TUNE
constexpr float kWheelSpeedHardRadS = 35.0f; // 超過で FAULT TUNE

// ---- 制御ゲイン初期値 (§2.2 TUNE) ----
constexpr float kPitchKp        = 3.0f;    // [A/rad] 実機テレメトリ較正: 1.5では
// 3.2Hz/±7.5°のリミットサイクル (i_cmd_sd 0.09A と過小authority) → 2倍へ
constexpr float kPitchKi        = 0.0f;    // [A/(rad*s)] Bala2 実績に従い 0
constexpr float kPitchKd        = 0.15f;   // [A/(rad/s)] Kp 比を概ね維持して増加
constexpr float kPitchILimitA   = 0.15f;   // 積分項クランプ [A]
constexpr float kDTermLpfHz     = 25.0f;   // D 項 PT1 カットオフ
constexpr float kVelKv          = 0.10f;   // [rad/(m/s)]
constexpr float kVelKvi         = 0.05f;   // [rad/m]
constexpr float kThetaRefLimitRad = 0.0524f; // ±3° クランプ
constexpr bool  kVelLoopEnabledDefault = true;
constexpr float kPitchEqDefaultRad = 0.0925f; // 平衡点校正値 (0中心 θ の単一変換点でのみ使用)
// 2026-07-07 実機テレメトリで較正: 直立静止時の生ピッチ実測 +5.3° (30s平均、
// 完全静止・車輪停止で±5°窓を恒常的に外しアーム不能だった)。UI の Eq[deg] で微調整可

// ---- 状態機械 (§6) ----
constexpr float kFallThresholdRad    = 0.611f;  // 35°
constexpr float kStartWindowRad      = 0.0873f; // 5°
constexpr float kStartRateMaxRadS    = 0.35f;   // ~20°/s
constexpr float kStartWheelMaxRadS   = 1.0f;
// #ifndef ガード化 (wifi-guard-trace 計画書 §6 T5): trace ビルドのみ
// platformio.ini から -DBSL_UPRIGHT_HOLD_S=5.0f を注入し、arm_pending
// ホールド窓を延長する (AP 電源断のビーコン喪失タイムアウト到達をベンチで
// 決定的に観測するため)。リリース値・型・意味は不変 (未定義時は 0.5f)。
#ifndef BSL_UPRIGHT_HOLD_S
#define BSL_UPRIGHT_HOLD_S 0.5f
#endif
constexpr float kUprightHoldS        = BSL_UPRIGHT_HOLD_S;  // 起立/静置検出の保持時間
constexpr int   kFallEscalationCount = 3;       // 30s 内 3 回で FAULT ラッチ
constexpr float kFallEscalationWindowS = 30.0f;

// ---- ヘルス監視閾値 ----
constexpr float kVoltageMinV   = 3.6f;  // AAA×3 のサグ考慮 TUNE
constexpr float kTempMaxC      = 65.0f; // XL330 Shutdown(70°C 相当) 手前 TUNE

// ---- UI ----
constexpr uint32_t kUiPeriodMs        = 33;
constexpr uint32_t kBtnLongPressMs    = 1000; // BtnC STOP/ARM トグル

// ---- NVS レコード (§9.1) ----
constexpr uint16_t kParamSchemaVersion       = 1;

// ---- パラメータ許容範囲 [min, max] (§9.1 範囲検証。1つでも外れたら全体破棄) ----
struct ParamRange { float min; float max; };
constexpr ParamRange kRangeKp        {0.0f, 10.0f};
constexpr ParamRange kRangeKi        {0.0f, 5.0f};
constexpr ParamRange kRangeKd        {0.0f, 1.0f};
constexpr ParamRange kRangeKv        {0.0f, 1.0f};
constexpr ParamRange kRangeKvi       {0.0f, 1.0f};
constexpr ParamRange kRangePitchEq   {-0.35f, 0.35f}; // ±20°
constexpr ParamRange kRangeThetaRefLimit {0.0f, 0.175f}; // ±10°

// ---- UDP テレメトリ (Phase1 追加。docs/plans/2026-07-05-udp-telemetry-phase1.md §3.1) ----
constexpr uint32_t kTelemetryPeriodMs = 50;  // 20 Hz。core1 / priority1 / stack8192
// Wi-Fi guard の abort 完了確認 (計画書 §3.1: 目安 3 tick = 150ms @50ms周期)
constexpr uint32_t kWifiAbortConfirmTicks = 3;
constexpr int      kWifiAbortMaxRetries = 3;      // 計画書 §3.1: 有界リトライ最大3回
// lib_reconnect_pending フォールバック タイムアウト (計画書 §3.1: 目安15s) の tick 換算
constexpr uint32_t kWifiLibReconnectTimeoutTicks = 300;
// 接続失敗後の再 begin() までの猶予 (busy loop 防止。計画書は具体秒数未規定のため
// 保守側の安全なデフォルトとして採用。TUNE)
constexpr uint32_t kWifiReconnectBackoffTicks = 60;  // 3s @50ms

}  // namespace cfg
