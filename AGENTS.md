# AGENTS.md

BSL-Balancer（ﾀｲﾘﾝﾁｬﾝ）ファームウェア。M5Stack Core2 + DYNAMIXEL XL330 による対向二輪型倒立振子の PID 制御ファームウェア。親プロジェクト [`bsl-fuzzy-balancer`](../../) の Python 制御アルゴリズム最適化で得たゲインを実機にデプロイする。

## Tech Stack

- Platform: ESP32（M5Stack Core2）/ Arduino Framework
- ビルド: **PlatformIO**（Arduino IDE は非サポート）
- 言語: C++11（gnu++11）
- 主要ライブラリ: M5Unified 0.2.7, Dynamixel2Arduino 0.8.1, Kalman Filter 1.0.2, M5Stack-Avatar 0.9.2

## コマンド

```bash
# ビルド
pio run -e m5stack-core2

# ビルド＆書き込み（USB 接続時）
pio run -e m5stack-core2 -t upload

# シリアルモニタ（115200 baud）
pio device monitor -b 115200

# ライブラリ依存の更新
pio pkg update
```

## プロジェクト構造

`tree -L 1 src/ case/ docs/` で確認。ソースは `src/` 以下の 3 ファイルのみ:

| ファイル | 役割 |
|----------|------|
| `src/main.cpp` | PID 制御ループ + UI（FreeRTOS タスク 2 本） |
| `src/TairinEye.h/cpp` | カスタム目描画（m5stack-avatar 拡張） |
| `src/TairinMouth.h/cpp` | カスタム口描画（m5stack-avatar 拡張） |

## アーキテクチャ

FreeRTOS デュアルコア構成:

```
Core 0: controlLoopTask (10ms 周期, 優先度 5)
  → IMU 読取 → Kalman フィルタ → PID 演算 → DYNAMIXEL 電流指令

Core 1: uiLoopTask (50ms 周期, 優先度 2)
  → Avatar 顔描画 / コントロールパネル表示 / タッチ入力処理
```

## ハードウェア構成

```cpp
// ピン
PIN_RX_SERVO = 33   // PORT A (GPIO33)
PIN_TX_SERVO = 32   // PORT A (GPIO32)

// DYNAMIXEL
DXL_ID_L = 0        // 左モーター
DXL_ID_R = 1        // 右モーター
Baudrate = 1000000   // 1 Mbps, Protocol 2.0, Current Control Mode

// IMU: MPU6886（M5Stack Core2 内蔵）
// pitch = atan2(accY, accZ), pitchDot = gyroX
```

## コード規約

- グローバル変数で PID パラメータ管理（`Kp`, `Ki`, `Kd`, `pitch_target`）
- 関数名は camelCase（`calcPID`, `driveCurrent`, `getPitch`）
- クラス名は camelCase（`tairinEye`, `tairinMouth`）— m5stack-avatar 由来の命名を踏襲
- ハードウェアピンは `const` で `PIN_` プレフィクス
- デバッグ出力は `#define ENABLE_DEBUG_PRINT` で条件コンパイル（通常は無効）

## 制御パラメータと安全制約

```cpp
// PID デフォルト値（Current Control Mode, SI 単位）
Kp = 3.0   // [A/rad]
Ki = 0.0   // [A/(rad*s)]（初期は無効）
Kd = 0.1   // [A/(rad/s)]
pitch_target = 86.0  // [deg] IMU 基準角

// フェイルセーフ（ラッチ式：復帰には再起動が必要）
if (|pitch_error| > 40°) → 電流 0 & フォルトラッチ
if (|wheel_speed| > 350 RPM) → 電流 0 & フォルトラッチ
if (通信エラー) → 電流 0 & フォルトラッチ

// 電流制限
SOFTWARE_CURRENT_LIMIT_A = 0.5  // ソフトウェア上限
EEPROM Current Limit = 1000 mA // ハードウェア上限
Bus Watchdog = 100 ms          // 通信途絶タイムアウト

// アンチワインドアップ: clamping + back-calculation
// D 項: 測定角速度の 1 次 LPF（τ = 20 ms）
```

**これらの閾値（40°, 350 RPM, 0.5 A, 1000 mA）は安全に直結するため、変更時は必ずユーザに確認する。**

## Git 運用

親プロジェクトのルールを継承する（[`../../AGENTS.md`](../../AGENTS.md) の「Git 運用」セクション参照）。

- コミットメッセージは**日本語**（件名 50 字以内）
- 末尾に `Co-Authored-By:` フッター

## 境界

### Always

- `pio run` でビルドが通ることを確認してからコミット
- ハードウェアピン番号・モーター ID の変更時はスキーマティクスとの整合を確認
- 電流指令は左モーター `-current_A`、右モーター `+current_A` の符号規約を維持
- 前進速度の推定は Present Velocity から `(-vel_L + vel_R) / 2` で合成する

### Never

- フェイルセーフ閾値（±40°）、電流上限（0.5 A / 1000 mA）、速度上限（350 RPM）を無断で変更しない
- `.pio/` 配下をコミットしない（ビルド成果物）

### Ask First

- PID デフォルトゲインの変更
- ライブラリバージョンのアップグレード
- IMU フルスケールレンジの変更（コメントアウト箇所: `setAccelFsr`, `setGyroFsr`）
- FreeRTOS タスク優先度・周期の変更

## 参考リソース

- 親プロジェクト（Python 制御アルゴリズム）: [`../../AGENTS.md`](../../AGENTS.md)
- カスタム Dynamixel IF 基板: [m5stack_board_dynamixel_ttl_rs3485](https://github.com/kim-xps12/m5stack_board_dynamixel_ttl_rs3485)
- MPU6886 データシート: M5Stack Core2 内蔵 IMU
