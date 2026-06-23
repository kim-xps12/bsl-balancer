# Plan: XL330 電流制御モードへの移行

**日付:** 2026-06-23
**対象:** `firmware/bsl-balancer`（独立リポジトリ）
**参照:** 親リポジトリ `bsl-fuzzy-balancer/docs/XL330_M077_TWIP_coding_agent_reference.md`

---

## 背景

現在のファームウェアは DYNAMIXEL XL330 の **速度制御モード** (`OP_VELOCITY`) で動作し、
Goal Velocity (addr 104, 4 byte) を送信している。リファレンスに従い、
**電流制御モード** (`Operating Mode(11) = 0`) へ移行し、Goal Current (addr 102, 2 byte) を
送信する実装へ変更する。

### 移行理由

- 外部倒立制御器が電流指令を計算するため、DYNAMIXEL 内部速度 PI 制御器は不要
- MuJoCo `<motor>` モデル（`gear=0.05239` はシステム同定で得た実効パラメータ）と
  実機の共通入力を「電流指令 [A]」に統一できる
- **注意:** XL330 の Present Current は入力電源側電流であり、モータ相電流ではない。
  Goal Current → 出力軸トルクの関係は理想的な一定係数では表せないため、
  `gear=0.05239` を N·m/A と断定せず、同定条件での実効入力ゲインとして扱う。
  制御ゲインの初期チューニングは車体浮かせ試験で実験的に決定する。

### 同定入力単位のスタートアップゲート

リファレンス Section 15.3 に従い、以下の設定を起動時の必須確認項目とする:
- `CTRL_UNIT`: 同定時の入力単位（A / mA / raw / normalized）
- `FIT_INPUT_SOURCE`: 同定に使用した入力（goal_current / present_current）

これらは `config.h` にプリプロセッサ定義として持ち、
**未設定（`CTRL_UNIT_UNSET`）の場合は BALANCING 状態への遷移を禁止**する。
Goal Current に非 0 値を書くパスはすべてブロックされる。
起動時にシリアルログへ現在の設定値を出力する。

```cpp
#define CTRL_UNIT_UNSET       0
#define CTRL_UNIT_A           1
#define CTRL_UNIT_MA          2
#define CTRL_UNIT_RAW         3
#define CTRL_UNIT_NORMALIZED  4

#define FIT_INPUT_UNSET       0
#define FIT_INPUT_GOAL        1
#define FIT_INPUT_PRESENT     2

// *** fail-closed: 両方 UNSET で出荷。確定後に手動で設定する ***
#define CTRL_UNIT             CTRL_UNIT_UNSET
#define FIT_INPUT_SOURCE      FIT_INPUT_UNSET

// コンパイル時バリデーション: 許可された値以外でビルドエラー
static_assert(
    CTRL_UNIT == CTRL_UNIT_UNSET || CTRL_UNIT == CTRL_UNIT_A ||
    CTRL_UNIT == CTRL_UNIT_MA || CTRL_UNIT == CTRL_UNIT_RAW ||
    CTRL_UNIT == CTRL_UNIT_NORMALIZED,
    "CTRL_UNIT must be one of: UNSET(0), A(1), MA(2), RAW(3), NORMALIZED(4)");
static_assert(
    FIT_INPUT_SOURCE == FIT_INPUT_UNSET || FIT_INPUT_SOURCE == FIT_INPUT_GOAL ||
    FIT_INPUT_SOURCE == FIT_INPUT_PRESENT,
    "FIT_INPUT_SOURCE must be one of: UNSET(0), GOAL(1), PRESENT(2)");
```

**両方を UNSET のまま**出荷する。

**コンパイル時ゲート:** `static_assert` で許可値以外はビルドエラー（typo 防止）。

**ランタイムゲート:** `CTRL_UNIT == CTRL_UNIT_UNSET` または
`FIT_INPUT_SOURCE == FIT_INPUT_UNSET` の場合:
- **Torque Enable を含む全ての駆動パスを禁止**（DISARMED に留まる）
- 初期化シーケンスのステップ 11（Torque Enable=1）を実行しない
- Bus Watchdog も有効化しない
- Goal Current に非 0 値を書くパスをすべてブロック
- シリアルログに `"BLOCKED: CTRL_UNIT or FIT_INPUT_SOURCE not configured"` を出力
- IMU キャリブレーション、PING、設定検証まではそのまま実行（診断目的）

同定単位が確定した後、`config.h` を編集して再ビルドする。

---

## ROBOTIS e-Manual 準拠の制御テーブル仕様

本計画で使用するアドレス・単位・挙動を明記する（リファレンス Section 6 に基づく）。

| アドレス | サイズ | 項目 | 単位・挙動 |
|---:|---:|---|---|
| 11 | 1 | Operating Mode | `0`: Current Control |
| 38 | 2 | Current Limit | ~1 mA/count, EEPROM, Torque OFF 時のみ書込可 |
| 64 | 1 | Torque Enable | `0`: OFF, `1`: ON |
| 70 | 1 | Hardware Error Status | bit mask |
| 98 | 1 | Bus Watchdog | 20 ms/count, `0`: 無効, `-1`: エラー状態 |
| 102 | 2 | Goal Current | ~1 mA/count, 符号付き 16-bit |
| 126 | 2 | Present Current | ~1 mA/count, 符号付き 16-bit |
| 128 | 4 | Present Velocity | 0.229 rpm/count, 符号付き 32-bit |
| 132 | 4 | Present Position | 4096 pulse/rev, 符号付き 32-bit |
| 144 | 2 | Present Input Voltage | 0.1 V/count |
| 146 | 1 | Present Temperature | 1 °C/count |

**重要なリセット挙動:**
- Operating Mode 変更後、Goal Current は Current Limit の値にリセットされる
- Torque ON 前に必ず Goal Current = 0 を書かなければならない

---

## 通信時間予算

### 前提条件
- ボーレート: 1 Mbps
- Return Delay Time: デフォルト 250 (500 μs) → **0 に設定する**
- プロトコル: DYNAMIXEL Protocol 2.0
- パケット構造: Header(4) + Reserved(1) + ID(1) + Length(2) + Instruction(1) + Params + CRC(2)

### Sync Write (Goal Current, 2 byte × 2 台)
```
TX: Header(4) + Reserved(1) + ID_broadcast(1) + Length(2) + Inst(1)
    + StartAddr(2) + DataLen(2)
    + [ID_L(1) + Data(2)] + [ID_R(1) + Data(2)]
    + CRC(2) = 20 byte
送信時間: 20 × 10 / 1,000,000 = 200 μs
応答なし（broadcast）
合計: ~200 μs
```

### Sync Read (addr 126, 10 byte × 2 台)
```
TX: Header(4) + Reserved(1) + ID_broadcast(1) + Length(2) + Inst(1)
    + StartAddr(2) + DataLen(2)
    + ID_L(1) + ID_R(1)
    + CRC(2) = 18 byte
送信時間: 18 × 10 / 1,000,000 = 180 μs

RX per motor: Header(4) + Reserved(1) + ID(1) + Length(2) + Inst(1)
              + Error(1) + Data(10) + CRC(2) = 22 byte
受信時間: 22 × 10 / 1,000,000 = 220 μs × 2台 = 440 μs
Return Delay: 0 μs × 2 = 0 μs

合計: 180 + 440 = 620 μs
```

### ヘルス監視 (非連続アドレス: 個別 Read × 3 系統 × 2 台)

Hardware Error Status(70, 1 byte)、Present Input Voltage(144, 2 byte)、
Present Temperature(146, 1 byte) は非連続アドレスのため、
**1 回の Sync Read では読めない**。以下の方法で読み取る:

**方法: 制御ループ内で分散読み取り（各項目 5 Hz）**
- 40 tick ごとにスタガ配置。各項目は 40 tick = 200 ms = 5 Hz で更新
  - tick % 40 == 0: HW Error Status (2 台分、個別 Read)
  - tick % 40 == 13: Voltage (2 台分、個別 Read)
  - tick % 40 == 26: Temperature (2 台分、個別 Read)
- 1 tick あたり最大 1 項目のみ実行（衝突なし）
- 各読み取りは 1 台あたり TX ~120 μs + RX ~120 μs = ~240 μs × 2 台 = ~480 μs

**タイムアウト時予算:**
- 片側タイムアウト: ~240 + 2000 = 2240 μs
- 上記の pre-Sync Read 予算ガードの後に実行するため、
  ヘルス読み取り前にも remaining を確認し、不足時はスキップ

**ヘルス読み取り鮮度管理（fail-closed、モータ×項目単位）:**
- `last_success_tick[2][3]` で左右各モータ × 3 項目 (HW Error, Voltage, Temp) を追跡
- `current_tick - last_success_tick[motor][item] > HEALTH_STALE_LIMIT` で FAULT
- `HEALTH_STALE_LIMIT = 400` ticks = 2 秒（5 Hz 読み取りの 10 回分）
- 起動直後: 全 6 セル（2 モータ × 3 項目）の初回読み取り完了するまで
  BALANCING 遷移を禁止（`initial_health_valid` ビットマップで管理）

### タイムアウトシナリオ別予算

Dynamixel2Arduino の Sync Read は**モータごとに個別タイムアウト**を適用する。
デフォルトタイムアウトは約 2000 μs/パケット。

```
正常 (Return Delay=0):
  TX: 180 μs + RX×2: 440 μs = 620 μs

片側タイムアウト (1台応答なし):
  TX: 180 μs + RX×1: 220 μs + timeout: 2000 μs = 2400 μs

両側タイムアウト (2台とも応答なし):
  TX: 180 μs + timeout×2: 4000 μs = 4180 μs

Return Delay 残存時 (500 μs × 2台):
  TX: 180 μs + (RD + RX)×2: (500+220)×2 = 1620 μs → 合計 1800 μs
```

### 時間予算まとめ
```
制御周期: 5000 μs (200 Hz)

正常時:
  Sync Write:       200 μs
  Sync Read:        620 μs
  IMU update:       ~200 μs
  制御演算:         ~100 μs
  合計:            ~1120 μs (周期の 22%)

片側タイムアウト時:
  合計:            ~2900 μs (58%) — 周期内に収まる

両側タイムアウト時:
  合計:            ~4680 μs (94%) — 周期超過の危険
```

### pre-Sync Read 予算ガード
Sync Read 開始前に残り時間を確認し、最悪ケース（片側タイムアウト）に
収まらない場合はスキップする:
```
elapsed = esp_timer_get_time() - loop_start
remaining = 5000 - elapsed

// 片側タイムアウト時の最悪所要: 2400 μs + 余裕 200 μs = 2600 μs
if remaining < 2600:
  Sync Read をスキップ、前回値を使用
  skip_count++

if skip_count >= 3:
  電流指令を 0 にフォールバック（フィードバック不明で駆動しない）
if skip_count >= 5:
  FAULT 遷移
```

Sync Read の明示タイムアウト値は Dynamixel2Arduino のデフォルト（~2000 μs/パケット）を維持する。

### pass/fail 基準
- `loop_us > 4000`: 警告ログ
- `loop_us > 4500`: 連続 3 回で FAULT
- Sync Read タイムアウト: 2000 μs/パケット（Dynamixel2Arduino 内部）
- Sync Read 連続失敗: 5 回で FAULT（電流制御モードでは速度無効化ではなく FAULT）
- 両側タイムアウトが発生した場合: 即座に電流 0 + 次回も失敗で FAULT

---

## 速度ガードの符号安全設計

### 符号規約（config.h で定義）
```
SIGN_LEFT_MOTOR  = -1   (DXL raw 正回転が車体後退)
SIGN_RIGHT_MOTOR = +1   (DXL raw 正回転が車体前進)
```

### 双方向の符号変換

**フィードバック: raw → 車体フレーム**
```
omega_body_L = SIGN_LEFT_MOTOR  × velocity_raw_to_rad_s(raw_L)
omega_body_R = SIGN_RIGHT_MOTOR × velocity_raw_to_rad_s(raw_R)
```

**コマンド: 車体フレーム → raw**
```
goal_current_raw_L = current_A_to_raw(SIGN_LEFT_MOTOR  × i_cmd_body_L)
goal_current_raw_R = current_A_to_raw(SIGN_RIGHT_MOTOR × i_cmd_body_R)
```

制御器は**車体フレーム**で電流指令を計算し、
Sync Write 直前で `SIGN_*_MOTOR` を掛けて raw 値へ変換する。
フィードバック読み取り直後に `SIGN_*_MOTOR` を掛けて車体フレームへ変換する。

### 加速判定（車体フレーム）

速度ガードは左右各輪について**車体フレーム**で判定する。

```
accelerating = (i_cmd_body × omega_body) > 0

速度ガード適用（左右各輪に独立適用）:
  if |omega_body| >= omega_hard → FAULT
  if accelerating && |omega_body| > omega_soft:
    scale = (omega_hard - |omega_body|) / (omega_hard - omega_soft)
    i_cmd_body *= clamp(scale, 0, 1)
```

### フィードバック無効時の安全デフォルト
- Sync Read 失敗時は**電流指令を 0** にする（加速許可しない）
- 速度値が更新されていない（前回と同一 timestamp）場合も同様

### 符号検証手順（実機試験）
**Step 1: raw 特性確認**
1. 車体を浮かせ、左モータのみ Goal Current raw = +50 を書く
2. Present Velocity raw の符号を記録
3. 右モータも同様に記録
4. → 左: raw +50 → raw velocity 正 = 車体後退（SIGN_LEFT=-1 と整合）を確認

**Step 2: 車体フレーム変換確認**
1. 車体フレームで i_cmd_body = +0.05 A を両輪へ指令
2. raw 値変換: Goal Current raw_L = -50, raw_R = +50 であることを確認
3. 両輪が車体前進方向に回ることを確認

**Step 3: 速度ガード確認**
1. 車体フレームで正電流→正速度（加速）のとき、速度ガードが抑制すること
2. 正電流→負速度（減速）のとき、速度ガードが通過すること

---

## Bus Watchdog 復旧手順

### 通常動作
- Bus Watchdog(98) = `ceil(200/20)` = 10 (200 ms タイムアウト)
- 制御ループの通信間隔が 5 ms なので、40 回分の余裕

### Watchdog 発火時の挙動
- Bus Watchdog = -1（エラー状態）
- Goal Current/Goal Velocity レジスタが**読み取り専用**になる
- Sync Write で Goal Current = 0 を送っても**無視される**

### FAULT 復旧フロー（非 Watchdog 障害）

pitch 超過、過速度、過温度、低電圧、HW エラーなど。
Bus Watchdog はまだ正常動作中なので、Goal 書込みは受理される。

```
  1. Goal Current = 0 を Sync Write
  2. Torque Enable = 0 を左右各モータへ個別書込
  3. --- 安全確認: 以下すべて成功するまで Bus Watchdog を維持 ---
  4. 左右各モータの Goal Current を個別 Read → 0 であることを確認
  5. 左右各モータの Torque Enable を個別 Read → 0 であることを確認
  6. いずれかの readback 失敗時:
     - Bus Watchdog は有効のまま残す（最後の安全ネット）
     - "POWER CYCLE REQUIRED" をログ出力して停止
  7. 全 readback 成功後: Bus Watchdog(98) = 0
  8. 原因をシリアルログへ出力
  9. 自動再アーム禁止
```

### FAULT 復旧フロー（Watchdog 発火時）

Bus Watchdog = -1 で Goal レジスタが読み取り専用になっている。
Torque OFF を**先に**行い、その後 Watchdog をクリアする。

```
  1. Torque Enable = 0 を左右各モータへ個別書込
  2. Torque Enable を左右各モータから個別 Read → 0 であることを確認
     - いずれかの readback 失敗時:
       Bus Watchdog は有効のまま残す
       "POWER CYCLE REQUIRED: Torque OFF readback failed" をログ出力して停止
  3. Bus Watchdog(98) = 0（エラークリア）
  4. Bus Watchdog(98) を読み戻し、0 であることを確認
     - 失敗時: 電源再投入のみで復旧（ログ出力して停止）
  5. Goal Current = 0 を書く（Torque OFF 確認済みなので実害なし）
  6. Goal Current を読み戻し、0 であることを確認
  7. 原因をシリアルログへ出力
  8. 自動再アーム禁止
```

### 復旧の原則
- **非 Watchdog 障害**: Goal Current=0 → Torque OFF → Watchdog 無効化
- **Watchdog 障害**: Torque OFF → Watchdog クリア → Goal Current=0 確認
- いずれの場合も、readback 失敗時は電源再投入のみで復旧
- FAULT 状態からの自動再アームは実装しない

---

## 変更対象ファイルと概要

### 1. `include/config.h` — 定数の追加・変更

**変更点:**
- `OP_VELOCITY` 関連定数を削除、`OP_CURRENT = 0` を追加
- Goal Current アドレス (102, 2 byte)、フィードバックブロック (126, 10 byte)
- 電流制限・速度ガード・Bus Watchdog・符号規約の定数を追加
- RPM_LIMIT を削除、電流上限と速度上限に置換
- DXL_VEL_UNIT は Sync Read デコード用に維持
- Return Delay Time アドレス (9, 1 byte) を追加

### 2. `include/shared_state.h` — 状態型の拡張

- `SafetyState` 列挙型を追加
- `MotorFeedback` 構造体を追加
- `TelemetryData` に電流・電圧・温度・状態フィールドを追加

### 3. `src/main.cpp` — 初期化シーケンスの変更

リファレンス Section 7 の順序を厳守:
```
1. ポートを開く
2. 左右 PING → Model Number 1190 確認（失敗時は起動中止）
3. Torque OFF (両輪)
4. Operating Mode(11) を読み取り、0 でない場合のみ書き込む
   （EEPROM 摩耗防止 + 不要な Goal Current リセットの回避）
   読み取り失敗時は起動中止
5. Current Limit(38) を読み取り、設定値と異なる場合のみ書き込む
   （EEPROM 摩耗防止: 読み取り値が一致すれば書き込みスキップ）
   読み取り失敗時は起動中止
6. Return Delay Time(9) を読み取り、0 でない場合のみ書き込む
   読み取り失敗時は起動中止
7. 設定値を読み戻し検証（不一致時は起動中止）
8. Goal Current(102) = 0 を左右へ書く
9. Goal Current を左右各モータから個別 Read → 0 であることを確認
   失敗時: Torque ON せず起動中止
10. IMU キャリブレーション
11. Torque Enable(64) = 1
12. Goal Current = 0 再送
13. Bus Watchdog(98) を有効化
```

### 4. `src/control_task.cpp` — 制御ループの変更

#### 4a. Sync Write: Goal Current (addr 102, 2 byte, int16_t)
#### 4b. Sync Read: フィードバックブロック (addr 126, 10 byte)
- デコード: Current(int16_t) + Velocity(int32_t) + Position(int32_t)
- 左右各 `MotorFeedback` へ変換

#### 4c. 制御出力の単位変更
- PID 出力 → 電流 A (float)
- ゲインの次元: deg → A（初期値 Kp=0.1, Ki=0.0, Kd=0.0）

#### 4d. 速度保護（上記「速度ガードの符号安全設計」参照）

#### 4e. 電流飽和の優先順位
```cpp
i_common = clamp(i_common, -i_max, i_max);
yaw_headroom = max(0.0f, i_max - fabsf(i_common));
i_yaw = clamp(i_yaw, -yaw_headroom, yaw_headroom);
i_left  = i_common - i_yaw;
i_right = i_common + i_yaw;
```

#### 4f. 安全状態機械
- DISARMED → INITIALIZING → ARMED_IDLE → BALANCING → FAULT
- FAULT 条件: pitch 超過、過速度、通信エラー連続、HW エラー、過温度、低電圧
- FAULT 処理: Bus Watchdog 復旧手順を含む

#### 4g. 低速ヘルス監視（5 Hz）
- Hardware Error Status(70): 非 0 で FAULT
- Present Input Voltage(144): 範囲外で FAULT
- Present Temperature(146): 上限超過で FAULT

### 5. `src/ui_task.cpp` — テレメトリ表示の更新
- `rpm_cmd` → `current_cmd_A`
- 安全状態の表示
- テレメトリシリアル出力フォーマット更新

### 6. PID ゲインのデフォルト値
- Kp = 0.1, Ki = 0.0, Kd = 0.0（車体浮かせ試験で調整）
- DEFAULT_PITCH_TARGET は変更なし

---

## 変更しないもの

- `imu_driver.h` / `imu_driver.cpp` — IMU ドライバは変更不要
- `TairinEye.*` / `TairinMouth.*` — アバター描画は変更不要
- `platformio.ini` — ビルド設定は変更不要
- `for_arduino_ide/` — Arduino IDE 版は今回のスコープ外

---

## 安全設計

### 3 層電流制限
1. **EEPROM Current Limit(38)**: 1.0 A（ハードウェア絶対上限）
2. **ソフトウェアピーク上限**: 0.8 A（1 秒間。倒立復帰用）
3. **ソフトウェア連続上限**: 0.4 A（熱的持続可能上限）
- 1.47 A（ストール電流）や 1.75 A（レジスタ最大）を連続上限に使わない

### FAULT 処理（Bus Watchdog 復旧含む）
- 上記「Bus Watchdog 復旧手順」参照
- 自動再アーム禁止

---

## リスクと対策

| リスク | 対策 |
|--------|------|
| ゲイン次元変更で暴走 | 初期ゲイン極小値。車体浮かせ低電流試験を先行 |
| Goal Current リセット | Torque ON 前に Goal Current=0 必須送信 |
| 速度上限なし | ソフトウェア速度ガード（符号検証済み） |
| 速度ガード符号逆転 | モータフレーム加速判定 + 浮かせ試験で検証 |
| Sync Read 失敗時に速度不明 | 電流指令を 0 にフォールバック |
| 連続高電流で過熱 | 3 層制限 + 温度監視 |
| 通信断 | Bus Watchdog + ホスト側タイムアウト |
| Watchdog 後の Goal 書込不可 | Torque OFF → Watchdog クリア/readback → Goal=0/readback |
| 通信時間超過 | Return Delay=0、時間予算 23%、超過連続で FAULT |

---

## 実装順序

1. `config.h` — 定数追加
2. `shared_state.h` — 型定義追加
3. `main.cpp` — 初期化シーケンス変更
4. `control_task.cpp` — 制御ループ変更（メイン作業）
5. `ui_task.cpp` — テレメトリ更新
6. ビルド確認 (`pio run`)
