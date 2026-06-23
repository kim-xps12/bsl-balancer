# PID ベース前後操縦制御の実装計画

## Context

現在のファームウェア (`firmware/bsl-balancer`) は単一 PID ループ（ピッチ偏差 → RPM）で倒立のみを行う。ファジィ制御器の移植（`jiggly-baking-quilt.md`）は Phase A〜C の長期計画であり、PID ベースで先に前後操縦を実現することで、ファジィ移植前の段階でも PS4 ゲームパッド操縦が可能になる。

Web サーベイの結果、B-Robot (JJRobots)・Elegoo Tumbller・学術論文 (Velazquez et al.) などで実績のある **カスケード PI(速度) → PD(姿勢)** アーキテクチャを採用する。BSL-Balancer は DYNAMIXEL XL330 の Present Velocity レジスタ (addr=128) により実速度フィードバックが得られるため、エンコーダ無しの他プロジェクトより精度の高い閉ループ速度制御が可能。

---

## 制御アーキテクチャ

```
v_d (ゲームパッド or 0) ──┐
                           ├─ v_err ──→ [速度 PI (25Hz)] ──→ theta_offset [deg]
v_measured (DXL Sync Read)─┘                                   │ clamp(±5°)
                                                               ↓
                      pitch_target (86°) ──── − ──── effective_target
                                                               │
                                                               ↓
pitch_filtered (Mahony) ──────────────────────→ [姿勢 PD (200Hz)] ──→ rpm_base
pitch_rate_dps ──────────────────────────────→                          │
                                                                        ↓
                              yaw_rate_cmd ──→ [差動操舵] ──→ rpm_L, rpm_R
                                                               │
                                                         DXL setGoalVelocity
```

### 符号規約真理値表（Codex 指摘 #1 対応）

本ファームウェアの符号規約:
- `pitch_target = 86°`: 垂直姿勢
- 前傾時: `pitch_filtered` が**減少**（85°方向）
- `pitch_error = target - pitch_filtered`: 前傾で**正**
- 正の RPM → 車輪前進 → 前傾を受け止めて復帰

| 操作意図 | v_d | v_error | v_integral | theta_offset | effective_target | 物理効果 |
|----------|-----|---------|------------|--------------|------------------|----------|
| 前進加速 | +0.3 | +0.3 | 蓄積+ | +2° | 86−2 = **84°** | PIDが84°を目指す→前傾維持→前進 |
| 後退加速 | −0.3 | −0.3 | 蓄積− | −2° | 86+2 = **88°** | PIDが88°を目指す→後傾維持→後退 |
| 静止復帰 | 0 | −v | 減衰 | →0 | →86° | 元の倒立位置に復帰 |

**重要**: `effective_target = pitch_target − theta_offset`（**減算**）。正の theta_offset で target を下げることにより前傾＝前進を実現する。

### 数式

**外側ループ（速度 PI, 25Hz, back-calculation anti-windup）**:
```cpp
v_error = v_d - v_measured;

// 1. v_d 符号変化 → 積分器リセット
if ((v_d > 0 && v_d_prev < 0) || (v_d < 0 && v_d_prev > 0)) v_integral = 0;

// 2. v_d 非ゼロ→ゼロ遷移 → 積分器リセット
if (fabsf(v_d) < 1e-4f && fabsf(v_d_prev) >= 1e-4f) v_integral = 0;

// 3. 通常の積分蓄積
v_integral += v_error * dt_speed;
v_integral = clamp(v_integral, ±V_INTEGRAL_LIMIT);

// 4. 出力計算 + back-calculation anti-windup
float theta_raw = Kp_speed * v_error + Ki_speed * v_integral;
theta_offset = clamp(theta_raw, ±THETA_OFFSET_MAX);
if (theta_raw != theta_offset) {
    // 出力がクランプされた → 積分器をクランプ後の出力と整合させる
    v_integral = (theta_offset - Kp_speed * v_error) / Ki_speed;
}

// 5. 方向整合性チェック (v_error デッドバンド付き):
//    減速中 (v_error<-0.05) なのに前傾指令 (theta_offset>0) → 積分器リセット
//    デッドバンド 0.05 m/s により定常追従時のゼロ交差では非発動
constexpr float V_ERR_DEADBAND = 0.05f;  // 定常ノイズを無視
if (v_error * theta_offset < -1e-6f && fabsf(v_error) > V_ERR_DEADBAND) {
    v_integral = 0.0f;
    theta_offset = clamp(Kp_speed * v_error, ±THETA_OFFSET_MAX);
}

v_d_prev = v_d;
```

**5条件 anti-windup の効果**:
- **条件1**: v_d 符号反転 → 即リセット
- **条件2**: v_d 非ゼロ→ゼロ → 即リセット（前コマンドの積分残留を一掃。リセット後 PI ループが新たなドリフト抑制積分を蓄積）
- **条件4 (back-calc)**: 出力クランプ時に積分器をクランプ値と整合するよう逆算
- **条件5 (方向整合)**: v_error と theta_offset が逆符号（減速中なのに加速指令）→ 積分器リセット、比例項のみで出力
- ドリフトホールド: 中立維持中は v_error≈0, theta_offset≈0 で条件5は非発動、通常通り積分蓄積

**積分器リセット規則（Codex 第4回 #R6 対応、第8回 #R11 で修正）**:

以下の**異常停止イベント**で `v_integral = 0`, `theta_offset = 0` に即時リセットする。
これらは全て `SET_SPEED_ENABLED = false` コマンドとして送信し、制御タスク内で速度状態をクリアする:
- `speed_enabled` が false に遷移した時（BtnB 長押し）
- PS4 切断コールバック（`SET_SPEED_ENABLED=false` を送信）
- L1 緊急停止（`SET_SPEED_ENABLED=false` を送信）
- 転倒検知（制御タスク内で直接リセット）
- 連続 Sync Read 失敗による自動無効化（制御タスク内で直接リセット）
- **コマンド鮮度タイムアウト（R21→R24 で強化）**: 制御タスク内で最後の速度/ヨーコマンド受信からの経過時間を監視。**500ms** 以内に新しいコマンドが到着しない場合、`speed_enabled = false` に自動遷移し **v_d, yaw_rate_cmd, v_integral, theta_offset をすべてゼロ化**（UI タスク停止・BT コールバック欠落への防護）。UI タスクは **speed_enabled が true の間** 常に 50ms 周期でハートビートとして v_d + yaw_rate を送信する（PS4 接続時はスティック値、非接続時は v_d=0 / yaw=0）

**デッドゾーン・符号変化・出力飽和の anti-windup（R14→R15→R16 で収束）**:

速度 PI ループの疑似コード（上記）に全 anti-windup ロジックを統合済み:
- **条件 1**: v_d 符号変化時に v_integral リセット（反転指令への即応答）
- **条件 2**: v_d 非ゼロ→ゼロ遷移時に v_integral リセット（中立復帰の即応答）
- **条件 3**: 出力飽和時に飽和深化方向の積分を停止（clamping anti-windup）
- 中立維持中は v_d=0 を目標として PI ループがアクティブに動作し、ドリフト抑制のための積分器を新規に蓄積する

**内側ループ（姿勢 PD, 200Hz）**: 速度制御有効時は真の PD（I 項ゼロ）
```
effective_target = pitch_target − theta_offset
pitch_error = effective_target - pitch_filtered

// D_val の定義 (R23→R26 で強化): 速度制御有効時は derivative-on-measurement
// D_val = -pitch_rate_dps  (deg/s 単位、既存 Kd と整合。符号: 前傾速度で正)
// theta_offset のステップ変化による微分キックを完全に回避
// 速度制御無効時は既存の微分-on-error + LPF(α=0.2) を維持
// Kd=0 (デフォルト) では D_val の寄与はゼロ

if (speed_enabled) {
    I_acc = 0;  // 遷移時・動作中ともに I_acc を完全クリア
    rpm = Kp * P_val + Kd * D_val              // 真の PD（I 項なし）
} else {
    // speed_enabled→false 遷移時: D 状態をリセット（theta_offset ステップによる D スパイク防止）
    if (speed_was_enabled) { preP = P_val; D_filtered = 0; }
    I_acc += P_val * dt;
    I_acc = clamp(I_acc, -i_limit, i_limit);
    rpm = Kp * P_val + Ki * I_acc + Kd * D_val // 従来 PID
}
speed_was_enabled = speed_enabled;
```

**差動操舵（Phase S-B）**:
```
yaw_diff_rpm = yaw_rate_cmd * WHEEL_TRACK / (2 * WHEEL_R) * 60/(2π)
rpm_L = clamp(rpm_base - yaw_diff_rpm, ±RPM_LIMIT)
rpm_R = clamp(rpm_base + yaw_diff_rpm, ±RPM_LIMIT)
```

### 転倒検知の分離（Codex 指摘 #2 対応）

転倒検知は `effective_target` ではなく **物理的な `pitch_target`** を基準とする:

```cpp
// 転倒検知: 物理バランス点からの絶対偏差で判定
float fall_error = target - pitch_filtered;  // effective_target ではなく target
if (fall_error < -FALL_THRESHOLD_DEG || FALL_THRESHOLD_DEG < fall_error) {
    // 転倒処理: モータ停止 (Sync Write) + 速度制御状態リセット
    driveMotors(0, 0);
    I_acc = 0; theta_offset = 0; v_integral = 0;
    ...
}

// PID 制御: 速度オフセット適用後の target で計算
float pitch_error = effective_target - pitch_filtered;
```

### 初期パラメータ

| パラメータ | 値 | 根拠 |
|---|---|---|
| Kp_speed | 5.0 deg/(m/s) | 0.1m/s 偏差で即座に 0.5° 補正。比例制動で中立復帰を高速化 |
| Ki_speed | 2.0 deg/(m·s) | 0.1 m/s 誤差で 0.2 deg/s のチルト蓄積 |
| THETA_OFFSET_MAX | 5.0° | ファジィ計画の θ_ref_max ≈ 0.10 rad ≈ 5.7° に近い |
| V_INTEGRAL_LIMIT | 2.5 m·s | Ki*V_INT_LIM = 5 deg = THETA_OFFSET_MAX と一致 |
| SPEED_LOOP_DIVIDER | 8 | 200/8 = 25 Hz (Tumbller と同じ) |
| WHEEL_R | 0.029 m | 既存ファジィ計画の値 |
| V_CMD_MAX | 0.3 m/s | 既存 Phase C 仕様 |

**V_INTEGRAL_LIMIT 修正（Codex 指摘 #4 対応）**: 旧値 1.0 では Ki*limit=2° で THETA_OFFSET_MAX=5° に到達不可能。2.5 に修正し Ki*2.5=5°=THETA_OFFSET_MAX と整合。

### ネスト積分器の回避（Codex 指摘 #4 対応、再レビュー #3 で強化）

B-Robot は内側ループを PD（積分なし）、外側を PI とする。本計画でも速度制御有効時に I_acc を**ゼロクリア**し、PID 出力から Ki*I_acc 項を**除外**する（真の PD）。旧値の残留バイアスは発生しない。速度制御無効時は従来通り Ki=0.05 の PID が動作し、I_acc はゼロからの蓄積を再開する。

---

## Phase S-A: 速度フィードバック + 速度 PI ループ

### 変更ファイル

1. **`include/config.h`** — 速度制御定数追加 (WHEEL_R, Kp_speed, Ki_speed, THETA_OFFSET_MAX 等)
2. **`include/shared_state.h`** — CtrlCommandType に `SET_SPEED_ENABLED`, `SET_VELOCITY_CMD` 等追加。TelemetryData に `v_measured`, `theta_offset`, `sync_read_us` 等追加
3. **`src/control_task.cpp`** — DXL Sync Read 関数 `readWheelVelocity()` 追加。ループ内に速度 PI 計算を追加（25Hz 分周）。`effective_target = target - theta_offset` で既存 PID に接続。転倒検知を物理 target 基準に分離。モータ書込を Sync Write に移行
4. **`src/ui_task.cpp`** — テレメトリ出力に `>v:`, `>theta_offset:` 等追加。BtnB 長押しで速度制御 ON/OFF トグル。**speed_enabled 中は 50ms 周期で v_d=0 / yaw=0 のハートビートコマンドを送信**（500ms デッドマンタイムアウト対応）

### DXL Sync Read タイムアウト設計（Codex 指摘 #3 対応、再レビュー #1 で強化）

Dynamixel2Arduino 0.8.1 の `syncRead(InfoSyncReadInst_t*, uint32_t timeout_ms)` は内部で各モータの Status Packet を順次待受けし、**各パケットに対して timeout_ms が適用される**。2 モータ構成では最悪 `2 × timeout_ms` の遅延が発生しうる。

**注意（Codex 第4回レビュー #2 対応）**: API は `millis()` ベースのタイムアウトを使用するため、`timeout_ms=1` では tick 境界で即座に期限切れとなり正常パケットをドロップする。**最低 2ms が必要**。

**タイムアウトを 2ms に設定**する:

```cpp
uint8_t recv_count = dxl.syncRead(&vel_sync_read, 2);  // 2ms timeout per status packet
```

- 正常時: 1Mbps で 4byte Status Packet ≈ 200μs × 2 = ~400μs
- 最悪時: 2ms × 2 = 4ms (両方タイムアウト)

失敗パス処理:
- `recv_count < 2` (一方または両方未応答): `readWheelVelocity()` は NAN を返す
- NAN 時: 前回の `v_measured` を保持（ドロップアウト保護）
- 連続 N 回 (N=5, 200ms 相当) 失敗時: `speed_enabled` を自動無効化し、`v_integral` と `theta_offset` をリセット。テレメトリに警告出力
- 最悪ケース: 4ms (2×2ms timeout) + 処理 5μs = ~4.0ms

### モータ書込の非ブロック化（Codex #R8 対応、第6回で #R9 修正）

既存の `setGoalVelocity()` は内部で `rxStatusPacket()` を**無条件に**呼び出す。SRL=1 に設定しても、ライブラリがパケットを待つ動作は変わらず 100ms/ID のタイムアウトが発生する（Codex 第6回で確認済み）。

**対策**: `driveTire()` を **Sync Write** に完全置換する。Protocol 2.0 の Sync Write はステータス応答を要求しないため、ブロッキングが発生しない:

```cpp
// 静的バッファ（一度だけ初期化）
static uint8_t goal_vel_buf_L[4];
static uint8_t goal_vel_buf_R[4];
static DYNAMIXEL::XELInfoSyncWrite_t vel_write_xels[2] = {
    { goal_vel_buf_L, DXL_ID_L },
    { goal_vel_buf_R, DXL_ID_R },
};
static DYNAMIXEL::InfoSyncWriteInst_t vel_sync_write = {
    104,    // Goal Velocity address (XL330)
    4,      // data length
    vel_write_xels,
    2,      // xel_count
    true,   // is_info_changed
    { nullptr, 0, 0, false }
};

static void driveMotors(int rpm_L, int rpm_R) {
    // RPM → raw velocity value (1 raw = 0.229 RPM)
    int32_t raw_L = (int32_t)(-rpm_L / 0.229f);  // 左モータ反転
    int32_t raw_R = (int32_t)( rpm_R / 0.229f);
    memcpy(goal_vel_buf_L, &raw_L, 4);
    memcpy(goal_vel_buf_R, &raw_R, 4);
    vel_sync_write.is_info_changed = true;  // バッファ更新をライブラリに通知（パケット再生成強制）
    dxl.syncWrite(&vel_sync_write);         // 応答待ちなし
}
```

既存の `driveTire(int rpm)` を `driveMotors(int rpm_L, int rpm_R)` に置換。差動操舵 (Phase S-B) にも対応する引数構成。

### タイミングバジェット

| 処理 | 所要時間 | 備考 |
|------|---------|------|
| 既存処理 (IMU+Mahony+PID) | ~330 μs | |
| DXL Sync Write (Goal Velocity) | ~80 μs | 1 パケット送信、応答待ちなし |
| DXL Sync Read (2ms/packet) | ~400 μs 正常 / 4000 μs 最悪 | 8 サイクルに 1 回 |
| 速度 PI 計算 | ~5 μs | 8 サイクルに 1 回 |
| テレメトリ | ~5 μs | |
| **正常ケース合計** | **~820 μs** | **余裕 84%** |
| **最悪ケース合計** | **~4420 μs** | **余裕 12%** (両モータ Sync Read タイムアウト時) |

**I2C 競合との複合最悪ケース（R25 対応）**: IMU の I2C mutex 待ちが最大 2ms 発生しうるため、DXL Sync Read 4ms と重なると合計 6ms > 5ms 周期を超過する。対策として **予算ガード** を導入する:

```cpp
// 速度ループ実行サイクルで、IMU 読取後の経過時間を確認
int64_t budget_us = esp_timer_get_time() - now_us;
if (speed_divider_count >= SPEED_LOOP_DIVIDER && budget_us < 600) {
    // 経過 600μs 未満 → 残り 4400μs 以上: Sync Read 最悪 4ms + 後処理 200μs に余裕
    readWheelVelocity();
} else if (speed_divider_count >= SPEED_LOOP_DIVIDER) {
    // 経過 600μs 以上: Sync Read + 後処理で 5ms を超過する恐れ → スキップ
    // v_measured は前回値を保持
}
```

**予算ガードの閾値根拠**: Sync Read 最悪 4000μs + PID/Sync Write/テレメトリ ~200μs = 4200μs。600 + 4200 = 4800μs ≤ 5000μs 周期。閾値 600μs は acceptance 基準 (loop_us ≤ 4800μs) と整合する。

**Phase S-A 検証で実測必須**: Sync Read の所要時間を **専用の `sync_read_us` テレメトリ** (`esp_timer_get_time()` で Sync Read 前後を計測) で個別に計測する。`loop_us` は全体の制御周期計測として別途監視する。

合格基準（R22 対応: 最悪ケースも明示的 pass/fail）:
- `sync_read_us`: 正常時 ≤ 1500μs / 片方タイムアウト時 ≤ 2500μs / 両方タイムアウト時 ≤ 4200μs
  - 注: XL330 の Return Delay Time デフォルト = 250 (500μs/パケット)。`setup()` で Return Delay Time を 0 に設定するか、正常時閾値を 1500μs に引き上げる。本計画では閾値引き上げを採用
- `loop_us` (速度ループ実行サイクル): 正常時 ≤ 2200μs (sync_read 1500 + 制御 330 + SW/PI/tel 370) / **最悪ケース ≤ 4800μs**
- `loop_us` (非速度ループサイクル): ≤ 800μs (既存ベースライン)
- **最悪ケース `loop_us` > 4800μs が正常動作中に発生**: 予算ガードの閾値を引き下げて Sync Read スキップを積極化する。それでも解決しない場合は Sync Read を制御ループ外（別タスク or DMA）に移動する設計変更を実施

### 速度符号変換の詳細（Codex 再レビュー #2 対応）

`readWheelVelocity()` の完全な変換式:

```cpp
// DXL Present Velocity: int32_t, 単位 0.229 RPM/raw
// uint32_t 経由で合成し、未定義動作を回避 (buf[3]>=0x80 時の signed overflow 防止)
static inline int32_t decodeInt32(const uint8_t* buf) {
    uint32_t u = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8)
              | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    int32_t s;
    memcpy(&s, &u, sizeof(s));
    return s;
}
int32_t raw_L = decodeInt32(buf_L);
int32_t raw_R = decodeInt32(buf_R);

// 符号補正: driveTire() は左モータに -rpm を送信するため、
// 前進時の Present Velocity は raw_L < 0, raw_R > 0
// 車体速度 = (-raw_L + raw_R) / 2 → 前進時に正
float avg_raw = (float)(-raw_L + raw_R) * 0.5f;

// raw → RPM → rad/s → m/s
float avg_rpm = avg_raw * DXL_VEL_UNIT;            // × 0.229 RPM/raw
float avg_omega = avg_rpm * (2.0f * M_PI / 60.0f); // RPM → rad/s
float v_measured = avg_omega * WHEEL_R;             // rad/s → m/s (× 0.029)
```

**符号検証アサーション** (デバッグ用、リリース時に削除):
```cpp
// 前進駆動中 (rpm_motor > 10): raw_L は負、raw_R は正であること
if (rpm_motor > 10) {
    assert(raw_L < 0 && raw_R > 0);  // 符号不一致なら即停止
}
```

### 検証手順

1. `pio run` ビルド成功
2. **符号検証デスクテスト**（手持ち、速度制御 OFF）:
   - 車輪を手で前方に回す → `v_measured > 0` 確認
   - 車輪を手で後方に回す → `v_measured < 0` 確認
   - `vel_L` (raw) が前進時に負、`vel_R` が正であること
3. **Sync Read タイムアウトテスト**: モータ電源 OFF で `readWheelVelocity()` が NAN を返し、`sync_read_us` ≤ 4500μs であること。連続 5 回失敗で速度制御が自動無効化されること
4. 倒立テスト（速度制御 OFF）: 既存動作の退行なし確認
5. **速度制御符号検証**（手持ち、速度制御 ON）:
   - 手で車輪を前方に回す → `theta_offset < 0` (ブレーキ方向) 確認
   - テレメトリで `effective_target > pitch_target` (後傾方向) 確認
6. 倒立テスト（速度制御 ON, v_d=0）: ドリフト抑制効果の確認
7. **転倒検知テスト**: theta_offset=±5° 状態でロボットを傾けても、物理 target 基準 ±40° で正しく転倒検知すること
8. **中立復帰テスト（走行中）**: 最大前進 (v_d=0.3) で数秒走行後にスティック中立 → `theta_offset` が 1 秒以内にブレーキ方向（負）に反転すること
9. **中立復帰テスト（拘束中）**: ロボットを手で固定し v_d=0.3 を数秒指令（v_measured≈0）後にスティック中立 → `theta_offset` が即座に≈0 になること（前傾発進しないこと）
10. **減速テスト**: v_d=0.3 で走行中に v_d=0.1 に減速 → `theta_offset` が 1 秒以内に減少方向に変化すること
11. **反転テスト**: v_d=0.3 で走行中に v_d=-0.3 に反転 → `theta_offset` が即座にブレーキ/後退方向に反転すること

---

## Phase S-B: PS4 ゲームパッド統合

### 変更ファイル

1. **`platformio.ini`** — `PS4_Controller_Host@v1.1.0` 追加
2. **`include/config.h`** — YAW_RATE_MAX, WHEEL_TRACK, PS4_DEADZONE, PS4_MAC_ADDRESS 追加
3. **`src/main.cpp`** — `PS4.begin()` + 接続/切断コールバック登録
4. **`src/ui_task.cpp`** — PS4 スティック読取 → CtrlCommand キュー送信
5. **`src/control_task.cpp`** — `driveMotors()` に差動操舵ロジックを統合（Phase S-A で導入済みの Sync Write ヘルパーを使用）

### スティックマッピング

| スティック | 軸 | 出力 | 範囲 | デッドゾーン |
|---|---|---|---|---|
| 左 Y | 前後 | v_d [m/s] | ±0.3 | 13 (~10%) |
| 右 X | 旋回 | yaw_rate [rad/s] | ±1.0 | 13 (~10%) |

### 安全機構

- **切断時**: コールバックで `SET_SPEED_ENABLED=false` + `SET_YAW_RATE_CMD=0` 送信 → 速度状態リセット → 静止バランスに復帰
- **L1 ボタン**: `SET_SPEED_ENABLED=false` + `SET_YAW_RATE_CMD=0` 送信（速度状態を完全リセット）
- **LCD 表示**: 接続状態表示

### 検証手順

1. SixaxisPairTool でペアリング → LCD に接続表示
2. 左スティック前倒し → ロボット前進
3. 右スティック左右 → その場旋回
4. ニュートラル → 静止バランス復帰
5. 切断 → 2 秒以内に停止
6. L1 → 即時停止

### 事前確認事項

- `WHEEL_TRACK`（左右車輪間距離）を CAD または実機から計測して `config.h` に設定する
- `PS4_MAC_ADDRESS` を ESP32 の実 MAC アドレスに設定する

---

## リスク分析

| リスク | 確率 | 対策 |
|--------|------|------|
| 速度フィードバック符号ミス → 即転倒 | 中 | 符号真理値表 + デスクテスト 3 段階検証 |
| 速度制御が姿勢制御を不安定化 | 中 | 内側 I 凍結 (PD化) + THETA_OFFSET_MAX=5° + Ki-only 開始 |
| Sync Read タイムアウト超過 | 中 | 明示 2ms timeout + 連続失敗 5 回で自動無効化 |
| 転倒検知閾値の速度オフセットによるドリフト | — | 転倒検知を物理 target 基準に分離（解決済み） |
| ネスト積分器による不安定化 | — | 速度制御有効時に内側 I_acc 凍結（解決済み） |
| BT が制御ループに干渉 | 低 | BT は Core 0 (UI)、制御は Core 1。キュー経由で疎結合 |
| 積分巻き上げによる振動 | 中 | V_INTEGRAL_LIMIT + 転倒時リセット + 無効化時リセット |

---

## ブランチ戦略

- `feat/fw-speed-phase-a` → `master` へ PR (firmware/bsl-balancer リポジトリ)
- `feat/fw-speed-phase-b` → `master` へ PR (Phase S-A マージ後)

## ファジィ計画との関係

- Phase S-A の `readWheelVelocity()` は ファジィ Phase B の Sync Read と同一
- Phase S-B の PS4 統合はファジィ Phase C と同一（差動操舵のベース関数が PID 出力かファジィ出力かの違いのみ）
- ファジィ移植時にこれらのインフラをそのまま再利用可能

---

## Codex Adversarial Review 対応履歴

### 初回レビュー

| # | 指摘 | 重要度 | 対応 |
|---|------|--------|------|
| 1 | 符号極性の誤り (`target + offset` で後傾指令) | HIGH | `effective_target = target − theta_offset` に修正。符号真理値表を追加 |
| 2 | 転倒検知が effective_target 相対でドリフト | HIGH | 転倒検知を物理 `target` 基準に分離。PID 計算のみ `effective_target` 使用 |
| 3 | Sync Read タイムアウト未指定 (API デフォルト 10ms) | HIGH | 明示 timeout 指定。連続 5 回失敗で自動無効化。最悪ケースバジェット追加 |
| 4 | ネスト積分器 (内側 Ki + 外側 Ki) | MEDIUM | 速度制御有効時に内側 I_acc 凍結→PD化。V_INTEGRAL_LIMIT を 2.5 に修正 |

### 再レビュー

| # | 指摘 | 重要度 | 対応 |
|---|------|--------|------|
| R1 | Sync Read per-motor timeout で 2×2ms=4ms の可能性 | HIGH | timeout を 1ms/packet に変更。最悪 2ms。実測検証ステップ追加 |
| R2 | raw→v_measured の符号変換式が Phase S-A に不足 | HIGH | 完全な変換式とアサーションを Phase S-A に明記 |
| R3 | 凍結 I_acc が RPM バイアスとして残留 | MEDIUM | I_acc をゼロクリア + Ki 項を出力から除外（真の PD）に変更 |

### 第3回レビュー

| # | 指摘 | 重要度 | 対応 |
|---|------|--------|------|
| R4 | buf[3]<<24 で signed overflow (未定義動作) | HIGH | uint32_t 経由合成 + memcpy で int32_t 変換。decodeInt32 ヘルパー追加 |
| R5 | loop_us で Sync Read 単体を検証できない | MEDIUM | 専用 `sync_read_us` テレメトリ追加。閾値を個別に設定 |

### 第4回レビュー

| # | 指摘 | 重要度 | 対応 |
|---|------|--------|------|
| R6 | v_d=0 時に v_integral が残留し前傾維持 | HIGH | 全停止パス（disable/disconnect/L1/転倒/失敗/deadzone）で v_integral+theta_offset を即時リセット |
| R7 | millis() ベース API で 1ms timeout は tick 境界で即期限切れ | HIGH | timeout を 2ms に変更。最悪 4ms。vTaskDelayUntil の自動回復で許容。連続失敗で自動無効化 |

### 第5回レビュー

| # | 指摘 | 重要度 | 対応 |
|---|------|--------|------|
| R8 | setGoalVelocity が 100ms/ID の応答待ちでブロック | HIGH | モータ書込を Sync Write に移行（応答不要） |

### 第6回レビュー

| # | 指摘 | 重要度 | 対応 |
|---|------|--------|------|
| R9 | SRL=1 でも setGoalVelocity は rxStatusPacket を無条件呼出し | HIGH | SRL 方式を撤回。dxl.syncWrite() による完全 Sync Write 移行に変更。driveMotors() ヘルパー実装 |

### 第7回レビュー

| # | 指摘 | 重要度 | 対応 |
|---|------|--------|------|
| R10 | is_info_changed が初回後 false → 古いパケット再送 | HIGH | driveMotors() で毎回 `is_info_changed = true` を設定してパケット再生成を強制 |

### 第8回レビュー

| # | 指摘 | 重要度 | 対応 |
|---|------|--------|------|
| R11 | デッドゾーン進入時の積分器リセットがゼロ速度ホールドを無効化 | HIGH | デッドゾーンでは v_d=0 のみ設定し積分器は維持。リセットは異常停止イベントに限定 |

### 第9回レビュー

| # | 指摘 | 重要度 | 対応 |
|---|------|--------|------|
| R12 | 飽和積分器の中立復帰に最大25秒（Kp_speed=0 で比例制動なし） | HIGH | Kp_speed を 0.0 → 5.0 に変更。比例項で即座にブレーキ方向補正 |
| R13 | 切断/L1 が v_d=0 送信だけでは積分器リセットされない | HIGH | 切断/L1 では SET_SPEED_ENABLED=false を送信。制御タスクで速度状態を完全クリア |

### 第10回レビュー

| # | 指摘 | 重要度 | 対応 |
|---|------|--------|------|
| R14 | 飽和積分器が比例制動を打ち消し、中立でも前傾維持 | HIGH | →R15 で再修正 |

### 第11回レビュー

| # | 指摘 | 重要度 | 対応 |
|---|------|--------|------|
| R15 | v_measured≈0 (停止/拘束) 時に積分器が残留し中立で発進 | HIGH | →R16 に統合 |

### 第12回レビュー

| # | 指摘 | 重要度 | 対応 |
|---|------|--------|------|
| R16 | 飽和積分器が減速・反転指令に追従しない | HIGH | →R17 で back-calculation 方式に置換 |

### 第13回レビュー

| # | 指摘 | 重要度 | 対応 |
|---|------|--------|------|
| R17 | 同符号減速時にクランプ anti-windup が積分器を巻き戻せない | HIGH | back-calculation + 方向整合性チェック（条件5）で統合解決 |

### 第14回レビュー

| # | 指摘 | 重要度 | 対応 |
|---|------|--------|------|
| R18 | 同符号減速で非飽和化すると back-calculation が発動しない | HIGH | 条件5追加: v_error と theta_offset が逆符号なら v_integral=0 にリセットし比例項のみで出力 |

### 第15回レビュー

| # | 指摘 | 重要度 | 対応 |
|---|------|--------|------|
| R19 | ソースコード未実装（計画フェーズなので想定通り） | HIGH | プラン承認後に Phase S-A で実装予定。プランレビューとしては非該当 |
| R20 | 条件5が定常追従中のゼロ交差でリセット発動し PI 収束を妨げる | MEDIUM | V_ERR_DEADBAND=0.05 m/s を追加。定常ノイズでは非発動、明確な減速時のみ発動 |

### 第16回レビュー (計画文書専用)

| # | 指摘 | 重要度 | 対応 |
|---|------|--------|------|
| R21 | UIタスク停止時にコマンドが陳腐化し無制御走行 | HIGH | コマンド鮮度タイムアウト 500ms 追加。超過で speed_enabled=false 自動遷移 |
| R22 | 最悪ケース loop_us が pass/fail 基準に含まれない | HIGH | 最悪ケース loop_us ≤ 4800μs を明示的合格基準に追加 |
| R23 | 内側 D 項 (D_val) の定義・符号・フィルタリングが未記載 | MEDIUM | →R26 で derivative-on-measurement に変更 |

### 第17回レビュー (計画文書専用 #2)

| # | 指摘 | 重要度 | 対応 |
|---|------|--------|------|
| R24 | デッドマンが velocity のみで yaw を放置 | HIGH | デッドマンを全駆動コマンド (v_d+yaw) に拡大。UIタスクは50ms周期でハートビート送信 |
| R25 | I2C mutex 2ms + DXL timeout 4ms = 6ms > 5ms 周期 | HIGH | 予算ガード追加。IMU後の残り予算が2.5ms未満ならSyncReadをスキップ |
| R26 | 微分-on-error が theta_offset ステップでキック | MEDIUM | 速度制御有効時は derivative-on-measurement (-pitch_rate_dps) に変更 |
