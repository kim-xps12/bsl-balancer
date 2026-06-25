# XL330-M077 Current Control Mode リファレンス

> **対象:** `firmware/bsl-balancer/` の実機制御コードで、XL330-M077 を Current Control Mode 専用で扱うための実装メモ
> **抽出元:** `docs/XL330_M077_TWIP_coding_agent_reference.md`
> **参照時点:** 2026-06-22

---

## 1. 移行時の基本判断

実機の XL330-M077 は Current Control Mode で使う。

```text
Operating Mode(11) = 0
```

倒立制御器が外部で車輪駆動入力を計算するため、DYNAMIXEL 内部の速度 PI 制御器とプロファイル生成器をプラントへ含めない。実機と MuJoCo の共通入力は「電流指令」とし、制御コードの I/O 境界は次の形に揃える。

```cpp
write_current_A(left_current_A, right_current_A)
```

制御器内部の単位は SI 単位へ統一する。

| 物理量 | 内部単位 |
|---|---|
| 電流 | A |
| 角度 | rad |
| 角速度 | rad/s |
| 距離 | m |
| トルク | N m |

未確定値をコードへ推測で埋め込まない。DYNAMIXEL ID、通信速度、車輪半径、トレッド、IMU 軸、制御周期、安全電流、速度上限、温度上限、電圧範囲は設定値として扱う。

---

## 2. XL330-M077 の実装で使う確定仕様

5 V 使用時の主要仕様は次の通り。

| 項目 | 値 |
|---|---:|
| 推奨電源電圧 | 5.0 V |
| 使用可能電圧 | 3.7-6.0 V |
| 物理的な減速比 | 77.5:1 |
| ストールトルク | 0.215 N m |
| ストール電流 | 1.47 A |
| ストール点から算出した比 | 約 0.1463 N m/A |
| 無負荷速度 | 383 rpm |
| 無負荷角速度 | 約 40.11 rad/s |
| 通信 | TTL, DYNAMIXEL Protocol 2.0 |
| Model Number | 1190 |

実装上の注意:

- 0.215 N m は 5 V 時のストールトルクであり、連続定格トルクではない。
- `0.215 / 1.47 ~= 0.1463 N m/A` はストール点の比であり、全動作域で一定のトルク定数として保証されない。
- XL330 の `Present Current` はモータ相電流ではなく、DYNAMIXEL の入力電源側で測定された電流である。
- 物理減速比 77.5 を MuJoCo の出力軸車輪関節へ重ねて掛けない。MuJoCo の `gear=0.05239` はシステム同定で得た実効入力ゲインであり、物理減速比とは別物である。

---

## 3. 制御テーブルと単位変換

ファームウェア更新で直接使う XL330-M077 の制御テーブル項目。

| アドレス | サイズ | 項目 | 単位・用途 |
|---:|---:|---|---|
| 11 | 1 byte | Operating Mode | `0`: Current Control |
| 38 | 2 byte | Current Limit | 約 1 mA/count, EEPROM |
| 44 | 4 byte | Velocity Limit | Goal Velocity 上限。Current Control の速度保護ではない |
| 63 | 1 byte | Shutdown | ハードウェアエラー時の停止条件 |
| 64 | 1 byte | Torque Enable | `0`: OFF, `1`: ON |
| 70 | 1 byte | Hardware Error Status | ハードウェア異常 |
| 98 | 1 byte | Bus Watchdog | 20 ms/count |
| 102 | 2 byte | Goal Current | 約 1 mA/count, 符号付き |
| 126 | 2 byte | Present Current | 約 1 mA/count, 符号付き |
| 128 | 4 byte | Present Velocity | 0.229 rpm/count, 符号付き |
| 132 | 4 byte | Present Position | 1 pulse/count, 4096 pulse/rev |
| 144 | 2 byte | Present Input Voltage | 0.1 V/count |
| 146 | 1 byte | Present Temperature | 1 degC/count |

複数バイト値はリトルエンディアン、負値は 2 の補数で扱う。

推奨変換式:

```cpp
constexpr float CURRENT_UNIT_A = 0.001f;
constexpr float VELOCITY_UNIT_RPM = 0.229f;
constexpr float POSITION_COUNTS_PER_REV = 4096.0f;

float currentRawToA(int16_t raw) {
    return static_cast<float>(raw) * CURRENT_UNIT_A;
}

int16_t currentAToRaw(float current_A) {
    return static_cast<int16_t>(lroundf(current_A / CURRENT_UNIT_A));
}

float velocityRawToRadS(int32_t raw) {
    const float rpm = static_cast<float>(raw) * VELOCITY_UNIT_RPM;
    return rpm * 2.0f * PI / 60.0f;
}

float positionRawToRad(int32_t raw) {
    return static_cast<float>(raw) * 2.0f * PI / POSITION_COUNTS_PER_REV;
}
```

`Goal Current` へ書く前に、必ずソフトウェア上限と `Current Limit(38)` の小さい方で飽和させる。

```cpp
float i_limit_A = min(command_current_limit_A, eeprom_current_limit_A);
i_cmd_A = constrain(i_cmd_A, -i_limit_A, i_limit_A);
int16_t goal_current_raw = currentAToRaw(i_cmd_A);
```

---

## 4. 実機初期化シーケンス

初期化はこの順序を維持する。

```text
1. 通信ポートを開く
2. 左右の ID へ PING し、Model Number 1190 を確認する
3. 左右とも Torque Enable(64) = 0
4. Operating Mode(11) = 0
5. Current Limit(38) を設定する
6. 必要に応じて Return Delay Time、Shutdown 等を設定する
7. 設定値を読み戻して検証する
8. Goal Current(102) = 0 を左右へ書く
9. センサと制御ループを開始できる状態にする
10. Torque Enable(64) = 1
11. Goal Current = 0 を再送する
12. Bus Watchdog(98) を有効化し、周期通信を開始する
```

危険事項:

- `Operating Mode` 変更後、DYNAMIXEL は `Goal Current` を `Current Limit` の値へリセットする。
- そのため、モード変更後かつ Torque ON 前に必ず `Goal Current=0` を書く。
- EEPROM 領域は Torque ON 中に書き換えられない。`Operating Mode` と `Current Limit` は Torque OFF で設定する。

Bus Watchdog の raw 値は 20 ms/count。

```cpp
uint8_t watchdogRawFromMs(float timeout_ms) {
    int raw = static_cast<int>(ceilf(timeout_ms / 20.0f));
    return static_cast<uint8_t>(constrain(raw, 1, 127));
}
```

通信間隔が設定値を超えると DYNAMIXEL は停止し、Bus Watchdog はエラー値 `-1` になる。この状態では Goal 値が読み取り専用になる。復旧時は Bus Watchdog へ `0` を書いてエラーを解除し、`Goal Current=0` を確認してから再度有効化する。

---

## 5. 通信ループ

高速ループでは左右モータへの `Goal Current(102)` 2 byte を Group Sync Write 相当で同時送信する。片輪だけ更新された指令状態を作らない。

高速フィードバックはアドレス 126 から 10 byte をまとめて読む。

```text
126-127: Present Current
128-131: Present Velocity
132-135: Present Position
```

これにより、左右モータの電流、速度、位置を連続ブロックとして取得する。

制御ループより低い周期でよい監視項目:

- `Hardware Error Status(70)`
- `Present Input Voltage(144)`
- `Present Temperature(146)`

ただし、異常検出時は即座に安全状態へ遷移する。

受信値にはホスト側の単調時刻を付ける。センサ値が古い、片輪だけ更新されない、期限を超過した場合は制御計算を継続せず FAULT へ遷移する。

```cpp
struct MotorSample {
    float host_monotonic_s;
    float current_A;
    float velocity_rad_s;
    float position_rad;
    bool valid;
};
```

---

## 6. 座標系・符号規約

次の規約へ正規化してから制御器へ渡す。

```text
x: 車体前方が正
z: 上向きが正
pitch theta: 車体上部が前方へ倒れる向きが正
yaw psi: 上から見て反時計回りが正
phiL, phiR: 正方向で車体を前進させる車輪角
IL, IR: 正方向で各車輪を前進側へ駆動する電流
```

実際の DYNAMIXEL 取付方向、IMU 軸、MuJoCo 関節軸は一致しない可能性がある。DYNAMIXEL raw 値は I/O 層で符号係数を掛け、制御器内部では上記の車体座標系だけを扱う。

差動二輪の運動学:

```text
x     = r / 2 * (phiR + phiL)
psi   = r / W * (phiR - phiL)
x_dot = r / 2 * (phiR_dot + phiL_dot)
psi_dot = r / W * (phiR_dot - phiL_dot)
```

`Present Position` は電源投入、再起動、モード変更を跨いで連続であると仮定しない。起動時の値を原点として相対位置を計算し、不連続を検出した場合はオドメトリを再初期化する。

---

## 7. 電流指令ミキサ

平衡制御と yaw 制御を分ける場合、内部では共通モード電流と差動電流を計算し、左右電流へ変換する。

```text
s_balance in {+1, -1}
s_yaw     in {+1, -1}

I_common = s_balance * K_balance * [theta, theta_dot, x - x_ref, x_dot - v_ref]^T
I_yaw    = s_yaw     * K_yaw     * [psi - psi_ref, psi_dot - psi_dot_ref]^T

I_L = I_common - I_yaw
I_R = I_common + I_yaw
```

符号は実機の正方向試験で確定する。最低限、「両輪正電流で前進」「差動電流で期待する yaw 方向」を車体を浮かせた低電流試験で確認する。

飽和時は倒立制御を優先する。左右を個別に単純クリップすると、共通モードと差動モードの比率が崩れるため、先に共通モードを制限し、残った電流余裕を yaw 制御へ割り当てる。

```cpp
i_common = constrain(i_common, -i_max, i_max);
float yaw_headroom = max(0.0f, i_max - fabsf(i_common));
i_yaw = constrain(i_yaw, -yaw_headroom, yaw_headroom);

float i_left = i_common - i_yaw;
float i_right = i_common + i_yaw;
```

積分器を使う場合は、出力飽和、過速度制限、FAULT 遷移に連動したアンチワインドアップを入れる。転倒中や DISARMED 中に積分状態を保持しない。

---

## 7.1. Current Control Mode 向け PID 倒立制御器設計

### Web サーベイから採る設計判断

調査対象は 2026-06-23 時点で確認できる一次情報と論文に限定する。

| 参照 | この設計へ採用する点 | 採用しない点 |
|---|---|---|
| ROBOTIS XL330-M077 e-Manual | `Operating Mode(11)=0` は DYNAMIXEL 内部で速度・位置を制御せず、外部制御器が電流/トルク相当の入力を与えるモードとして扱う。`Goal Current(102)` は `Current Limit(38)` を超えられず、XL330 の `Present Current` は入力電源側電流である。長時間高電流は禁止する。 | `Goal Current` を理想的な出力軸トルク指令、`Present Current` を相電流フィードバックとして扱わない。 |
| Rahman et al., "Comparison of Different Control Theories on a Two Wheeled Self Balancing Robot", arXiv:1807.08243 | 二輪倒立は pitch を主観測量にした PID/LQR/Fuzzy の比較対象であり、PID は実装容易なベースラインとして成立する。ただし論文のゲイン値は ROS/Gazebo モデル固有であり、本機へ数値移植しない。 | 論文中の PID ゲインや応答曲線を XL330 電流指令のゲインとして流用しない。 |
| Music et al., "Design of a Networked Controller for a Two-Wheeled Inverted Pendulum Robot", arXiv:1812.03071 | WIP は通信遅延・ジッタが安定性へ直接効く。倒立の内側ループは ESP32 上のローカル 200 Hz で閉じ、ネットワーク/ゲームパッド入力は外側の低速目標だけに使う。 | 無線や UI タスク経由で姿勢ループを閉じない。 |
| Caparroz et al., "Anti-Windup in PID Control", arXiv:2606.01959 | アクチュエータ飽和は windup によって overshoot、復帰遅れ、不安定化を起こす。電流制限・速度ガード・転倒検知を PID 積分器へ必ず結合する。 | 積分器を単純に `error * dt` で蓄積し続けない。 |
| Torres-Garcia and Michiels, "PID controllers and low-pass filters for time-delay systems", arXiv:2604.16124 | 微分項のローパスフィルタは後付け定数ではなく閉ループ安定性の一部として扱う。IMU ノイズ、通信遅延、制御周期から `D` フィルタを調整する。 | 生の差分微分をそのまま電流指令へ入れない。 |

参照 URL:

- ROBOTIS XL330-M077 e-Manual: https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/
- Rahman et al. 2018: https://arxiv.org/abs/1807.08243
- Music et al. 2019: https://arxiv.org/abs/1812.03071
- Caparroz et al. 2026: https://arxiv.org/abs/2606.01959
- Torres-Garcia and Michiels 2026: https://arxiv.org/abs/2604.16124

結論: XL330 Current Control Mode では、公開例に多い「PID 出力を PWM/RPM へ渡す」構成をそのまま使わない。内側の姿勢 PID/PD は電流[A]を直接出力し、速度・位置・yaw は外側から姿勢目標または差動電流として注入する。電流飽和と通信期限切れを前提にした離散時間制御器として実装する。

### 制御器の構成

倒立だけを成立させる最小構成は、姿勢 PD を主系とし、積分は最後に足す。

```text
theta_ref ─┐
           ├─ e_theta ─→ [姿勢 PID/PD 200 Hz] ─→ I_common_unsat [A]
theta   ───┘                                     │
theta_dot ───────────────────────────────────────┘

v_ref ─────┐
           ├─ e_v ─→ [速度 PI 20-50 Hz] ─→ theta_ref_offset [rad]
v_meas ────┘

yaw_ref ───┐
           ├─ e_yaw ─→ [yaw PD/PI 20-50 Hz] ─→ I_yaw_unsat [A]
yaw_meas ──┘

I_L = I_common - I_yaw
I_R = I_common + I_yaw
```

内側姿勢ループ:

```text
theta      : 車体上部が前方へ倒れる向きが正 [rad]
theta_ref  : 倒立目標。静止時は 0 [rad]
theta_dot  : theta の時間微分 [rad/s]
I_common   : 両輪を同方向へ駆動する共通モード電流 [A]

e_theta = theta - theta_ref
z_theta = integral(e_theta)

I_common_unsat =
    Kp_theta * e_theta
  + Kd_theta * theta_dot_filtered
  + Ki_theta * z_theta
  + Kx       * (x - x_ref)
  + Kv       * (x_dot - v_ref)
```

符号はこのリファレンスの座標系に合わせる。`theta > 0` は前方へ倒れる状態であり、両輪へ正電流を与えると車体が前進する向きに揃える。したがって初期実装では `Kp_theta > 0`, `Kd_theta > 0` を期待する。ただし、IMU 軸または DYNAMIXEL 取付方向が逆なら符号は破綻するため、実機符号試験に合格するまで `BALANCING` を許可しない。

`Kx` と `Kv` は「位置を戻すための弱いバイアス」であり、最初の倒立成立前は 0 にする。速度操縦を使う場合は、内側姿勢ループへ直接速度誤差を足さず、外側速度 PI で `theta_ref` を変える。

外側速度 PI:

```text
e_v = v_ref - v_meas
z_v = integral(e_v)

theta_ref_raw = theta_trim + Kp_v * e_v + Ki_v * z_v
theta_ref = clamp(theta_ref_raw, -theta_ref_max, theta_ref_max)
```

`theta_ref_max` は最初は小さく、手動停止できる状態で広げる。初期値の目安は `0.03-0.05 rad`、実機で内側ループが 30 s 以上安定してから `0.08-0.10 rad` まで広げる。`theta_ref` へステップを入れると微分キックが出るため、速度 PI の出力は slew rate で制限する。

yaw 制御:

```text
e_yaw_rate = yaw_rate_ref - yaw_rate
I_yaw_unsat = Kp_yaw_rate * e_yaw_rate
```

倒立成功前は yaw を無効にする。倒立が成立した後も、共通モード電流を優先し、残った電流余裕だけを yaw へ割り当てる。yaw のために `I_common` を削ってはならない。

### deg 系実装から rad 系実装への移行

現在のファームウェアには `DEFAULT_PITCH_TARGET = 86.0f` のような deg 系の暫定実装が残る。制御器本体は rad 系へ正規化し、I/O 境界だけで deg を扱う。

```cpp
float theta_rad = (pitch_target_deg - imu_pitch_deg) * PI / 180.0f;
float theta_dot_rad_s = -imu_pitch_rate_dps * PI / 180.0f;
```

この変換は、既存実装で「前傾すると `pitch_target_deg - imu_pitch_deg` が正になる」前提に合わせたもの。IMU 実装が変更された場合は、低電流・手で保持した状態の試験で `theta_rad > 0` と前傾が一致することを再確認する。

既存の deg 入力ゲインを挙動保持で rad 入力へ移す場合は次式を使う。

```text
K_rad = K_deg * 180 / pi
```

ただし、これは単位変換であって安定性の保証ではない。Current Control Mode では制御出力が RPM ではなく A であるため、PID ゲインは電流指令系として設計する。

### 離散 PID 実装

制御周期 `dt` は実測値を使い、許容範囲外なら制御せず FAULT へ遷移する。200 Hz なら通常 `dt ~= 0.005 s`。

```cpp
struct BalancePidState {
    float z_theta = 0.0f;
    float theta_dot_lpf = 0.0f;
    float i_unsat_A = 0.0f;
    float i_sat_A = 0.0f;
};

float updateBalancePid(BalancePidState& s,
                       float theta_rad,
                       float theta_ref_rad,
                       float theta_dot_rad_s,
                       float dt_s,
                       float i_limit_A) {
    const float e = theta_rad - theta_ref_rad;

    const float tau_d_s = 0.02f;  // 実機ログで決める。未検証なら小さくしない。
    const float alpha = dt_s / (tau_d_s + dt_s);
    s.theta_dot_lpf += alpha * (theta_dot_rad_s - s.theta_dot_lpf);

    const float p = Kp_theta * e;
    const float d = Kd_theta * s.theta_dot_lpf;
    const float i_before = Ki_theta * s.z_theta;

    s.i_unsat_A = p + d + i_before;
    s.i_sat_A = constrain(s.i_unsat_A, -i_limit_A, i_limit_A);

    const bool saturated_high = s.i_unsat_A > i_limit_A;
    const bool saturated_low = s.i_unsat_A < -i_limit_A;
    const bool drives_out_of_sat =
        (saturated_high && e < 0.0f) || (saturated_low && e > 0.0f);

    if ((!saturated_high && !saturated_low) || drives_out_of_sat) {
        s.z_theta += e * dt_s;
    }

    if (fabsf(Ki_theta) > 1e-6f) {
        const float back_calc_gain = 0.2f;
        s.z_theta += back_calc_gain * (s.i_sat_A - s.i_unsat_A) / Ki_theta;
    }

    return s.i_sat_A;
}
```

実装規則:

- 初期倒立調整では `Ki_theta = 0` とし、P と D だけで立つことを確認する。
- `D` は測定値の微分、つまり `theta_dot` を使う。`theta_ref` のステップに反応する error 微分は使わない。
- `theta_dot_lpf` の時定数はゲインと同時に設計する。ログ上で IMU ノイズが電流指令に見えている場合は、`Kd` を上げる前にフィルタを見直す。
- FAULT、DISARMED、転倒角超過、Bus Watchdog 復旧、速度ガード発動、電流制限のハード切替時は `z_theta` と速度 PI の積分器を 0 にする。
- 電流制限で `I_common` が飽和している間は、yaw と速度 PI の積分を進めない。

### 電流飽和と yaw 優先順位

左右個別に最後だけ clip すると、共通モードと差動モードの比率が壊れ、倒立中に yaw が姿勢電流を奪う。必ず次の順序にする。

```cpp
float i_common = constrain(i_common_unsat, -i_max, i_max);

if (i_common != i_common_unsat) {
    freezeVelocityIntegrator();
    freezeYawIntegrator();
}

float yaw_headroom = fmaxf(0.0f, i_max - fabsf(i_common));
float i_yaw = constrain(i_yaw_unsat, -yaw_headroom, yaw_headroom);

float i_left = i_common - i_yaw;
float i_right = i_common + i_yaw;
```

この後に左右各輪の速度ガードを適用する。速度ガードが駆動方向電流を削った場合、その周期では姿勢 PID の積分器を進めない。ハード速度上限、通信期限切れ、フィードバック不一致では電流を 0 にして FAULT とする。

### 実機チューニング手順

この順序を飛ばした個体は「倒立成功を保証して送り出せる」状態ではない。

1. 駆動禁止でセンサだけを記録する。
   - 200 Hz 周期の `dt`、IMU pitch/pitch rate、左右 Present Velocity、Present Current、電圧、温度を 60 s 記録する。
   - 静止時の `theta_rad` 標準偏差、`theta_dot_rad_s` 標準偏差、通信欠落数を残す。
   - `dt > 0.007 s` が 1 回でも出る、または Sync Read 欠落が連続する個体は調整へ進めない。

2. 車体を浮かせて低電流符号を確認する。
   - 左右それぞれ `+0.03-0.05 A` を 100 ms 以下で印加する。
   - 両輪正電流で前進方向、差動電流で期待 yaw 方向、Present Velocity の符号が座標規約と一致することを確認する。
   - 符号が不明なまま `Kp_theta` を上げない。

3. 手で保持した状態で姿勢符号を確認する。
   - `Ki_theta = 0`, `Kx = 0`, `Kv = 0`, yaw 無効、速度 PI 無効にする。
   - 車体を前へ小さく傾けたとき、`theta_rad > 0`、`I_common_unsat > 0` になることをログで確認する。
   - 逆なら IMU 符号またはモータ符号を直し、ゲイン符号で相殺しない。

4. 内側 PD だけで短時間倒立させる。
   - `software_peak_limit_A` を最初は低く置き、手で捕まえられる治具または落下保護を使う。
   - `Kp_theta` を小さい値から上げ、前後どちらへ倒しても車輪が倒れ込み方向へ動くことを確認する。
   - 振動が出たら `Kp_theta` を戻し、`Kd_theta` と `theta_dot_lpf` を調整する。
   - 10 s 立たない状態で `Ki_theta`、速度 PI、yaw を有効にしない。

5. 30 s の静止倒立を合格させる。
   - 条件: yaw 無効、速度 PI 無効、`Ki_theta = 0` または極小。
   - 合格基準: 転倒なし、ハード速度ガードなし、通信 FAULT なし、温度上昇が管理範囲内、電流飽和時間が合計 5% 未満。
   - 飽和が多い場合はゲインで押し切らず、重心位置、電源電圧、車輪摩擦、床面、バッテリ内部抵抗を見直す。

6. 必要な場合だけ `Ki_theta` を足す。
   - 目的は IMU バイアスや重心ズレによる定常傾きの補正に限定する。
   - `Ki_theta` を入れて振動周期が長くなる、復帰が遅れる、飽和解除後に大きく戻る場合は 0 へ戻す。

7. 外側速度 PI と yaw を追加する。
   - `theta_ref_max` を `0.03 rad` から始める。
   - 速度指令の符号反転、ゼロ復帰、通信途絶で速度 PI 積分器が必ず 0 へ戻ることを確認する。
   - yaw は `I_common` の残電流でのみ動かし、倒立の合格基準を再度満たすまで上限を広げない。

8. 最終出荷試験を行う。
   - 同じ個体、同じ電源構成、同じ車輪、同じ床条件で 10 回連続起動し、各回 60 s 以上倒立する。
   - 各回で開始、停止、軽い外乱、速度ゼロ復帰、非常停止、通信断を実行する。
   - 失敗ログを 1 件でも残したら、原因を潰して 10 回連続試験を最初からやり直す。

### 出荷判定ゲート

「倒立成功すると保証して送り出す」は、次の条件をすべて満たした個体・設定・環境に限って使う。未実施または未記録なら保証不可と判定する。

```text
[ ] CTRL_UNIT と FIT_INPUT_SOURCE が確定しており、起動ログに残る
[ ] pitch, pitch_rate, motor current, motor velocity の符号試験ログが残る
[ ] 200 Hz 制御周期で worst-case loop time が閾値内
[ ] Sync Write/Read の欠落・スキップ時に非ゼロ電流を出さない
[ ] inner PD だけで 30 s 静止倒立に合格
[ ] 最終設定で 10 回連続 60 s 倒立に合格
[ ] 電流飽和時間、最高温度、最低電圧がログで管理範囲内
[ ] Bus Watchdog 発火時と通常 FAULT 時の両方で Torque OFF/readback に合格
[ ] FAULT 後に自動再アームしない
[ ] 使用した firmware commit、config、床条件、電源、車輪をログへ紐付ける
```

このゲートは「ソフトウェア設計として妥当」という意味ではなく、実機個体を出荷してよいかの判定である。XL330 のトルク定数、床摩擦、電源電圧、重心、IMU 取付、通信品質が変わった場合は、同じゲインでの倒立成功を再保証しない。

---

## 8. Current Control Mode で必要な速度保護

`Velocity Limit(44)` は `Goal Velocity` の上限であり、Current Control Mode における機械的な速度リミッタではない。外側の制御コードで速度保護を実装する。

原則:

- 車輪が上限方向へ回っているとき、その方向へさらに加速する電流を抑える。
- 減速方向の電流は許可する。
- ソフト上限を超えたら滑らかに駆動側電流を縮小する。
- ハード上限を超えたら FAULT へ遷移する。

```cpp
float applySpeedGuard(float i_cmd_A,
                      float omega_rad_s,
                      float omega_soft,
                      float omega_hard) {
    const float abs_w = fabsf(omega_rad_s);

    if (abs_w >= omega_hard) {
        raiseSafetyFault("wheel overspeed");
    }

    const bool accelerating = i_cmd_A * omega_rad_s > 0.0f;
    if (!accelerating || abs_w <= omega_soft) {
        return i_cmd_A;
    }

    float scale = (omega_hard - abs_w) / (omega_hard - omega_soft);
    scale = constrain(scale, 0.0f, 1.0f);
    return i_cmd_A * scale;
}
```

`omega_soft` と `omega_hard` は仕様上の無負荷速度だけで決めず、車輪径、転倒時の危険性、床面条件、電源電圧を考慮して実験的に決める。

---

## 9. 安全状態機械

最低限、次の状態を持つ。

```text
DISARMED
INITIALIZING
ARMED_IDLE
BALANCING
FAULT
SHUTDOWN
```

`BALANCING` へ入る条件:

- 左右 DYNAMIXEL が正常に応答する。
- IMU が正常で時刻が新しい。
- pitch が開始許容範囲内。
- 車輪速度が開始許容範囲内。
- `Goal Current` が左右とも 0。
- `Hardware Error Status` が 0。
- 電圧と温度が設定範囲内。

少なくとも監視する FAULT 条件:

- pitch が転倒角を超えた。
- 車輪速度がハード上限を超えた。
- IMU または DYNAMIXEL フィードバックが期限切れ。
- 通信エラーが連続した。
- 制御ループの期限超過が連続した。
- `Hardware Error Status` が非 0。
- 温度がソフト上限を超えた。
- 電圧が許容範囲外。
- 指令電流と実測電流の偏差が異常に大きい。
- ユーザー停止または非常停止。

FAULT 時の基本順序:

```text
1. Goal Current を左右とも 0
2. 短い確認時間内に 0 指令を再送
3. Torque Enable を 0
4. ログへ原因を記録
5. 自動再アームを禁止
```

Bus Watchdog は最後の防御であり、ホスト側の FAULT 処理を代替しない。

---

## 10. 電流・熱保護

電流制限は三層に分離する。

```yaml
current:
  eeprom_limit_A: REQUIRED
  software_peak_limit_A: REQUIRED
  software_continuous_limit_A: REQUIRED
  peak_duration_s: REQUIRED
```

- `Current Limit(38)` は DYNAMIXEL 側の絶対上限。
- `software_peak_limit_A` は短時間の倒立復帰などに使うピーク上限。
- `software_continuous_limit_A` は熱的に持続可能と確認した上限。
- ピーク電流の累積時間または簡易的な `I^2t` を監視し、長時間の高電流を禁止する。
- 1.47 A やレジスタ最大 1.75 A を安全な連続電流として採用しない。
- 温度と入力電圧をログへ残し、同定試験と倒立試験で電流ゲインが変化していないか確認する。

---

## 11. MuJoCo・同定値と実機コードの関係

システム同定から得られた MuJoCo パラメータ:

| モデル | `gear` | `damping` | 速度 RMSE |
|---|---:|---:|---:|
| 変更前 XML | 0.22 | 0.001 | 104.6 rpm |
| 同定後 | 0.05239 | 7.314e-05 | 27.0 rpm |

この値は仕様表から直接得た物理定数ではなく、試験時の電源電圧、入力スケーリング、負荷条件、通信遅延、内部電流制御、摩擦などをまとめて吸収した実効パラメータである。

必須の設定項目:

```yaml
motor_model:
  fit_input_source: REQUIRED      # goal_current / present_current
  ctrl_unit: REQUIRED             # A / mA / raw_count / normalized
  gear_per_ctrl_unit: 0.05239
  actuator_damping: [7.314e-05, 0.0, 0.0]
  fit_rmse_rpm: 27.0
```

`gear=0.05239` を N m/A と解釈できるのは、同定時に MuJoCo の `ctrl` が A 単位だった場合だけである。`fit_input_source` または `ctrl_unit` が未設定の場合、実機で非 0 電流を出す経路を有効化しない。

MuJoCo 側の基本式:

```text
tau_active = gear * ctrl
tau_damping = -gear^2 * damping * qdot
```

同定後の線形減衰の関節側実効値:

```text
0.05239^2 * 7.314e-05 ~= 2.00748e-07
```

入力単位を変える場合は、同定入力と新入力の関係を `u_fit = s * u_new` として、能動トルクと関節側減衰を保存する。

```text
gear_new = s * gear_fit
damping_new = damping_fit / s^2
```

---

## 12. 推奨設定項目

ファームウェアでは少なくとも次の項目を設定として持つ。

```yaml
robot:
  wheel_radius_m: REQUIRED
  track_width_m: REQUIRED

dynamixel:
  protocol: 2.0
  port: REQUIRED
  baudrate: REQUIRED
  left_id: REQUIRED
  right_id: REQUIRED
  expected_model_number: 1190
  operating_mode: 0
  return_delay_raw: OPTIONAL
  bus_watchdog_timeout_ms: REQUIRED

current:
  eeprom_limit_A: REQUIRED
  software_peak_limit_A: REQUIRED
  software_continuous_limit_A: REQUIRED
  peak_duration_s: REQUIRED
  slew_rate_A_per_s: REQUIRED

sign:
  left_motor: REQUIRED
  right_motor: REQUIRED
  imu_pitch: REQUIRED
  imu_pitch_rate: REQUIRED

control:
  rate_hz: REQUIRED
  pitch_start_limit_rad: REQUIRED
  pitch_fault_limit_rad: REQUIRED
  wheel_speed_soft_rad_s: REQUIRED
  wheel_speed_hard_rad_s: REQUIRED
  feedback_timeout_s: REQUIRED

motor_model:
  ctrl_unit: REQUIRED
  fit_input_source: REQUIRED
```

`REQUIRED` が残っている場合は、実機を駆動するモードを開始せず、不足項目を列挙して終了する。

---

## 13. 必須ログ項目

同定、制御調整、事故解析のため、同じ時刻軸で保存する。

```text
host monotonic time
control-loop dt
IMU raw acceleration / angular velocity
estimated pitch / pitch rate
left/right position
left/right velocity
left/right goal current
left/right present current
left/right present PWM
unsaturated common/yaw command
saturated left/right command
input voltage
temperature
hardware error status
safety limiter flags
state-machine state
fault reason
```

ファイル書き込みを制御ループ内で同期実行しない。リングバッファと別スレッド、またはファームウェアで利用可能な非同期送出手段を使う。

---

## 14. 受入試験として実装する項目

### 単位変換

- `+1 A` が `Goal Current raw +1000` へ変換される。
- `-1 A` が `Goal Current raw -1000` へ変換される。
- `Present Velocity raw 1` が `0.229 rpm` へ変換される。
- 負値が 2 の補数として正しく処理される。
- 4096 pulse が `2*pi rad` へ変換される。

### 初期化安全

- Torque ON 中に EEPROM を書かない。
- Operating Mode 変更後、Torque ON 前に `Goal Current=0` が送信される。
- 片輪の PING または設定検証に失敗した場合、両輪とも Torque ON しない。

### 符号確認

車体を持ち上げた状態で低電流を短時間印加し、次を確認する。

- 両輪正指令で前進方向へ回る。
- 左右差動指令で期待する yaw 方向になる。
- IMU の正 pitch と車体の物理傾斜が一致する。

### 安全

- 通信を意図的に止め、Bus Watchdog が停止する。
- IMU 更新を止め、FAULT へ遷移する。
- pitch 閾値超過で電流 0、Torque OFF になる。
- 過速度時に加速方向電流だけが抑制される。
- FAULT 後に自動再アームしない。

---

## 15. 実装時の禁止事項

- `gear=0.05239` を、入力単位の確認なしに `0.05239 N m/A` と断定しない。
- `gear=0.05239` を最大トルク `0.05239 N m` と断定しない。
- 物理減速比 77.5 を出力軸モデルへ重ねて掛けない。
- ストールトルク `0.215 N m` を連続定格として電流制限に使わない。
- `Velocity Limit` が Current Control Mode の速度を保護すると仮定しない。
- モード変更後、`Goal Current` を 0 へ戻さず Torque ON しない。
- 実機とシミュレーションで異なる符号規約や入力単位を使わない。
- 制御ゲイン、電流上限、転倒角、速度上限を根拠なく生成しない。
- 単一試験条件で得た同定値を全電流・全速度・全負荷域に外挿しない。
- 安全機構、ログ、単体試験を省略しない。
