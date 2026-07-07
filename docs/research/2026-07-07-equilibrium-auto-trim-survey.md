# 平衡点オンライン自己推定 サーベイ結果 (2026-07-07)

対応する設計計画: [`docs/plans/2026-07-07-equilibrium-auto-trim.md`](../plans/2026-07-07-equilibrium-auto-trim.md)
調査体制: 文献系・実装系の2系統並列 (Web一次情報)。本文書は両報告の全文アーカイブ。

---

# Part A: 文献調査 — TWIP平衡点のオンライン自己推定・自己調整

## A1. 平衡点/重心オフセットのオンライン推定

### A1(a) 車輪速度・位置の積分による trim 学習

**方式A: 外側速度ループの積分項がθ_refに収束する標準構成** (本システムと同一構成)
- 更新則: `θ_ref[k] = θ_ref[k-1] + Ki_outer·(v_target − v_wheel[k])·Ts`
- 定常で `v_wheel→0` になるよう θ_ref が収束し、収束値が「真の平衡点(重心オフセット込み)」の推定値になる
- 時定数目安: 外側ループ帯域は内側の1/10程度が経験則。積分ゲインは「数秒〜十数秒で定常収束」に絞る(速すぎると内側と干渉して振動)
- 出典: [Designing a Self-Balancing Robot with Pictorus](https://blog.pictor.us/self-balancing-robot/) / [Tilt set-point correction system for balancing robot using PID controller](https://www.researchgate.net/publication/308838380) (要旨のみ)

**方式B: 二重速度推定の差分によるオフセット推定** (Toyota 量産車両特許)
- [US8874319 "Inverted pendulum type vehicle"](https://patents.google.com/patent/US8874319)
- 運動学的推定と動力学的推定の**差分**にゲインを乗じてオフセットを漸進更新。単一積分よりノイズ頑健。デッドゾーン処理で誤動作防止。計算負荷は極小

**共通の弱点**: いずれも「車輪速度」が学習信号のため、**静止中(スティクション帯域内)は学習信号が消失する**。

### A1(b) 平均モータ電流(トルク)ゼロ化による平衡点推定 ★本システム最有力

- 原理: 定常で電流指令の時間平均は重心オフセット分の復元トルクに収束。`Δθ̂ ≈ ī / (Kt·m·g·L)` と既知の静的ゲインで直接変換可能
- 更新則 (緩学習): `θ_offset_hat[k] = θ_offset_hat[k-1] + α·(LPF(i_cmd[k]) / Kτ)`
- 出典: [The Secrets of Segway Revealed to Students](https://www.researchgate.net/publication/329391745) / [Cornell ECE4760 Self-balancing Robot](https://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/f2015/dc686_nn233_hz263/final_project_webpage_v2/dc686_nn233_hz263/index.html)
- **本システムへの意義**: XL330は電流制御モードのため既存制御信号のタップのみで実装可能。**スティクション静止中も復元トルク分の電流バイアスは流れ続けるため、(a)が沈黙する場面でも学習信号が残る**

## A2. DOB / ESO / ADRC

### 実機実績の代表例 (Mechatronics 76 (2021) 102552)
Curiel-Olivares et al., "Self-balancing based on ADRC for the Two-In-Wheeled Electric Vehicle" — [DOI](https://doi.org/10.1016/j.mechatronics.2021.102552) / [PDF](https://www.utm.mx/avance_mir_2021/Programa%20104/001%20DESAROLLO_PROYECTOS_INVESTIGACION/Productos%20de%20investigacion/3%20Inst_EM/Art_Rev_Internac/5_Self-balancing%20based%20on%20Active.pdf)
- 4次線形ESO: `F̂0'=F̂1+l3e, F̂1'=F̂2+l2e, F̂2'=u_δ+η+l1e, η'=l0e`
- 極配置 `(s²+2ζω_o s+ω_o²)²` → `l3=4ζω_o, l2=2ω_o²+4ζ²ω_o², l1=4ζω_o³, l0=ω_o⁴`
- 実機: ω_o=25, ζ=0.707 / ω_n=18, α=18。TMS320F28335、傾斜計≈143Hz。ISSで分離設計を証明
- 200Hz MCU実現性: 数十FLOP、ESP32で余裕

### ESO残差を教師信号とする重心学習 ([arXiv:1810.03076](https://arxiv.org/abs/1810.03076), Georgia Tech)
- ADRCで即時バランス + ESO推定外乱を教師に質量モデルβを勾配降下 `β_{t+1}=β_t−η∇J`
- CoM誤差 2.5cm→0.4cm。**「ESO残差→低速二次学習器で平衡点更新」パターンは200Hz MCUへ移植可能**

### その他 (要旨レベル)
- [LADRC for Two Wheel Self-Balancing Robot](https://www.joe.uobaghdad.edu.iq/index.php/main/article/view/3528) (GA整定、PID比90%安定性向上と報告)
- [Nonlinear-Model-Based Disturbance Compensation (Actuators 2022)](https://www.mdpi.com/2076-0825/11/11/339)
- [Control Strategies for Two-Wheeled Self-Balancing Robotic Systems: Review (2025)](https://www.mdpi.com/2218-6581/14/8/101) — MCU/DSP実装のSMC+外乱推定例複数

## A3. MRAC / L1適応

- MRACは主にプラント利得・慣性変動への適応で、**定常オフセット(平衡点)推定を明示ターゲットにした例はほぼ無い** ([ICRA 2019 MRAC](https://dl.acm.org/doi/10.1109/ICRA.2019.8793633) 等)
- L1適応のTWIP平衡点適応への直接適用例は**発見できず (研究ギャップ)**。近縁: [L1 Adaptive Output Feedback (倒立振子カート)](https://arxiv.org/pdf/1901.07427)

## A4. スティクション/バックラッシ下の平衡点近傍制御

- 記述関数法によるリミットサイクル予測。積分制御+バックラッシは自励LCを生みやすい
  - [Limit Cycle Existence Condition in Control Systems with Backlash and Friction (IFAC)](https://www.sciencedirect.com/science/article/pii/S1474667015370117)
  - "Self-Excited Limit Cycles in an Integral-Controlled System With Backlash" (全文未入手、本システム構成に直結、追加入手価値高)
- **学習信号消失問題への実務対処 (統合)**:
  1. 静止中も電流指令は非ゼロ(リミットサイクル状) → **平均電流方式は信号を保持**
  2. 平均化窓はLC周期(3.2Hz→0.31s)の数倍(1〜数秒)必須。短いとACリップル混入
  3. スティクション対策: PFM/微小ディザ ([Hackaday PDF](https://cdn.hackaday.io/files/16098688736832/GafarA_final.pdf), [RCmags/SelfBalancingRobot](https://github.com/RCmags/SelfBalancingRobot))。LC周波数を高域へシフトし学習ループと周波数分離
  4. ゲーティング則: 「|v|<ε かつ |LPF(i)|閾値以下」なら学習停止、「|v|<ε だが電流DC有意」なら電流ベースへフォールバック

## A5. trim transfer と永続化

- Toyota特許ファミリー(US8874319, US9162726): オンライン推定値を制御基準へ反映する量産実例 (恒久保存への言及なし)
- ホビー実装: 一回限りの静的較正 + EEPROM が主流。「書込磨耗のため頻繁保存禁止」が通説
- HVAC "Trim & Respond": 内側整定値を外側の遅い積分で基準値へ移す一般パターン
- **定石**: RAM学習(毎周期) → 「収束・安定判定 + 最小書込間隔レート制限」でNVSへ二段階反映

## A6. 本システムへの推奨順位

1. **平均電流ゼロ化 + 長時間移動平均** — 新規センサ不要・静止中も信号残存・実装コスト極小
2. **既存速度ループのESO的再解釈 + 残差ゲーティング** — TMS320実績ゲイン帯 (ω_o=25) を参考
3. **trim transfer + NVS階層書込** — RAM学習と永続化の分離、電源再投入後の即戦力化

---

# Part B: 実装調査 — OSSバランサの平衡点自動trim

## B0. 総括

- **「電流制御モード + 走行中自動学習 + 自動永続化」を全て備えた実装は存在しない** (空白領域)
- 自動trimの実体は例外なく積分器: (a)速度誤差/PID出力符号 (b)角度定常誤差 (c)車輪位置累積 (d)理論/実測加速度残差
- 自動・定期的なEEPROM/NVS永続化は皆無。手動ワンショット保存のみ (B-ROBOT EVO2 / ArduPilot / Bala2)
- デッドバンド補償・摩擦FF・ディザとtrimロジックの明示連携例も無し

## B1. 実装別要点表

| 実装 | 初期値 | 自動trim (信号→積分先) | クランプ/リセット | 永続化 |
|---|---|---|---|---|
| B-ROBOT (JJROBOTS) | オフセット概念なし | 無し | 転倒±76°でPID積分リセット | 無し |
| B-ROBOT EVO2 | `ANGLE_OFFSET=0`固定 | 速度PI積分がtarget_angleを生成(構造的副産物) | ITERM_MAX=10000、転倒でリセット | 手動ワンショット(RAM止まり) |
| YABR (Brokking系) | 加速度角±0.5°時にロード | **pid_output符号→±0.0015°/4msで基準角へ** | クランプ無し。転倒±30°/低電圧で0リセット | 無し |
| YABR (Draradech) | 固定`-210` | 位置+速度カスケードで吸収 | 異常時モータ無効化 | 無し |
| ArduPilot BalanceBot | `BAL_PITCH_TRIM`固定 | **無し** | 通常フェイルセーフ | 手動パラメータ保存 |
| M5 BalaC | 起動較正 | 角度誤差積分→**トルクバイアス** (KIang=800, 10ms) | 詳細未確認 | 詳細未確認 |
| M5 Bala2 | 起動較正 | 速度積分→トルクバイアス (s_ki=0.075, ±40クランプ) | ±40 | 手動NVS保存のみ |
| hoverboard-hack (Niklas/EFeru) | — (人間が平衡) | 無し | 電流保護のみ | ペダル較正のみ |
| Onewheel系 | 固定 (FWパッチ±10°) | 無し | PID出力クランプ | 商用機は不揮発保存 |
| milana_robot (ODrive脚ロボ) | 幾何計算 | **理論/実測加速度残差→P=4·smooth=0.998で基準角へ** | ±0.2rad、回頭中ゲート、転倒で0リセット | 未確認 |
| HoverBot (ODrive電流制御) | 固定0 | **無し** (純電流制御だがtrim無し) | tilt 40°のみ | 無し |
| SimpleFOC balancer (公式) | ジャイロ較正のみ | **速度誤差→PI(I=0.03)→LPF(0.07s)→target_pitch** | pid_vel.limit=π/10。転倒検知自体なし | 無し |
| FranHawk simplefoc | 手動`angle_pitch_offset` | speed_I_sum(±15)がtrim兼用 | 転倒で0リセット | 無し |
| teeterbot (ROS) | — | 無し (シミュレータ) | pitch>60°でトルク0 | 無し |

## B2. 優良3パターン (コード引用)

### P1: YABR — 符号積分 (最小実装)
```cpp
if(pid_setpoint == 0){
  if(pid_output < 0)self_balance_pid_setpoint += 0.0015;
  if(pid_output > 0)self_balance_pid_setpoint -= 0.0015;
}
pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
// 転倒/低電圧: pid_i_mem = 0; self_balance_pid_setpoint = 0;
```
評価: 「PID出力≠0が続く=平衡点ズレ」を固定ステップ符号積分のみで実現。クランプ無し・永続化無しが弱点。

### P2: milana_robot — 加速度残差積分 (最も本番品質)
```cpp
md.error_p = md.imu_x_angle - md.balance_control_target_angle - md.t_correction_angle;
// 回頭中でなければ:
float t_correction_error = md.balance_control_vel_output - md.balance_control_vel_quasi;
md.t_correction_angle -= sign(t_correction_error) * cd.t_correction_simple_factor * delta_time;
md.t_correction_angle_raw += (md.coi_theta_acc - md.coi_world_acc) * cd.t_correction_acc_p * delta_time;
md.t_correction_angle = update_smooth(..., cd.t_correction_acc_smooth /*0.998*/, ...);
// クランプ ±t_correction_angle_max(0.2rad)、!is_standing で全リセット
```
評価: 理論/実測加速度の乖離を学習信号化。クランプ・平滑・回頭ゲート・転倒リセット完備。

### P3: SimpleFOC balancer — カスケードPI+LPF (トルク指令モードで唯一)
```cpp
PIDController pid_vel{.P=0.01, .I=0.03, .D=0, .ramp=10000, .limit=_PI/10};
LowPassFilter lpf_pitch_cmd{.Tf=0.07};
float target_pitch = lpf_pitch_cmd(pid_vel((motor1.shaft_velocity+motor2.shaft_velocity)/2 - lpf_throttle(throttle)));
float voltage_control = pid_stb(target_pitch - pitch);
motor1.controller = MotionControlType::torque;  // 実体はvoltage
```
評価: カスケード構造自体がtrimを兼ねる。電流制御サーボでのバランサに最も直接的な参考。

## B3. 出典

- https://github.com/jjrobots/B-ROBOT / https://github.com/jjrobots/B-ROBOT_EVO2
- https://github.com/berton7/Self-balancing-robot / https://github.com/jcleve/BalanceBot / https://github.com/eigenpi/YARY / https://github.com/siredmar/yabr / https://github.com/Draradech/yabr
- https://github.com/NiklasFauth/hoverboard-firmware-hack / https://github.com/EFeru/hoverboard-firmware-hack-FOC
- https://github.com/non-bin/rewheel / https://github.com/akash-idnani/OneWheelv2
- https://github.com/helmutbuhler/milana_robot (`robot/balance_control.cpp`)
- https://github.com/LuSeKa/HoverBot / https://github.com/GearDownForWhat/BalanceBot
- https://github.com/simplefoc/Arduino-FOC-balancer / https://github.com/FranHawk/simplefoc_balance_car
- https://github.com/robustify/teeterbot
- https://patents.google.com/patent/US8874319 / https://patents.google.com/patent/WO2012160400A1/en
- Ninebot S FAQ: https://www.mi.com/global/support/faq/details/KA-07635/

## B4. 未解決・追加調査推奨

1. ArduPilot / M5 BalaC / Bala2 は要旨止まり — 採用検討時はコード原文の直接確認を推奨
2. electricunicycle.org系フォーラム (403) に一輪車較正アルゴリズムの逆解析情報の可能性
3. 「milana方式の残差積分 × SimpleFOCカスケード構造 × 真の電流フィードバック」の組合せはOSS前例なし — 本プロジェクトで実装すれば先行事例
