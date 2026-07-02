# 設計計画: 電流制御モード + FreeRTOS 再設計（BSL-Balancer / ﾀｲﾘﾝﾁｬﾝ）

- 日付: 2026-07-02
- ブランチ: `dev/current-mode-xl330-fable5`
- 目標: XL330-M077 を **Current Control Mode (Operating Mode=0)** で駆動する対向二輪型倒立振子の **PIDベース制御器**を、**最適設計された FreeRTOS 構成**の上に **200Hz 制御ループ**で実装する。
- 規範: [`docs/XL330_M077_TWIP_coding_agent_reference.md`](../../../../docs/XL330_M077_TWIP_coding_agent_reference.md)（ルートリポジトリ。以下「XL330規範」）
- 監査/リサーチ入力: Fable本体による現行コード監査 + ライブラリソース実査、Sonnet5 サブエージェントによる Web リサーチ2本

## 0. 現行実装の要修正点（監査サマリ）

現行 `src/main.cpp`（287行・単一ファイル）の監査で確定した主要問題:

| 領域 | 問題 |
|---|---|
| 制御 | 速度モード駆動（内部速度PIが二重ループ化）/ D項が誤差の数値微分（`kalman.getRate()`算出済み未使用）/ アンチワインドアップが突然ゼロ化方式 / 車輪速度フィードバック無し / 出力飽和と積分が非連動 |
| 推定 | dt固定値（実測値未使用）/ ジャイロバイアス起動時校正なし / 加速度ノルムゲートなし / pitch≈90°基準で平衡点(86°)とオフセットが混在 |
| RTOS | 100Hz / 周期デッドライン超過の検出なし / 共有パラメータの同期機構なし / 空loop() |
| DXL | 個別書き込み×2往復 / ping・書込エラー未確認 / Model Number未検証 / **Bus Watchdog未使用** / Present系常時読取なし / Current Limit・Shutdown未設定 |
| 品質 | 単位系混在(deg/RPM/ms) / マジックナンバー散在 / 単体テストゼロ |

## 1. 確定事実（ライブラリソース実査 + リサーチ）

1. **M5GFX I2C はトランザクション単位の FreeRTOS mutex で保護**（`M5GFX/src/lgfx/v1/platforms/esp32/common.cpp:1027-1039`）。core0 の IMU 読取と core1 のタッチ/電源読取は、各アクセスが単一トランザクションである限りバスレベルで直列化される。
2. **M5Unified の `getAccelData`/`getGyroData` は前回 update から 256µs 超で自動 `update()`**（`IMU_Class.cpp:464`）。正攻法は周期毎に `M5.Imu.update()` → `getImuData()` で一貫スナップショット（タイムスタンプ付き）を取ること。
3. **Core2 は M5Unified の軸リマップ補正対象外**。v1.0(MPU6886) / v1.1(BMI270) ともチップ実装向きの生軸 → v1.1 での軸・符号は実機符号試験で確定させる必要がある（§7 参照）。
4. **M5Unified は BMI270 の ACC_CONF/GYR_CONF を書かない** → チップデフォルト（gyro ODR 200Hz / accel ODR 100Hz、gyro ±2000dps / accel ±8g）のまま。200Hz 制御にはgyro ODR 引き上げ（400Hz）をレジスタ直書きで行う（`imu_bmi270` 判定時のみ）。
5. **M5Unified の IMU オフセット状態は完全に自前管理へ**: `IMU_Class::begin()` は NVS から過去のオフセットを暗黙ロードし、`setCalibration(0,0,0)` は自動調整を止めるだけでロード済みオフセット値は残る（ソース確認）。→ `imu_backend` は begin() 直後に **`clearOffsetData()`（公開 API 確認済み）で隠れ状態を破棄**し、自前の起動時静止校正（ジャイロバイアス）→ `setCalibration(0,0,0)` 凍結、の順とする。M5Unified の NVS オフセットは使用も保存もしない（校正値は §9.1 の自前 NVS レコードのみ）。テスト: 事前に不正オフセットを仕込んでも姿勢推定に混入しないこと。
6. **Dynamixel2Arduino 0.8.1 で必要 API は全て利用可能**: `OP_CURRENT=0` / `setGoalCurrent(id, val, UNIT_MILLI_AMPERE)`（XL330_M077 対応）/ `syncWrite`・`syncRead`・`fastSyncRead`（`utility/master.h:151-156`）/ `ControlTableItem::BUS_WATCHDOG`。
7. ベースライン `pio run` 成功（RAM 0.6% / Flash 8.1%）。
8. **FreeRTOS tick = 1000Hz**（arduino-esp32 sdkconfig `CONFIG_FREERTOS_HZ=1000` 確認）→ `vTaskDelayUntil` 5 tick = 正確に 5ms。
9. **m5stack-avatar のタスクは `drawLoop`(優先度1)・`facialLoop`(優先度2) とも APP_CPU(core1) 固定**（`Avatar.cpp:175-190`）。`avatar.suspend()` は drawLoop のみ停止（facialLoop は継続、負荷軽微）。core0 の制御タスクとは完全分離。
10. **XL330 e-manual 公式確認**（リサーチA、`research_twip_control.md`）:
    - **Current Limit(38) デフォルト = 1750（=1.75A、レジスタ最大値）** → 明示的に下げない限り事実上無制限。初期化での保守的設定が安全上必須
    - **Operating Mode 変更時に Goal Current が Current Limit 値へリセットされる挙動は公式 e-manual で確認**（モード変更後・Torque ON 前の Goal Current=0 送信は必須手順）
    - PWM Limit(36、デフォルト885=100%)は**全モード共通の出力上限**として電流モードにも作用 → 初期化時に読取・検証
    - Shutdown(63) デフォルト=53。**Shutdown 発火後の復帰は Torque 再 ON では不可、REBOOT が必須**
    - Status Return Level(68) はデフォルト 2 のまま維持（毎周期 READ するため変更禁止）
    - Return Delay Time(9) デフォルト 250（=500µs）→ 0 へ設定（コミュニティ実践と整合、読取レイテンシ短縮）
    - トルク定数は電圧依存 0.131〜0.162 N·m/A（公式スペック）。単4×3（~4.5V）では ~0.146-0.150 N·m/A と推定
11. **200Hz TWIP の直接の先行事例 = M5Stack Bala2**（公式ファーム）: `vTaskDelayUntil(&last_ticks, pdMS_TO_TICKS(5))` の 200Hz、**角度 PID（Ki=0）＋速度 PID（Ki=0.075、積分クランプ±40）の出力並列加算**、転倒時（70°）に出力 0＋速度積分強制ゼロ。ドリフトトリムの積分器は速度側にのみ置く構成が実績あり。
12. Dynamixel2Arduino の `dir_pin` はデフォルト -1（GPIO トグルなし）で、半二重は基板側ハードで実現 → 現行構成のまま変更不要。`setGoalCurrent` は raw=round(mA)・±1750 内部クランプ。
13. **ギャップ（実機検証が必要な項目）**: DYNAMIXEL 電流モード駆動のホビー TWIP 実例は未発見 / D 項 LPF カットオフの確立値なし（ドローン領域からの類推で 20-30Hz を初期値に）/ XL330 の静止摩擦補償の数値事例なし / 1Mbaud 実測レイテンシは理論値（往復 ~0.3-0.8ms）のみ。

## 2. 制御アーキテクチャ

### 2.1 制御構造（カスケード PID）

XL330規範 §10 の構造を PID 系で実装する。内部単位は SI（rad, rad/s, m/s, A）。

**座標系契約（単一変換点）**: `imu_backend`/推定器は**絶対傾斜 `pitch_abs`**（atan2 基準）のみを公開する。ControlTask はループ先頭で **一度だけ** `θ = pitch_abs − pitch_eq` を計算し、**以降の全計算（制御誤差・起立検出・転倒検出・スナップショット）は 0 中心の θ のみを使う**。`pitch_eq` がこの変換点以外に現れたら実装バグである。native テストは pitch_eq≠0 で制御誤差・BALANCING 開始判定・転倒判定を検証する。

```
v_ref(=0) ──►[速度 PI]──► θ_ref（0 中心、±theta_ref_limit にクランプ）
                              │
θ, θ̇ ────────────────────────►[ピッチ PID]──► I_common [A]
                                                  │
                            I_yaw(=0, 構造のみ) ──►[ミキサ+飽和(倒立優先)]
                                                  │
                                    [速度ガード]→[スルーレート制限]→[クランプ]
                                                  │
                        Goal Current 書込 L, R（BALANCING 中は Status 検証付き個別 WRITE、§4.2）
```

- **内側ピッチ PID**（200Hz）: `I_common = Kp·e + Ki·∫e·dt − Kd·θ̇_filt`
  - e = θ_ref − θ（両者とも 0 中心）。**D はジャイロレート直接使用**（derivative on measurement。BalaC/Bala2/HomeMadeGarbage の 3 実例すべてで確認された定石）
  - θ̇_filt: D 項用 PT1 ローパス（カットオフ初期値 25Hz、TUNE。TWIP 固有振動数 ~2Hz に対し十分高く、ギヤノイズを抑制）
  - **Ki 初期値 = 0**（Bala2 実績に従い、ドリフトトリムの積分は速度側にのみ置く。Ki はパラメータとして残す）
  - 積分はクランプ型アンチワインドアップ + 出力飽和時の条件付き積分。BALANCING 以外では常時リセット（Bala2 の「転倒時積分強制ゼロ」と同じ）
- **外側速度 PI**（200Hz 実行・低帯域）: `θ_ref = Kv·(v_ref − v) + Kvi·∫(v_ref − v)dt`（0 中心。pitch_eq は §2.1 冒頭の単一変換点でのみ使用）
  - v = r·(φ̇_L + φ̇_R)/2（Present Velocity から算出）
  - θ_ref は ±theta_ref_limit（初期値 3°）にクランプ、積分もクランプ
  - **役割**: 電流モードで必然の定常ドリフト抑制 + 平衡点誤差の自動トリム（Kvi 積分が pitch_eq 誤差・IMU 取付誤差を吸収。リサーチA 2.1 節「方式B」— 直接観測量ベースで IMU オフセットにロバスト）
  - コンパイル時/パラメータで無効化可能（教育用に「純粋な角度 PID」との比較実験を可能にする）
  - 注: Bala2 は「2 つの PID の出力並列加算」だが、本設計はクランプ付き参照補正（カスケード）を採る。数学的にはほぼ等価で、**速度ループが要求できる傾きが ±theta_ref_limit で構造的に制限される**安全性を優先
- **ミキサ**: `I_L = s_L·(I_common − I_yaw)`, `I_R = s_R·(I_common + I_yaw)`。yaw 制御は v1 では I_yaw=0（teleop 不在）だが構造を用意。飽和は XL330規範 §10.1 の**倒立優先**方式
- **速度ガード**（XL330規範 §11）: 加速方向電流のみソフト上限から線形縮小、ハード上限超過で FAULT
- **電流制限 3 層**（XL330規範 §13): EEPROM Current Limit / ソフトピーク / ソフト連続。初期値は保守的に（実機調整前提、TUNE マーク）

### 2.2 ゲイン初期値の桁（重力トルク均衡からの見積り、リサーチA §5.1 と整合）

質量 ~0.3kg・重心高 ~6cm・車輪半径 29mm・2輪、トルク定数 ~0.146 N·m/A（4.5V 時の推定 ~0.146-0.150）とすると、
重力トルク勾配 `mgh ≈ 0.177 N·m/rad` を両輪で相殺する理論下限 Kp は `0.177/(2×0.146) ≈ 0.6 A/rad（≈11 mA/deg）`。
安定化には数倍必要 → **実用 Kp は 0.5〜5 A/rad（10〜100 mA/deg）帯を想定、初期値 1.5 A/rad、TUNE**。
Kd 初期値 ~0.05-0.10 A/(rad/s)、内側 Ki=0、外側 Kv/Kvi は小さく開始（TUNE）。
トルク定数の電圧依存・Present Current が電源側電流である事実（公式確認）により、机上値への過信は禁物。実機の漸増チューニングが前提。

### 2.3 電流制限（3 層 + I²t 型ピーク制限器 + 立ち上げプロファイル）

| 層 | 値（初期） | 根拠 |
|---|---|---|
| EEPROM Current Limit(38) | **900mA**（TUNE。ブリングアップ完了までは 150mA） | デフォルト 1750=事実上無制限のため必ず下げる。ストール 1.47A@5V の ~60% |
| ソフトピーク上限 | 450mA（TUNE） | 倒立復帰用の短時間ピーク |
| ソフト連続上限 + スルーレート | 300mA / 例 5A/s（TUNE） | AAA×3 の電圧サグ・熱を考慮 |

**ピーク制限は状態を持つ制限器として実装する**（XL330規範 §13 の `peak_duration_s` 要件）。電流指令は符号付きのため、**エネルギー計算と実効上限は |I| で対称に定義する**:

```
E += (|I_cmd|² − I_cont²)·dt   （右辺が正のとき蓄積、負のとき回復）、E は [0, E_max] にクランプ
E ≥ E_max のとき: 実効上限 I_eff = I_cont、それ以外: I_eff = I_peak
出力は ±I_eff で対称にクランプ
E_max = (I_peak² − I_cont²) × peak_duration_s（初期値 peak_duration_s = 0.5s、TUNE）
```

これにより飽和・符号誤り・転倒復帰の繰り返し等でピーク電流が実質連続指令になる経路を遮断する。native テストは**正・負両方向の持続ピーク指令**が連続上限へ収束することを検証する。

**ブリングアップ・プロファイル**: 初回は EEPROM Current Limit=150mA で §7 符号試験→浮かせ試験を通し、合格後に段階的に本番値へ引き上げる（README に手順を記載）。

## 3. FreeRTOS タスク設計

### 3.1 タスク構成

| タスク | コア | 優先度 | 周期 | 責務 |
|---|---|---|---|---|
| `ControlTask` | 0 | 20（高） | 5ms `vTaskDelayUntil` | IMU update → 姿勢推定 → FSM → 制御則 → DXL syncRead/syncWrite → スナップショット発行。**IMU と DXL バスの単一所有者** |
| `UiTask` | 1 | 3（avatar タスクより上） | ~33ms | `M5.update()`、ボタン（STOP 検出含む）、チューニングパネル、avatar 制御、スナップショット表示 |
| avatar 内部タスク | 1 | lib 既定 | - | 顔描画（m5stack-avatar が生成） |
| `loopTask` | 1 | 1 | - | `loop()` は即 `vTaskDelete(NULL)` |

設計原則:
- **単一所有者原則**: DXL バス・IMU は ControlTask のみが触る。ヘルス監視（電圧/温度/HW エラー、XL330規範 §8.2）は ControlTask 内で N 周期毎（例: 各項目 0.5-1s 周期のラウンドロビン 1 読取/周期）に実施し、バス mutex を不要化
- core0 は WiFi/BT 未使用のため制御専用に近い。WiFi を将来追加する場合の再検討条件を app_config に注記
- I2C は M5GFX ロックでトランザクション直列化されるため、ControlTask の IMU 読取と UiTask のタッチ/電源読取の併存は安全（確定事実 1）

### 3.2 タスク間データ受け渡し

- **制御 → UI**: `TelemetrySnapshot` 構造体（θ, θ̇, pitch_ref, v, I_cmd L/R, FSM state, dt 統計, 電圧, 温度, フラグ）を **seqlock**（`std::atomic<uint32_t>` セケンス + memcpy）で発行。書き手（制御）は非ブロッキング、読み手（UI）はリトライ
- **UI → 制御**: `ParamCommand`（種別+値）を FreeRTOS Queue で送信、ControlTask がループ先頭でドレイン。パラメータ実体は ControlTask ローカル所有
- 制御ループ内での `String`・ヒープ確保・ログ I/O は禁止

### 3.3 周期管理と監視

- `vTaskDelayUntil` 5ms（tick=1000Hz 確認済み → 5 tick 丁度）。**M5Stack Bala2（200Hz 公式ファーム）が同一手法**であり、高優先度タスク+専用コアでは十分な周期精度が得られる直接の実績。esp_timer+notify 方式は複雑さに見合う利得がないため不採用
- 各周期で `esp_timer_get_time()` により実測 dt を取得し推定器へ供給。dt 統計（max/p95）をスナップショットに含め、パネルで確認可能に
- **デッドライン監視**: dt > 1.5×周期 が連続 N 回（例 5 回）で FAULT（XL330規範 §8.3）

## 4. DYNAMIXEL バックエンド

### 4.1 初期化シーケンス（XL330規範 §7 厳守）

```
ping L/R → Model Number == 1190 検証（不一致なら FAULT・トルク ON しない）
→ Torque OFF → Operating Mode(11)=0 → Current Limit(38)=CFG.current_limit 設定 → Return Delay Time(9)=0
→ **Status Return Level(68)=2 を書込・読み戻し検証（配達証明（§4.2 の Status 応答付き WRITE）の成立前提。設定不能なら FAULT）**
→ PWM Limit(36) 読取・885 であることを検証 → Shutdown(63) 確認 → 全設定読み戻し検証（CFG と不一致なら FAULT）
→ Goal Current=0 書込（★モード変更で Goal Current が Current Limit 値へ自動セットされる公式挙動のため必須。省略すると Torque ON 瞬間に最大電流で駆動される）
→ Bus Watchdog(98)=CFG.bus_watchdog_raw 有効化 → 零電流心拍開始（Torque は OFF のまま IDLE へ）
```

**Torque ON は BALANCING 突入時のみ**（XL330規範 §7 の「Torque ON→Watchdog」順序からの意図的適合: トルクが有効な時間窓を制御が実際に必要な期間だけに限定する）。**`enter_balancing()`（名前付き突入契約 — FSM からの突入は必ずこの関数を経由し、手順の定義はここが唯一）**: Hardware Error Status==0 確認 **かつ raw98 正常（≠0xFF）確認（トリップ潜在時は §4.3 復旧を先に実行）** → **Torque Enable==0 のまま Goal Current=0 を検証付き書込（読み戻し==0 を確認。モード変更リセットや stale 値による非零パルスを Torque ON 前に遮断）** → Torque ON → Torque Enable 読み戻し検証 → Goal Current==0 再確認（冗長チェック） → 制御開始。mock テスト: アーム前に Goal Current が非零/stale のケースで非零のまま Torque ON に到達しないこと。退出時（FALLEN/DISARMED/FAULT/IDLE への遷移）は零電流→Torque OFF→読み戻し検証。

**安全定数はシーケンス中にリテラルで書かず、`app_config.h` の名前付きプロファイルを単一ソースとする**:

| プロファイル | Current Limit(38) | Bus Watchdog(98) | 用途 |
|---|---|---|---|
| `PROFILE_BRINGUP` | **150mA** | **raw 1（20ms）** | 符号試験・初回立ち上げ（§7 合格まで既定） |
| `PROFILE_NORMAL` | 900mA（TUNE） | **raw 1（20ms）** | 通常運転（Watchdog は常に 20ms、§4.2 の stale 予算と整合） |

起動時の読み戻し検証は選択中プロファイルの値と厳密比較し、不一致（例: ブリングアップ中に 900mA が読めた）は FAULT とする。

- 片輪でも検証失敗 → 両輪とも Torque ON しない（XL330規範 §19.2）
- Status Return Level(68)=2 は上記の通り**明示設定＋読み戻し検証**（デフォルト依存にしない。SRL=0/1 だと WRITE の Status が返らず配達検証が成立しないため、mock テストで SRL≠2 の状態では BALANCING に入れないことを検証）
- FAULT 復旧手順の文書化: Shutdown(63) 起因の HW エラーは REBOOT（電源再投入 or REBOOT 命令）が必須（Torque 再 ON では復帰しない、公式確認）

### 4.2 周期通信（@1Mbaud, 予算 5ms）

- **書き込み（トルク有効中は配達検証付き）**: **BALANCING 中の Goal Current(102, 2B) はアドレス指定の個別 WRITE（Status 応答あり、Status Return Level=2）を左右順に発行**する（~0.5ms。左右間の適用時差 ~250µs は本系の動特性では無視できる）。**配達成功の定義は `verified_write(id, addr, data)` ヘルパ 1 箇所に集約**し、成功条件は「**(1) 送信前に RX バッファの stale バイトをドレーン済み ∧ (2) 受信 Status が正しい Protocol 2.0 フレーミングで、パース済み送信元 ID が書込対象 ID と一致 ∧ (3) Status エラーバイト == 0**」とする。共有バスでは他サーボ／前トランザクションの遅延 OK 応答が次の WRITE の証明として誤消費されうるため、**ID・鮮度の検証なしに配達成功と判定してはならない**（`Master::write()`+`getLastStatusPacketError()` の組だけでは ID を検証できないため、パース済み Status の ID を公開する薄いラッパで実装する）。mock テスト: 他 ID からの遅延 OK Status／前トランザクションの残留 Status が配達証明として受理されないこと。⚠ライブラリの罠（ソース確認要）: Dynamixel2Arduino の `Master::write()` は **err_idx==0x80（ALERT=Hardware Error 通知）でも true を返す**ため、戻り値 bool を配達証明に使ってはならない。生の Status エラーバイト（`getLastStatusPacketError()` 相当）を毎 WRITE 後に検査し、**0x80（ALERT）を含む非零エラーは全て安全側経路（零電流→FAULT）へ**（ALERT は HW エラーの即時通知であり、低頻度ヘルスポーリングを待たず即応する）。mock テスト: err=0x80 および非零エラーの Status で非零電流が「検証済み」と扱われないこと。**トルク OFF 状態の零電流心拍は `syncWrite`**（軽量。配達保証は不要 — トルクが無いため）
- **読み取り**: アドレス 126 から 10B ブロック（Present Current/Velocity/Position）を左右 `syncRead`。Return Delay Time=0 設定後の往復は理論 ~0.5-0.8ms（要実測）。`fastSyncRead` は 0.8.1 に実装ありだが XL330 ファーム側の対応・安定性が未検証のため**まず標準 syncRead で実装し、実測で余裕がなければオプションフラグで切替**
- **ヘルス**: Hardware Error Status(70)/Present Input Voltage(144)/Present Temperature(146) を低頻度ラウンドロビン
- **タイムアウト規律**: 全トランザクションに明示タイムアウトを渡す（syncRead: 2ms、個別読取: 2ms）。ライブラリデフォルト（10-100ms）は 5ms 周期予算を破壊するため使用禁止
- **失敗処理は読取と書込で区別する（指令権喪失を最優先で扱う）**:
  - **読取失敗**（syncRead タイムアウト/CRC）: 当該周期は外側速度ループ・速度ガードを更新せず、Goal Current=0 を書く（書込経路は健在という前提のコースト。5ms は動的に無視できる）。連続 4 周期（20ms）で FAULT。前回帰還値の制御計算への再利用は禁止
  - **書込の信頼境界（ソース確認済み）**: `Master::syncWrite` は応答検証なしで true を返す（=キューしたことしか意味しない）。**Bus Watchdog が保証するのは「当該サーボへの全 instruction packet が途絶した」場合のみ**であり、syncRead が生きたまま書込パケットだけが失われる非対称故障では watchdog は発火しない。→ **トルク有効中は上記の Status 応答付き個別 WRITE を配達証明とする**: Status 未受信/エラー → 直ちに零電流の検証付き書込を試行し、それも未検証なら**即ラッチ FAULT**、零書込が検証できればコースト+**最初の未検証非零指令からの経過時間（壁時計、esp_timer 基準）が 10ms を超えたら、以降の非零書込を行う前にラッチ FAULT**（周期カウントではなく絶対時間で規定する — タイムアウトでサイクルが 5ms を超過しても保証が破れない）。**劣化サイクルの規律**: 1 周期内の DXL トランザクション時間に総予算（例 3ms）を設け、超過が見えたらヘルス読取など非必須トランザクションをスキップして安全系（零書込検証・raw98 評価）を優先する。これにより**未検証トルクの時間窓 ≤10ms（壁時計）** が不変条件になる。Bus Watchdog(20ms) は全通信沈黙（ホスト停止・断線）専用の最終防御、電流妥当性監視（§6、200ms）はさらに後段のバックストップという 3 層構造
  - mock テスト: 「書込パケットのみ落ち、syncRead は正常継続、raw98 は正常のまま」のケースで Status 欠落から壁時計 10ms 以内に FAULT へ至ること（パケット消失だけでなく**タイムアウト遅延（各トランザクションに実時間を注入）**でも検証する）
  - **安全遷移（FALLEN/DISARMED/FAULT への突入）では syncWrite を信用しない**: アドレス指定の個別書込（Goal Current=0）→ Goal Current / Torque Enable の**読み戻し検証**を行い、検証失敗時は反復する（FAULT の既存仕様を FALLEN/DISARMED 突入にも適用）
  - **バス検疫（fail-stop、最終手段。dxl_backend の最優先状態）**: トルクが有効でありうる状況で**検証付きの零電流書込／Torque OFF が達成できない**場合（書込経路の非対称故障）、FAULT ラッチと同時に `dxl_backend` を**検疫状態（`quarantine_until_us` ゲート）へ遷移**させる。検疫中は **バス全体への全 instruction 送信を禁止**する — 心拍・FAULT の零書込再送・syncRead/syncWrite・ヘルス・raw98・ブロードキャストの**全て**（半二重共有バス上で他 ID 宛てパケットが検疫対象サーボの Watchdog をリセットしない保証がないため、per-ID ではなくバス全体を沈黙させる）。窓は Watchdog 窓＋余裕（例 40ms）。**意図的な完全沈黙でサーボ自身の Watchdog を強制発火させ、モータを確実に停止させる**（e-manual: トリップでサーボは停止し Goal 値は read-only）
  - **不変条件の優先順位（正準定義はここ 1 箇所のみ）**: `検疫（沈黙） > save_in_progress（検証済み safe-off 下の保存停止、§9.1） > 心拍（毎周期書込） > その他の通信`。**検疫と save_in_progress はどちらも backend レベルの送信ゲート**であり、いずれかが有効な間は「全 FSM 状態で毎周期 Goal Current を書く」心拍不変条件と §6 FAULT の零書込再送は適用されない（backend が API レベルで全送信を拒否し、上位はブロックされる）。保存ゲートの正準名は `save_in_progress` とし、mock テストも同名のゲートに対して検証する。沈黙窓の経過後に Torque Enable / Goal Current / raw98 を読取って停止を確認し、FAULT ラッチは維持（復帰はリセットのみ）。FAULT 突入は「検証済み安全停止」と「検疫停止」を区別してログに記録する
  - mock テスト（検疫）: 「書込のみ喪失・syncRead 正常」で、FAULT 状態に入るだけでなく**バスイベントログ上で沈黙窓中に 1 パケットも送信されないこと**、および**サーボ側 Watchdog トリップ（モータ無効化）まで到達すること**の両方をアサートする
  - mock テスト: 「syncWrite が true を返すがパケットは落ちる」ポートで、電流監視/Watchdog 経路が機能すること
- **Bus Watchdog = 20ms（raw 1、最小値）とホスト心拍の不変条件**: **DXL 初期化完了後は FSM 状態によらず ControlTask が毎周期（5ms）Goal Current を書き続ける**（BALANCING 以外は常に 0。Torque OFF 中の零書込は無害）。ジャイロ静止校正（INITIALIZING）中もループは回し心拍を維持する。これにより Watchdog は常時武装のまま、正常時 4 周期分の余裕で、ホスト完全停止時の残留トルクを 20ms で断つ。**例外は 2 つのみ**: (1) バス検疫ゲート有効時、(2) 検証済み safe-off 下の `save_in_progress`（§9.1 の NVS 保存シーケンス）— いずれも backend レベル送信ゲート。**優先順位は前述の正準定義（検疫の項）1 箇所のみを参照**。保存中は DXL 送信ゼロ、保存後は raw98 点検/復旧完了まで enter_balancing() 不可（mock テストは §9.1 と同内容を §4 側の契約としても検証する）
- **Watchdog エラーのライフサイクル**: §4.3 の状態別遷移表（Torque Enable 読み戻しを根拠とする単一の表）に従う。これにより一時的なループ遅延後の再アーム不能を防ぎつつ、トルク有効中の指令権喪失は必ずラッチ FAULT にする
- **Watchdog トリップの検出契機（読む cadence を契約化）**: (a) **実測 dt > Watchdog 予算（20ms）を 1 回でも観測したら、Torque Enable の状態に関わらず次の書込の前に必ず左右の生アドレス 98 を 1 バイト読取**して §4.3 の表を評価する（トルク OFF 中のトリップはその場で自動復旧され、潜在化しない。デッドライン FAULT の「連続 N 回」条件とは独立。単発ストールでもトリップは見逃さない）。(a') **BALANCING 突入シーケンスに raw98 正常（≠0xFF）確認を含める**（§4.1: HW エラー確認と同時に実施。潜在トリップ状態のままトルク ON する経路を遮断）。**この dt 起因 raw98 読取が左右いずれかで失敗（タイムアウト/CRC/片側のみ応答）した場合、トルク有効中なら Goal Current を書く前にラッチ FAULT する（fail-closed。復旧経路に入れるのはトルク OFF 状態のみ）**。(b) 平常時も低頻度ヘルスラウンドロビンに raw98 を含める。mock テスト: 25ms の単発ストール後、syncWrite が true を返し続ける状況で raw98==0xFF を検出し表に従い FAULT すること / **raw98 読取がタイムアウト・片側のみ成功するケースで書込前に FAULT すること**
- 受入試験: 非零電流指令中に制御を意図的に停止（デバッグフック）→ 20ms 以内にモータが停止すること / 停止 → 20ms 超待機 → BtnC アームが復旧シーケンス経由で成功すること
- **符号正規化はバックエンドで完結**: 左右車輪は鏡像実装のため生値の符号が逆（現行コードでは L に −rpm を書いている）。`dxl_backend` は**指令（Goal Current）と帰還（Present Velocity/Position）の両方**に同一の符号定数 `s_L`/`s_R` を適用し、上位層には「前進 = 正」の SI 値のみを公開する。生値の平均を取ると左右が相殺して速度ループと速度ガードが無効化・逆転するため、この正規化は受入試験（§7-1）で必ず検証する
- 全 API 境界は SI 単位。raw 変換・2 の補数処理は `units.h` に集約（XL330規範 §6.2）

### 4.3 Bus Watchdog トリップの状態別遷移表（XL330規範 §7.2）

Bus Watchdog 発火後は Goal 値が読み取り専用になり、Bus Watchdog レジスタはエラー値を示す。

**アクセス経路の制約（ライブラリソース確認済み）**: Dynamixel2Arduino の共通テーブルは `{BUS_WATCHDOG, 98, 2}`（**2 バイト**）だが、XL330 の実レジスタは **1 バイト**。`read/writeControlTableItem(BUS_WATCHDOG, …)` は隣接アドレス 99 を巻き込み、読みは化け・書きは隣接破壊のリスクがある。**Bus Watchdog(98) へのアクセスは必ず生アドレス指定の 1 バイト read/write** で行い、**トリップ判定は raw==0xFF(255)** とする。native/mock テストにレジスタが 255 を返すケースを含める。

**判定は FSM 状態の想定ではなく、左右両モータの Torque Enable 読み戻し値を根拠**とし、以下の単一の表に従う（**集約規則: 自動復旧は「両輪とも読取成功かつ両輪とも 0」の場合のみ**）:

| BUS_WATCHDOG トリップ（raw==0xFF）検出時の状況 | 遷移 |
|---|---|
| L/R いずれかの Torque Enable 読み戻し == 1 | **ラッチ FAULT**（トルク有効中に指令権を失った） |
| L/R **両方**の読み戻しが成功し**両方** == 0 | **自動復旧**: Watchdog へ 0 書込（エラー解除）→ Goal Current==0 読み戻し確認 → Watchdog 再有効化 → 心拍再開。**復旧は左右とも完了して初めて成功**とみなす。発生をカウントし、60s 以内に 3 回で FAULT |
| L/R いずれかの読み戻しが失敗、または左右で状態不一致 | ラッチ FAULT（状態不明・非対称故障は安全側） |

FAULT からの復旧はリセットのみ（リセット後の INITIALIZING が同じ復旧書込を実施）。mock テストは BALANCING/IDLE/FALLEN/DISARMED の各状態に加え、**左右非対称（片輪トルク ON・片輪読取失敗・左右不一致）**を個別に検証する。

受入試験: 通信を意図的に停止（デバッグフックまたは配線断）し、モータ停止と上記復旧経路を確認する。

## 5. 姿勢推定（精査結果と設計）

### 5.1 現行アルゴリズムの妥当性判定

**判定: 「骨格は適、実装細部と軸検証が不適」**

- `atan2(accY, accZ)` による YZ 面傾斜角は、縦置き（重力≈+Y）の 1 自由度バランスに対して**軸選択として正しく、90° 近傍でも数値的に良条件**
- 1 次元 Kalman（角度+バイアス）という選択も用途に適合
- ただし以下を修正しないと 200Hz 電流モード制御には不適:
  1. dt を実測値に（固定 10ms → 実測）
  2. 起動時静止ジャイロバイアス校正（1-2s 平均）+ M5Unified 自動校正の凍結
  3. 加速度ノルムゲート（||a|−g| > 閾値で補正停止/減衰）— 電流モードは並進加速度が大きく出るため必須
  4. pitch を「平衡点 0 中心」へ再定義（`pitch_eq` を校正可能なオフセットとして分離）
  5. BMI270(v1.1) の軸向き・符号の実機検証（確定事実 3: M5Unified はリマップしない。v1.0 との PCB 実装向き等価性は未確認情報のため、設計は軸マップ+符号を config 化して両対応）
  6. gyro ODR 引き上げ（デフォルト 200Hz は 200Hz サンプリングとビート → 400Hz へ。`M5.Imu.getType()==imu_bmi270` のときのみ `M5.In_I2C` 経由で ACC_CONF(0x40)=200Hz 相当・GYR_CONF(0x42)=400Hz 相当をレジスタ直書き。値は BMI270 データシートで実装時に確定。v1.0 の MPU6886 はデフォルト ODR が高いため変更不要）

### 5.2 推定器実装（IMU 鮮度を契約に含める）

- `imu_backend` は M5Unified の `update()` 戻り値（sensor_mask）**のみ**を鮮度の根拠とし、鮮度情報付きの観測を返す:
  `ImuSample { accel[3], gyro[3], gyro_last_fresh_us, accel_last_fresh_us, gyro_fresh, accel_fresh }`
  - **注意（ソース確認済み）**: M5Unified の `_latest_micros`（`getImuData().usec`）は **sensor_mask==0 でも update() 呼出のたびに進む**ため、サンプル時刻として使用禁止。鮮度判定は sensor_mask のビットのみから導出し、センサ別の last-fresh タイムスタンプは `imu_backend` が自前で保持する（mask のビットが立った時刻を記録）
  - `update()` が新規データなしを返した場合、`getImuData()` は前回 raw を返すだけであり（確定事実 2 の裏面）、これを新鮮なサンプルとして推定器に流してはならない
  - native/mock テスト: 「usec が進み続けるが sensor_mask==0」のシーケンスで stale が正しく検出されることを検証
  - **gyro stale 時**: 当該周期は前回角速度で予測をホールドし stale カウンタ++。連続 5 周期（25ms）で FAULT（FAULT 条件に「IMU 期限切れ」を明記、XL330規範 §12.2 と整合）
  - **accel stale/ゲート時**: 補正のみスキップ（予測は継続）。危険ではないため FAULT にしない
  - gyro ODR 400Hz 化（§5.1-6）により通常運転では毎周期新鮮なサンプルが保証される
- 自前 1D 推定器 `attitude_estimator.h`（純ロジック・native テスト可能）:
  - 予測: θ += (gyro − bias)·dt（dt は実測値）
  - 補正: accel 傾斜角 atan2 との相補融合（固定ゲイン 2 状態 = 定常カルマンと等価）+ バイアス緩更新
  - accel ゲート付き（||a|−1g| > 0.3g で補正停止）。パラメータは時定数で指定（例 τ=1.0s、TUNE）
  - native テストに「サンプル欠落（stale）を含むシーケンスでの挙動」を含める
- 先行事例の推定器: BalaC=相補フィルタ(100Hz)、Bala2=Madgwick(200Hz)、HomeMadeGarbage=1D Kalman(400Hz) — いずれも実用。1 自由度バランスには 1D 相補+バイアス+ゲートが必要十分で、フル AHRS(Madgwick) は過剰
- TKJ Kalman Filter Library への依存は削除（lib_deps から外す）。理由: accel ゲート機能がない・native テスト不能・自前 ~40 行で等価以上の機能

## 6. 安全状態機械（XL330規範 §12 を教育用途に適合）

```
BOOT → INITIALIZING ─┬─(コミッショニング済み ∧ auto_arm)→ IDLE ⇄ BALANCING → FALLEN ─┐
                     │                              ▲   （※Torque ON は BALANCING のみ）│
                     └─(それ以外)→ DISARMED          └──────（静置検出で IDLE へ）───────┘
                                      ▲│ BtnC 長押し（ユーザ STOP/ARM トグル、IDLE ⇄ DISARMED）
            ── いずれの状態からも ──→ FAULT（ラッチ、リセットのみで復帰）
```

**ユーザ STOP（明示的な停止経路）**: BtnC 長押し（1s）でいつでも `DISARMED` へ遷移 — 零電流×2 → Torque OFF（読み戻し検証付き）→ ラッチ。再アームは再度の BtnC 長押しのみ（姿勢では復帰しない）。UI タスクからは STOP 要求フラグ（atomic）を立てるだけとし、実際の停止処理は ControlTask が次周期内（≤5ms）で実行する。
**STOP レイテンシ契約**: 端から端で「長押し判定 1s（意図的）+ UI ポーリング ≤33ms + 制御反映 ≤5ms」。ポーリング遅延が支配項にならないよう **UiTask の優先度は avatar 内部タスク（drawLoop=1, facialLoop=2）より高い 3 とし**、描画負荷でボタン検出が飢餓しないようにする。**BtnC は利便停止であり機能安全装置ではない**（Core2 のボタンはタッチ式で電気的に UI 経路に依存する）。ハード停止手段は電池ボックスの電源スイッチであることを README に明記する。mock テスト: STOP フラグ set → 次制御周期で零電流+Torque OFF 系列が開始されること。
**起動時アーム（コミッショニングゲート付き）**: 自動アーム（起立検出だけで BALANCING に入る動作）は**コミッショニング完了後にのみ許可**する。
- `PROFILE_BRINGUP`（既定・工場状態）: **常に明示アーム必須**（起動後 DISARMED。BtnC 長押しで IDLE へ。符号試験 §7 が完了するまで自動アームは選択不可）
- コミッショニング完了（§7 の符号試験合格をユーザがパネルで明示確認）後、`PROFILE_NORMAL` で `auto_arm_on_boot` が有効化可能になる。**既定値（true=電源 ON→立てると倒立開始という現行 UX / false=毎回 BtnC 長押し）はユーザ確認事項 Q5**。true を選ぶ場合は手が近い状態で再アームが起こり得るリスク受容を README に明記する。
- **コミッショニング記録は fail-closed の認可レコード**として NVS に保存する: `{schema_version, 対象プロファイル, 軸マップ/符号定数のチェックサム, pitch_eq 校正レコードの版, ユーザ確認済みフラグ}`。読込時にファームの現行値と照合し、**欠落・破損・版不一致・符号定数変更のいずれでも未コミッショニング扱い**（= 明示アーム必須へフォールバック）。§9.1 の validate と同様に純関数化して native テスト（欠落/破損/旧版/符号変更後）を行う。

- `INITIALIZING`: IMU/DXL 初期化・ジャイロ静止校正。**ループタイミング自己検査**（トルク OFF のまま全経路を ~2s 実行し dt p95 が予算内であること）を IDLE 遷移の条件に含める。失敗 → FAULT
- `IDLE`: **Torque OFF**・零電流心拍継続。起立検出（|θ| < 開始窓 かつ |θ̇| < 閾値 かつ |車輪速| < 閾値 が **1.0s 継続**）→ **`enter_balancing()`（唯一の突入契約。手順は §4.1 のみに定義し、ここには再掲しない）** を実行して BALANCING へ。FSM レベルの mock テストで IDLE→BALANCING が必ず enter_balancing() を経由することを検証
- `BALANCING`: 制御有効（**Torque ON はこの状態のみ**）。|θ| > 転倒閾値（初期 35°）→ FALLEN
- `FALLEN`: **Goal Current=0 を 2 回送信 → Torque OFF（読み戻し検証付き）**・全積分リセット。静置検出（起立姿勢 1.0s 静止 + 車輪停止 + Hardware Error Status==0）で IDLE へ復帰（トルクは OFF のまま。手で立て直すと IDLE の起立検出を経て再アームされる**現行 UX を維持**）
  - **エスカレーション**: 30s 以内に 3 回 FALLEN したら ラッチ FAULT へ（バウンス・発振・推定異常による無限再アームループを遮断）
  - XL330規範 §12.2「自動再アーム禁止」からの**意図的な逸脱**であることを明記する。根拠: 教育・デモ用途では転倒→手で立て直す→再開が主要ワークフローであり、Torque OFF + 静置 1s + 車輪停止 + HW エラー無しのゲートとエスカレーションで衝撃後の無認可再突入リスクを実用上十分に抑える
- `FAULT`（ラッチ）: HW エラー / 過熱 / 低電圧 / **DXL フィードバック期限切れ（連続 20ms）** / **IMU 期限切れ（連続 25ms）** / ループ超過連続 / 車輪過速度ハード超過 / Bus Watchdog トリップ検出 / **電流妥当性違反**。電流 0 ×2 → Torque OFF → avatar 表情で通知。自動再アーム禁止。復旧はリセットのみ（Shutdown(63) 起因は REBOOT 必須）
- **電流妥当性監視**（毎周期読む Present Current を不変条件として活用）: |I_present − I_cmd| > 300mA が 200ms 継続、または零指令中に |I_present| > 100mA が 200ms 継続 → FAULT（閾値・時間とも TUNE）。XL330 の Present Current は電源側電流で相電流ではないため（公式確認）、閾値は寛大に・dwell 付きで設定し、誤検知よりサーボ内部故障・符号設定誤り・零トルク経路故障の検出を狙う。native テストは mock 帰還で両条件を検証
- Bus Watchdog は最後の防御（ホスト側 FAULT 処理を代替しない）
- 受入試験: 転倒 → 電流 0 → Torque OFF → 静置なしでは再アームしないこと / 3 回連続転倒で FAULT ラッチすること / BtnC 長押しで BALANCING 中でも即 DISARMED になり姿勢では復帰しないこと（FSM native テストにも DISARMED 遷移を含める）

## 7. 実機符号試験（実装後の受入、XL330規範 §19.3）

コード変更だけでは確定できない項目。README に手順を記載し、ユーザが実施:

1. 車体を浮かせ、低電流（±30mA 程度）で両輪正指令 → 前進方向回転を確認（`s_L`/`s_R` 確定）。**同時に正規化後の Present Velocity が両輪とも正値になること**をパネル/シリアルで確認（帰還符号正規化の検証、§4.2）
2. 車体を前傾 → pitch が正方向へ変化、gyro レートの符号が d(pitch)/dt と一致することをパネル表示で確認（IMU 軸・符号確定）
3. v1.1(BMI270) で軸が v1.0 と異なる場合は `app_config.h` の軸マップ/符号定数のみで吸収できる構造とする
4. 200Hz タイミング実測: パネルの dt 統計（max/p95）が予算内であることを、IMU+syncRead+syncWrite の全経路有効状態で確認してから BALANCING を許可する（INITIALIZING の自己検査 §6 と二重化）

## 8. モジュール構成とテスト

```
src/
  main.cpp                     # 配線とタスク生成のみ
  app_config.h                 # 全定数 SSOT（ピン/ID/通信/周期/ゲイン初期値/制限/符号/軸マップ）
  core/                        # Arduino 非依存（native 単体テスト対象）
    units.h                    # SI⇔DXL raw 変換
    pid.h                      # PID + クランプ AW + D-on-measurement
    attitude_estimator.h       # 1D 相補推定器 + accel ゲート + バイアス
    balance_core.h             # カスケード制御 + ミキサ + 速度ガード + スルーレート
    safety_fsm.h               # 状態機械
  hw/
    imu_backend.{h,cpp}        # M5Unified ラッパ（update→getImuData、軸マップ、ODR 設定、静止校正）
    dxl_backend.{h,cpp}        # Dynamixel2Arduino ラッパ（初期化/syncWrite/syncRead/watchdog/ヘルス）
  tasks/
    control_task.{h,cpp}       # 200Hz ループ
    ui_task.{h,cpp}            # avatar + パネル
  shared/
    shared_state.{h,cpp}       # seqlock スナップショット + パラメータキュー
  TairinEye/TairinMouth        # 既存維持
test/
  test_core/                   # Unity（env:native）: units/pid/mixer/guard/fsm/estimator
platformio.ini                 # [env:native] 追加、Kalman Filter Library 依存削除
```

- 受入テスト（native）: ±1A↔raw±1000 / 0.229rpm 変換 / 2 の補数 / ミキサ倒立優先飽和 / 速度ガード（加速方向のみ縮小）/ FSM 遷移表（FALLEN 再アームゲート・3 回転倒エスカレーション含む）/ PID AW / 推定器収束と stale サンプル挙動 / **I²t ピーク制限器の連続上限収束** / **帰還符号正規化（鏡像実装で速度平均が相殺しないこと）** / **stale フィードバック時の零電流化**
- ビルド: `pio run -e m5stack-core2` + `pio test -e native` を CI 相当のローカルゲートに

## 9. UI / avatar

- 現行 UX 維持: BtnA=avatar 復帰、BtnB=チューニングパネル切替、手で立てると倒立再開
- **BtnC 長押し（1s）= ユーザ STOP/ARM トグル**（§6。DISARMED ⇄ IDLE。パネル/avatar に状態を明示）
- パネル項目を新体系へ: `pitch_eq trim` / `Kp` / `Ki` / `Kd`（+速度ループ ON/OFF）。ステップ幅は電流モードのゲインスケールに合わせ再設計。表示は deg 系に変換（内部 SI）
- 追加表示: FSM 状態、ループ dt max、DXL 電圧、バッテリ%
- avatar 表情を FSM 連動（IDLE=眠, BALANCING=通常, FAULT=困り顔）: 工数小なら実施

### 9.1 NVS（Preferences）保存の安全契約（Q3 採用時）

保存値を無検証で制御に流すと、破損・旧版レコードが Torque ON 後に初めて破綻する。以下を契約とする:

- **バージョン付きレコード**: `schema_version` + 全パラメータを 1 構造体で保存。版不一致は**全体を破棄**し既定値へ
- **読込時の範囲検証**: 各パラメータに `app_config.h` で定義する [min, max] を適用。1 つでも範囲外なら**全体を破棄**し既定値へ（部分採用しない）+ パネルへ「設定リセット」を表示
- **読込は Torque ON 前**（INITIALIZING 内）に完結させる
- **保存操作は明示的なユーザ操作のみ**（パネルの保存ボタン。自動保存しない）。校正値（pitch_eq・軸符号）の保存はゲインと別レコードにし、それぞれ明示操作
- **保存はトルク OFF 状態（IDLE/DISARMED/FALLEN）でのみ実行**: NVS/フラッシュ書込のレイテンシは 5ms ループと Watchdog(20ms) を破るため、BALANCING 中の保存要求は拒否しパネルに「停止してから保存」を表示する（ランタイム調整は RAM 上で有効、保存は停止後）。テスト: Torque ON 中の保存要求が拒否されること
- **保存の実行条件は論理状態ではなく検証済み safe-off**: 保存を開始してよいのは「**両輪の Torque Enable 読み戻し==0 かつ 両輪の Goal Current 読み戻し==0 かつ 停止系シーケンス（STOP/FALLEN/FAULT 突入・検疫）が進行中でない**」ことをその場で確認できた時のみ。アーム要求と保存要求は直列化し（同時受理しない）、条件不成立の保存要求は拒否・後回しにする。**`save_in_progress` 中は自動アーム（起立検出による IDLE→BALANCING）を含む全てのアーム経路をブロック**し、保存完了→raw98 点検/復旧→心拍再開が完了するまで `enter_balancing()` を呼べないようにする。mock テスト: `auto_arm_on_boot=true`・起立窓成立済みの IDLE で保存要求 → 保存シーケンス完全終了まで Torque ON が発生しないこと
- **保存＝予期された心拍停止**として明示的にシーケンス化する: `safe-off 検証 → 保存開始（心拍中断を許容）→ 保存完了 → raw98 点検 → トリップしていれば §4.3 の自動復旧 → 心拍再開`。トルク OFF 中のトリップは無害かつ復旧可能であり、この経路で潜在化しない。mock テスト: IDLE/DISARMED での心拍ギャップおよび NVS 相当の長時間ストールで raw98==0xFF になっても、アーム前に必ず復旧されること（潜在 0xFF 状態が BALANCING 突入に到達しないこと）/ **BALANCING→FALLEN 遷移中・STOP シーケンス進行中の保存要求が拒否されること**
- 検証ロジックは純関数（`validate_params()`）として native テスト対象（破損/旧版/欠落/境界値）

## 10. スコープ（ユーザ確認事項）

> **採用記録（2026-07-02）**: ユーザへ質問を提示したが応答タイムアウトのため、以下の推奨値で自動前進（全て config/フラグレベルで変更可能）: Q1=カスケード採用 / Q2=テレメトリはスコープ外 / Q3=NVS保存あり / Q4=yaw構造のみ / Q5=コミッショニング後 auto_arm=true。ユーザの異議があれば実装中でも反映する。

| # | 項目 | 推奨 |
|---|---|---|
| Q1 | 車輪速度外側ループ（カスケード） | **含める**（無効化フラグ付き。電流モードでは実用上必須） |
| Q2 | UDP テレメトリ（既存調査 `docs/telemetry_agent_debug_survey.md`） | **本ブランチ対象外**（スナップショット構造で将来対応可能にだけしておく） |
| Q3 | Preferences(NVS) へのゲイン・校正値保存 | **含める**（README 記載の将来機能。パネル調整が電源断で消える現状は調整効率が悪い） |
| Q4 | yaw 差動制御 | **構造のみ**（I_yaw=0。teleop 追加時に有効化） |
| Q5 | コミッショニング後の `auto_arm_on_boot` 既定値 | **true**（現行 UX 維持: 電源 ON→立てると倒立開始。ブリングアップ中は常に明示アーム必須なので初回安全性は担保済み） |

## 11. 作業手順（実装フェーズ）

1. コミット 1: `core/` 純ロジック + `app_config.h` + native テスト + platformio.ini 変更
2. コミット 2: `hw/` バックエンド + タスク + `main.cpp` 配線（この時点で実機書込可能）
3. コミット 3: UI 改修 + README 更新（符号試験・立ち上げ手順）
4. 各コミット前に `pio run -e m5stack-core2` + `pio test -e native` 通過
5. `/codex:review` ゲート 2 通過後、PR 作成（マージは人間）

実装は Sonnet5 サブエージェントへ委譲し、本セッション（Fable）は設計・監査・レビューに専念する。
