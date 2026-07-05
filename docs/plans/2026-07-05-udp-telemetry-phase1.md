# UDP テレメトリ Phase 1 実装計画

作成日: 2026-07-05
ブランチ: `feat/udp-telemetry-phase1`（`dev/current-mode-xl330-fable5` から作成、PR は同ブランチへ）
基礎資料: [`docs/telemetry_agent_debug_survey.md`](../telemetry_agent_debug_survey.md)（2026-06-29 調査）

## 1. 背景・目的

USB シリアルによる有線デバッグはケーブルの張力・重量が倒立動作そのものに影響するため、
実機挙動の観測手段として不適切である。M5Stack Core2 から Wi-Fi/UDP で PC へテレメトリを
one-way 送信し、PC 側で実験セッションとして蓄積・解析し、coding agent（Claude Code /
Codex）が保存済みログを読んでパラメータチューニングとデバッグを行える枠組みを整備する。

**ユーザ合意済みスコープ**（2026-07-05 ヒアリング）:

- Phase 1 のみ: firmware UDP 送信 + PC 受信ロガー + post-run 解析器 + agent ワークフロー文書
- one-way telemetry のみ（PC→Core2 コマンドチャネルは含めない。ゲイン変更は再ビルド+書込
  または実機タッチパネル）
- PC 側ツールも本リポジトリ（`kim-xps12/bsl-balancer`）内に配置し 1 PR で完結
- MCP サーバは次回以降の PR

## 2. 前提: survey 文書と現行実装の差分

survey（2026-06-29）は**旧構成**（10 ms loop / `calcPID()` / PS4 controller）を前提に書かれた。
現行firmware は FreeRTOS 再設計済みであり、以下の点で survey の前提を読み替える:

| survey の前提 | 現行実装 | 影響 |
|---|---|---|
| 10 ms 制御ループ、`calcPID()` に直書き回避 | 200 Hz `ControlTask`（core 0, prio 20）が毎周期 `shared::Snapshot` を seqlock 公開済み（`src/shared/shared_state.h`） | telemetry task は UiTask と同じ seqlock read パターンで Snapshot を読む。制御則・安全機構は無変更（観測カウンタの追加のみ、§3.1） |
| 「telemetry snapshot 構造体を追加する」 | Snapshot が既に存在し必要フィールドをほぼ網羅 | firmware 変更は telemetry task 追加 + main.cpp の task 生成のみ |
| PS4 controller (Bluetooth) との 2.4 GHz 共存リスク | 現行構成に PS4/Bluetooth は存在しない | 共存リスク消滅。検証項目から除外 |
| PID gains (Kp=50 等)・`rpm_cmd` 等の旧フィールド | Current Control Mode。電流指令 `i_cmd_l/r` [A]、`TuningParams`（kp/ki/kd/pitch_eq 等） | packet schema を現行の物理量に合わせ再定義 |

## 3. 設計

### 3.1 Firmware（新規: `src/tasks/telemetry_task.{h,cpp}`）

- **タスク配置**: core 1（APP_CPU）、priority 1、stack 8192、`vTaskDelayUntil` 周期
  `kTelemetryPeriodMs = 50`（20 Hz）。ControlTask（core 0, prio 20）とは物理的に分離。
  UiTask（prio 3）・avatar タスク（prio 1-2）より下位/同位の best-effort。
- **前提修正: seqlock の writer 側メモリ順序バグ**（ゲート1第9回指摘対応。Phase 1 の
  前提条件）: 現行 `SharedState::publish()`（`src/shared/shared_state.h:59-64`）は
  「奇数 seq を release store → plain memcpy → 偶数 seq を release store」だが、release
  store は**後続の**データ書込みが奇数 store より前に移動することを禁止しない。弱メモリ
  順序のデュアルコア環境では読み手が torn snapshot を受理し得る（読み手側の
  acquire+fence+再確認パターンは正しい）。telemetry はこの snapshot を Wi-Fi 安全ゲートに
  使うため、**publish/read を portMUX critical section 方式へ置き換える**（ゲート1
  第15回・第16回指摘で確定 — fence-only seqlock は plain `memcpy` の並行アクセスが C++
  規格上データレース（UB）のままであり、安全ゲートの基盤には採用しない）:
  - `publish()`/`read()` の双方を `portMUX_TYPE` spinlock（`taskENTER_CRITICAL` /
    `taskEXIT_CRITICAL`）で包む。クロスコア相互排他が保証され、コピーは ~100 B で数 µs、
    writer（ControlTask）の最悪待ちは読者側 critical section 長（数 µs）に有界。
  - `read()` は常に成功する設計になるが、API（bool 戻り値）と telemetry 側の
    read-fail/diagnostic セマンティクス（§3.1）は防御的に維持する（期待値ゼロの
    カウンタとして観測を残す）。freshness 述語（`loop_count` 前進）はこれまで通り
    Wi-Fi ゲートの必須条件。
  - 設計注記コメントに採用理由（データレース回避の形式保証）を明記し、native env での
    多スレッド stress テスト（torn read 不検出・両タスク進行）を追加する。
  この修正は UiTask の snapshot 読みの正しさも同時に改善する。
- **制御則・安全機構への変更ゼロ + 追加的（additive）計測のみ許可**: `control_task.cpp` /
  `shared_state.h` への変更は「既に計算済みの dt を分類してカウンタを増やし Snapshot で
  公開する」観測性追加に限定する。制御則・FSM・モータ指令経路・既存フィールドの意味は
  一切変更しない。具体的には `Snapshot` に以下の**新設の単調増加（monotonic）テレメトリ
  専用カウンタ**を追加し、ControlTask の publish 部で更新する:
  - `dt_hist_total[8]`: dt の累積ヒストグラム（uint32）。bin 境界は周期比
    `<1.02x, <1.05x, <1.1x, <1.2x, <1.3x, <1.5x, <2.0x, >=2.0x`（微小 jitter の劣化も
    検出できる分解能。§5 の受け入れ基準はこの bin 比率で定義する）
  - `overrun_total`: dt > 1.5x 周期の累積総回数（uint32）
  - `imu_stale_total`: IMU stale 読みの累積総回数（uint32）
  - `arm_pending`: **Idle → Balancing への遷移保留（直立ホールド進行中）全般**を示す bool。
    Idle への入り方（commissioned auto-arm / BtnC 手動アーム後の Idle）に依存せず、既存
    FSM が内部で持つ enter-balancing 判定（直立ホールドカウンタ）の進行状態をそのまま
    公開する（ゲート1第13回指摘対応: 手動アーム経路も同一機構なので定義を遷移保留全般に
    一般化。Wi-Fi 開始ゲートに使用、§3.1 Wi-Fi 管理）。
    **公開方法**（ゲート1第17回指摘対応）: ホールド進行状態は `SafetyFsm` の private
    実装内にあるため、**additive なアクセサ `SafetyFsm::armPending()` の追加を本計画で
    明示的に許可**する（遷移ロジック・タイミングは一切変更せず、既存内部カウンタを
    読み出すのみ。タイミングの複製実装は禁止）。ControlTask は毎周期これを Snapshot の
    `arm_pending` に転記する。host テスト: commissioned auto-arm と BtnC 手動アームの
    両ホールド窓で EnterBalancing まで `armPending()` が true を維持することを検証。

  **重要（ゲート1第2回指摘対応）**: 既存 `LoopState` の `overrun_count`・`read_stale_count`
  は「連続回数」カウンタであり正常サイクルでリセットされる。**これらを流用してはならない**。
  上記は別変数として新設し、リセット経路を持たないことを host テストで検証する
  （連続カウンタがリセットされる系列でも telemetry total の差分が正の正しい値になること、
  uint32 wrap は PC 側で modulo 差分処理）。

  **累積カウンタ方式の意図**: 20 Hz の telemetry サンプリングでも、隣接 packet 間の
  カウンタ差分を取れば **全 200 Hz サイクルの dt 分布を漏れなく被覆**できる
  （リセット不要・単調増加なので one-way 制約とも整合）。
- **seqlock read 失敗のセマンティクス**（明示定義、ゲート1第2回指摘反映）:
  - `seq` は**送信 datagram の通し番号**（emitted-datagram sequence）、`tick` は telemetry
    task の周期実行回数（attempt count）と定義する。read 失敗でも `tick` は進む。
  - seqlock read 成功 → 通常 packet（`snap_valid: true`）を生成・送信。
  - 失敗（リトライ上限到達）→ 古い snapshot の再送・偽装はせず、Wi-Fi 接続中であれば
    **診断 datagram** `{"v":1, "seq":.., "tick":.., "t_us":.., "dev":.., "fw":..,
    "snap_valid":false, "reason":"read_fail", "read_fail":N, "trunc":M}` を送信する
    （沈黙しない）。これにより PC 側は「seqlock 継続失敗」を「Wi-Fi 断・firmware
    クラッシュ・receiver 停止」と区別できる。
  - **schema v1 は `snap_valid` で判別する正式な 2 variant**（full / diagnostic）として
    定義する（ゲート1第3回指摘対応）。receiver は diagnostic variant も valid packet として
    受理・保存し、session 開始のトリガにもなり得る（`dev`/`fw` を含むため metadata pinning
    が成立する）。「first packet が snap_valid:false」の E2E テストを追加する。
  - **`trunc`（truncation 累積カウンタ）と `read_fail` は full/diagnostic 両 variant の必須
    フィールド**とし、diagnostic には `reason`（`"read_fail"` | `"trunc"`）を含める
    （ゲート1第4回指摘対応）。analyzer は `snap_valid:false` を reason 別に分類し、
    truncation を seqlock 競合や UDP 損失と混同しない。truncation を強制する host テスト
    （formatter 単体）と、diagnostic variant が receiver で受理・集計される E2E テストを
    追加する。
  - 通常 packet には Snapshot の `loop_count` を必ず含める。PC 側の異常判別:
    `seq` ギャップ = UDP 損失 / `tick`−`seq` 乖離と `snap_valid:false` = seqlock 競合 /
    `loop` 差分ゼロ = ControlTask ストール / 完全沈黙 = Wi-Fi 断か電源断。
- **Wi-Fi 管理**: telemetry task 内の非ブロッキング状態機械
  （`IDLE → CONNECTING → CONNECTED → LOST → バックオフ後 IDLE`）。`WiFi.begin()` の発行は
  起動時を含め**すべて下記の開始条件（fresh 非 Balancing snapshot）を満たした時のみ**。
  以降は `WiFi.status()` ポーリング。接続待ちでループをブロックしない。
  未接続時は送信スキップのみ（制御は Wi-Fi 状態と完全独立）。
  **ライブラリ既定動作の無効化と限界**（ゲート1第3回・第7回指摘対応。
  framework-arduinoespressif32@3.20016.0 の実ソースで検証済み）:
  - `WiFi.begin()` 前に必ず `WiFi.persistent(false)` と `WiFi.setAutoReconnect(false)` を
    明示する（flash-backed config 更新と定常自動再接続の無効化）。
  - **ただし** `WiFiGenericClass::_eventCallback`（WiFiGeneric.cpp:1079-1094）は**初回の
    非自発的 STA 切断時に `first_connect` one-shot として autoReconnect 設定に関係なく**
    `WiFi.disconnect(); WiFi.begin();` を発行する。自発的切断（ASSOC_LEAVE）は除外される
    ため、telemetry FSM 自身の abort `disconnect()` はこれを発火させない。
  - よって「Balancing 中にライブラリ側 Wi-Fi 操作が一切起きない」は現行 Arduino コアでは
    保証不能であり、不変条件を**有界キャンセル**として定義する。検出は
    **イベント駆動ガード**で行う（ゲート1第8回指摘対応。`WiFi.status()` の値だけでは
    「one-shot 再接続が in-flight」と「再接続なしの単純 AP 喪失」を区別できないため）:
    - telemetry task は `WiFi.onEvent()` でイベントリスナを登録し、GOT_IP・
      STA_DISCONNECTED（reason 付き）を追跡する。リスナ内では **atomic フラグの更新のみ**
      行い、Wi-Fi API は呼ばない（イベントタスク内での再入を避ける）。
    - リスナはコア内部の `first_connect` one-shot の armed/burnt 状態をミラーする:
      非自発的切断（reason != ASSOC_LEAVE）を初めて観測した時点で「ライブラリ one-shot
      再接続が発火した」ことが確定するので `lib_reconnect_pending`（in-flight 状態）を
      立てる。
    - **`lib_reconnect_pending` のライフサイクル**（ゲート1第10回・第12回指摘対応。
      sticky 化による誤 abort・abort 反復の防止）: 以下のいずれかで**必ずクリア**する —
      (a) GOT_IP（one-shot 再接続の成功）、(b) one-shot begin 後の再度の STA_DISCONNECTED
      （one-shot 失敗。first_connect burnt + autoReconnect off により以降の再接続は
      発生しない = in-flight 終了）、(c) タイムアウト（フォールバック、目安 15 s）、
      (d) telemetry FSM 自身が `!WIFI_QUIET` で容認した begin/disconnect の発行時、
      (e) **cancel-watchdog による abort の完了確認時**（§上記「abort の完了確認と
      リトライ」— 発行のみではクリアしない。abort 反復は `abort_in_flight` 状態で抑止）。
    - telemetry task は毎 tick でフラグを確認し、**`WIFI_QUIET` かつ
      `lib_reconnect_pending`（in-flight）**の場合、`WiFi.disconnect()`
      でキャンセルし、完了確認後に (e) でフラグをクリアする（CONNECTING 中の abort と同一の
      fail-closed 述語。Balancing だけでなく arm_pending・snapshot 不明でも中断する —
      「Idle 接続済み → 直立ホールド中に AP 喪失 → one-shot 発火」の経路をホールド中に
      塞ぐ。one-shot が既に burnt かつ autoReconnect off の通常 AP 喪失では begin は
      発火しないため、abort も発行しない — 誤 positive なし）。
    - host テスト追加: 「Idle 中に one-shot が完了/失敗 → その後 arm」で Balancing 中の
      `disconnect()` が発行されないこと。「直立ホールド（arm_pending）中の AP 喪失 →
      one-shot 発火」で abort が 1 回だけ発行されフラグがクリアされること。
    - ライブラリ側バーストの継続時間は最大約 1 telemetry 周期（50 ms）+ disconnect 処理
      時間に有界化される。
  - ベンチ検証（§5）に「GOT_IP 後の AP 喪失 × Balancing 中」を追加し、
    **one-shot 発火 → 有界キャンセル**のケースと **one-shot burnt 済みの通常 AP 喪失 →
    Wi-Fi 操作なし**のケースを区別して確認する。
  **重い Wi-Fi/UDP 活動の抑止（不変条件）**: 抑止対象は API 呼び出しだけでなく
  **接続処理そのもの（in-flight CONNECTING の scan/認証バースト含む）**と定義する
  （ゲート1第4回指摘対応。auto-arm 構成では起動直後の初回接続が Balancing 開始に食い込み
  得るため）。

  **単一の禁止述語 `WIFI_QUIET`**（ゲート1第12回指摘対応 — 条件の伝播漏れを防ぐため
  1 箇所で定義し、以下すべての規則はこれを参照する）:

  ```
  WIFI_QUIET :=
    snapshot が fresh でない（read 失敗 / 未 publish / loop_count 停滞）
    または snapshot.fsm == Balancing
    または snapshot.arm_pending == true（直立ホールドによる auto-arm 進行中）
  ```

  - **操作クラスの分離**（ゲート1第16回指摘対応 — 開始系と中断系で許可条件が異なる）:
    - **start-class**（`WiFi.begin()`・STA への mode 変更・UDP socket 作成/再作成/
      `stop()`・初回 `beginPacket()`（prewarm））: **`!WIFI_QUIET` の場合のみ**発行できる。
    - **abort-class**（abort の `WiFi.disconnect()`、リトライ上限後の終端
      `WiFi.mode(WIFI_OFF)`）: **`abort_in_flight` 状態でのみ**発行できる。`WIFI_QUIET` 中に
      許容される Wi-Fi 操作は abort-class のみであり、radio を静穏化する方向の操作である
      こと（start-class の再発行でないこと）をテストで検証する。
  - **abort の完了確認とリトライ**（ゲート1第14回指摘対応 — `disconnect()` は失敗し得る
    非同期 API であり、発行しただけでガードを解除してはならない）:
    - disconnect 発行で `abort_in_flight` 状態に入る。in-flight ガード
      （`lib_reconnect_pending` / CONNECTING 状態）は**キャンセル完了の確認まで維持**する。
    - **完了確認はイベント epoch ベース**（ゲート1第15回指摘対応 — `WiFi.status()` は
      隠れた one-shot begin の実行**前**に非接続値へ更新されるため、status 値だけでは
      「接続活動なし」を証明できない）: イベントリスナは受信イベントごとに単調増加の
      epoch カウンタを付与する。abort 発行時に現在 epoch を記録し、**発行 epoch より後に
      観測された STA_DISCONNECTED（自発的切断 = ASSOC_LEAVE 系）または STA_STOP** のみを
      完了条件とする。abort 発行前から存在する非接続 status では完了と見なさない。
      （`lib_reconnect_pending` クリア条件 (e) は「abort 発行直後」ではなく
      「**post-abort イベントによる完了確認時**」と読み替える）。
    - 発行後 N tick（目安 3 tick = 150 ms）以内に確認できない、または `disconnect()` が
      false を返した場合は、`WIFI_QUIET` 継続中に限り**有界リトライ**（最大 3 回）する。
    - **リトライ上限到達時の終端処理 = radio 停止**（ゲート1第15回指摘対応 — API 呼び出しを
      やめるだけでは走行中の scan/auth バーストは止まらない）: `WiFi.mode(WIFI_OFF)`
      （`esp_wifi_stop` 相当）を発行し、post-abort epoch の STA_STOP イベントまたは
      `WL_NO_SHIELD` status で **radio 停止の完了を確認**する。停止確認まで tick ごとに
      有界リトライ（idempotent）。確認後に **`wifi_abort_failed` フォールトを latch** し、
      以降 telemetry の Wi-Fi 活動を全停止する（begin/prewarm/送信とも再開しない。制御には
      影響しない）。フォールトは累積カウンタとして diagnostic variant / UI 経由で可観測に
      する。radio 停止すら確認できない残余ケース（Wi-Fi ドライバの恒久的 wedge）は発生
      確率・影響を文書化した受容残余リスクとする。
    - host テスト追加: `disconnect()` の失敗・遅延を強制し、CONNECTING 経路とライブラリ
      one-shot 経路の両方で「ガード維持 → リトライ → WIFI_OFF → 停止確認 → latch」の系列を
      検証する。「status が非接続だが one-shot begin が継続中」のケースで早期クリアが
      起きないこと、latch 後に STA_CONNECTED/GOT_IP が発生しないこと（計装）も検証する。
  - **snapshot freshness の機械的定義**（ゲート1第6回指摘対応）: telemetry FSM は前回
    観測した `loop_count` を保持し、「fresh」とは **read が成功し、かつ `loop_count` が
    前回観測値から前進している**ことと定義する。`SharedState` の未 publish 既定バッファ
    （`loop_count == 0` で不変）はこの述語で自動的に reject される（Initializing 表示の
    既定値を非 Balancing と誤認して begin する経路を塞ぐ）。host テストで
    no-publish / stale-loop / read-fail / Balancing / fresh 非 Balancing の 5 ケースを検証。
  - **接続開始（begin/retry）の条件**: `WiFi.begin()`/mode 変更は、**`!WIFI_QUIET`**
    （= fresh かつ非 Balancing かつ `arm_pending == false`）の snapshot を確認した場合のみ
    発行する（初回 begin も再接続と同一ルール）。fresh でない場合は開始しない（開始方向の
    fail-closed）。
    **アーム遷移レースの遮断**（ゲート1第11回・第13回指摘対応）: Idle から直立ホールド
    経由で Balancing へ遷移する経路（commissioned auto-arm・BtnC 手動アーム後の Idle の
    **両方**）では、Idle での begin 容認だけでは接続 in-flight のまま Balancing に突入し得る。
    `arm_pending`（Idle → Balancing 遷移保留全般、§3.1 Snapshot 定義）を begin 禁止条件に
    加えることで、ホールド開始以降は新規接続が発行されず、in-flight 接続は abort 規則
    （`WIFI_QUIET` 成立で abort）によりホールド時間分の先行マージンをもって Balancing
    開始前に中断される。ホールド時間が telemetry 検出遅延（≦約 50 ms）+ disconnect 処理
    より短い構成では既存の有界キャンセルが backstop となる（ベンチで実測）。
    host テスト追加（両経路）: 「commissioned Idle → 直立ホールド → Balancing」および
    「Disarmed → BtnC 手動アーム → Idle → 直立ホールド → Balancing」の各遷移で、
    arm_pending 以降 begin/prewarm が発行されず、in-flight 接続が Balancing 開始前に
    abort されること。
  - **CONNECTING 中の fail-closed は abort 方向**（ゲート1第5回指摘対応）: 接続試行が
    in-flight の間に **`WIFI_QUIET` が成立**（Balancing / `arm_pending` / snapshot read
    失敗・停滞のいずれか）
    したら、telemetry FSM は即座に 1 回の `WiFi.disconnect()` で接続試行を中断する
    （abort は安全側操作であり、状態不明時に接続バーストを放置する方が危険。この
    disconnect が Balancing 中に許容される唯一の Wi-Fi 操作）。中断後は新鮮な非 Balancing
    snapshot を確認できるまで再開しない。
  - **検出遅延の明示**: telemetry task は 20 Hz（50 ms 周期）で snapshot を確認するため、
    arm から abort までの最大遅延は約 1 telemetry 周期 + disconnect 処理時間である。
    この遅延は仕様として受容し、host テスト（read-fail while CONNECTING → abort 発行）と
    ベンチ測定（abort 最大遅延）で検証する。
  - 接続済み（CONNECTED）なら Balancing 中も送信は継続する（UDP send は接続処理と異なり
    軽量で、影響は dt ヒストグラムで常時監視される）。
  - 帰結として「auto-arm で Wi-Fi 接続前に Balancing が始まると、非 Balancing に戻るまで
    telemetry は開始されない」仕様となる。実験運用では Idle（横倒し/スタンド上）で接続
    完了を待ってから倒立を開始する旨を workflow 文書に明記する。
  実機検証（§5）: ベンチ（スタンド上、検証時のみ USB 併用可）のデバッグ計装ビルドで、
  ケースを分けて確認する（ゲート1第6回指摘対応）:
  - **非 CONNECTING で Balancing 中**: `WiFi.begin()`/mode 変更/`disconnect()` が一切
    発行されないこと。
  - **CONNECTING 中に Balancing（または snapshot 不明）へ遷移**: **ちょうど 1 回の**
    `WiFi.disconnect()`（abort）が発行され、`begin()` の retry は発行されず、fresh 非
    Balancing snapshot を観測するまで以降の Wi-Fi 操作がないこと。
- **送信**: `snprintf` で固定長バッファに JSON 1 行を生成し
  `WiFiUDP::beginPacket(pc_ip, port) / write / endPacket`。heap 確保・`String` 禁止。
  **UDP socket/バッファのライフサイクル**（ゲート1第7回指摘対応）: 現行実装の
  `WiFiUDP::beginPacket()` は初回呼び出し時に socket 作成と 1460 B tx バッファの heap 確保を
  **遅延実行**する。よって socket 作成・破棄（`stop()`）・初回 beginPacket は begin/
  disconnect と同じ「重い操作」に分類し、同一の不変条件（fresh 非 Balancing 必須）下に
  置く。**prewarm と `udp_ready` 状態**（ゲート1第8回・第12回指摘反映）: CONNECTED 遷移を
  **`!WIFI_QUIET`**（fresh・非 Balancing・非 arm_pending、§3.1 の単一述語）の snapshot で
  確認した直後に最初の packet 送信を試行し、**`beginPacket`（socket 作成+tx バッファ
  malloc）・`write`・`endPacket` の全戻り値が成功した場合のみ** `udp_ready = true` とする。
  prewarm のいずれかが失敗（beginPacket が 0 を返す等）したら `udp_ready = false` のまま
  送信を全スキップし、`!WIFI_QUIET` を確認できた tick でのみ prewarm を再試行する
  （`write` は tx バッファを deref するため、beginPacket 失敗後に write へ進まないことを
  必須とする）。Balancing 中の送信は `udp_ready` の場合のみ行い、確保済み資源の再利用に
  限る。Balancing 中に `endPacket` が失敗した場合は packet を破棄・`trunc` とは別の送信
  失敗カウンタに計上して継続し、`udp_ready` を維持したまま socket 再作成は `!WIFI_QUIET`
  まで保留する。host テストに「beginPacket の malloc/socket 失敗を強制 → write に進まず
  udp_ready にならない」「arm_pending 中に CONNECTED 遷移 → prewarm が保留される」ケースを
  追加。計装ビルドで「`fsm == Balancing || arm_pending` 中に malloc/socket 作成が発生
  しない」ことを確認する（§5）。
  **バッファ長・truncation 対策**（ゲート1第3回指摘対応）: 全フィールドの float 精度・
  整数幅を明示指定した上で最大長を host テストで固定し、バッファは **1024 B** とする
  （既定 %f 精度では例示 packet が 537 B 超になる実測に基づく余裕設計）。`snprintf` の
  戻り値を必ず検査し、バッファ長以上（truncation）なら**送信せず** telemetry ローカルの
  `trunc` 累積カウンタを増やし、diagnostic variant（`reason:"trunc"`）で報告する
  （不正 JSON を送らない）。`trunc` は schema v1 の必須フィールド（§上記）。
- **packet schema v1**（すべて Snapshot 由来 + telemetry ローカル値）:

```json
{
  "v": 1,
  "seq": 1234, "tick": 1236, "snap_valid": true,
  "t_us": 987654321,
  "dev": "core2-a4cf", "fw": "b0d8535",
  "fsm": 2, "fault": 0,
  "theta": 0.012, "theta_dot": -0.34, "theta_ref": 0.002,
  "v_body": 0.05, "om_l": 3.1, "om_r": 3.0,
  "i_cmd_l": 0.12, "i_cmd_r": 0.11, "i_mea_l": 0.10, "i_mea_r": 0.09,
  "dt_us": 5012, "dt_max_us": 5480, "loop": 123456,
  "dt_h": [123000, 300, 80, 15, 4, 1, 0, 0], "ovr": 0, "stale": 0,
  "read_fail": 0, "trunc": 0,
  "volt": 7.4, "temp": 34.0,
  "sat": false, "i2t": false,
  "kp": 1.5, "ki": 0.0, "kd": 0.08, "eq": 0.0,
  "rssi": -55
}
```

  `dt_h`（累積 dt ヒストグラム 8 bin）・`ovr`（`overrun_total`）・`stale`（`imu_stale_total`）
  は ControlTask が公開する monotonic カウンタ、`read_fail` は telemetry task ローカルの
  累積 seqlock read 失敗数。いずれも PC 側で隣接 packet の差分（uint32 modulo）を取って
  区間統計に変換する。`dev` は eFuse MAC 由来のデバイス ID、`fw` はビルド時に埋め込む
  git 短縮ハッシュ（実験ログと firmware 版数の突合用）。

  単位は firmware 内部単位（rad, rad/s, m/s, A, V, °C, µs）をそのまま送る。角度の deg 変換等は
  PC 側の責務。`seq`/`t_us`（`esp_timer_get_time()`）は telemetry task が付与。
- **認証情報・宛先設定**: `include/wifi_secrets.h`（`.gitignore` 追加）に SSID / password /
  PC IP / UDP port を定義。`include/wifi_secrets.h.example`（ダミー値入り）をテンプレート
  としてコミット。
- **secrets 不在時のビルド分岐**（ゲート1指摘対応）: `telemetry_task.h` は secrets の有無に
  よらず**常に同一の安定 API**（`tasks::startTelemetryTask(shared_state)` と
  `tasks::telemetryEnabled()`）を提供する。`__has_include("wifi_secrets.h")` の分岐は
  telemetry_task.cpp 内部に閉じ込め、不在時は task を生成しない no-op 実装になる。
  main.cpp は無条件に `startTelemetryTask()` を呼ぶ（呼び出し側に #ifdef を置かない）。
  **有効経路が未検証のままにならないよう**、検証手順（§5）で
  `cp include/wifi_secrets.h.example include/wifi_secrets.h` によるダミー有効ビルドを必須化
  する（enabled/disabled 両経路のコンパイルを常に再現可能にする）。
- **ライブラリ**: `WiFi.h` / `WiFiUdp.h` は arduino-esp32 core 同梱。`lib_deps` 変更なし。
- **platform/framework バージョン固定**（ゲート1第9回・第10回指摘対応）: 本設計の Wi-Fi
  安全ガードは arduino-esp32 の `_eventCallback` 実装（first_connect one-shot 挙動）に
  依存する。本ワークスペースで `pio pkg list -e m5stack-core2` が解決するのは
  **espressif32 6.12.0 → framework-arduinoespressif32 3.20017.241212** であり（検証済み
  ソース 3.20016.0 とは別バージョン。ただし両者の `WiFiGeneric.cpp` の該当ロジックは
  diff で同一であることを確認済み）、pin はこの実解決バージョンに合わせる:
  - `platformio.ini` に `platform = espressif32@6.12.0` を明記。
  - 実装時に解決された `framework-arduinoespressif32` のソースで first_connect one-shot
    挙動（非自発的切断の初回で無条件 `disconnect(); begin();`、ASSOC_LEAVE 除外）を再確認
    し、確認したバージョン番号を計画/コードコメントに記録する。
  - `scripts/verify_builds.sh` は **platform と framework の両方**の解決済みバージョンを
    assert し、不一致なら fail する（platform 更新時に one-shot 挙動の再確認を強制）。

**既知のリスクと対応**:

| リスク | 対応 |
|---|---|
| ESP32 の WiFi task は core 0・prio 23 で ControlTask（prio 20）より高優先。送信・再接続時に制御 jitter の可能性 | 20 Hz・小packet で開始。**累積 dt ヒストグラム + overrun カウンタで全 200 Hz サイクルの jitter を観測可能にする**（間引きサンプリングでは検出不能というゲート1指摘への対応）。既存の overrun 検出（1.5×5 連続で FAULT）が安全網。実機検証で telemetry 有効/無効の dt 分布比較を必須化 |
| Wi-Fi 接続処理（scan/connect）は core 0 負荷バーストが大きい | 接続は task 起動直後（Initializing/Idle 中）に開始。**Balancing 中は再接続試行を発行しない**（保守側デフォルト、§3.1） |
| dt_max が lifetime 値でリセットされない（既知の監査指摘） | 累積 dt ヒストグラムの差分により PC 側で区間ごとの dt 分布を再構成できるため、lifetime dt_max への依存は補助指標に格下げされる |
| seqlock read 失敗・snapshot 不整合がログを汚染 | read 失敗時は送信スキップ + `read_fail` 累積カウンタ、`loop_count` 同梱により PC 側で UDP 損失/読取失敗/制御ストールを判別（§3.1） |

### 3.2 PC 側ツール（新規: `tools/telemetry/`、Python 3.11+、標準ライブラリのみ）

依存ゼロ（stdlib only）で `python3` 直接実行可能にする。構成:

```
tools/telemetry/
  bsl_telemetry/
    __init__.py
    schema.py        # packet schema v1 定義・validation（必須キー・型・version）
    receiver.py      # UDP 受信 → session dir 作成 → raw.jsonl append
    analyzer.py      # post-run: loss/dt統計/event検出 → events.jsonl, summary.json
    report.py        # session の markdown digest を stdout へ（agent/人間向け）
    simulator.py     # 合成 packet 送信（実機なしで receiver を検証）
  tests/
    test_schema.py
    test_analyzer.py
    test_e2e.py      # simulator→receiver→analyzer→report をループバックで通す
  README.md          # 使い方（受信・解析・レポートのコマンド例）
```

- **session layout**（survey 準拠）: `logs/telemetry/<session_id>/` に `metadata.json`,
  `raw.jsonl`, `events.jsonl`, `summary.json`。`logs/` は `.gitignore` 追加。
- **receiver**: `0.0.0.0:<port>` bind。最初の valid packet で session 自動開始、無通信
  N 秒（既定 10 s）で自動 close、SIGINT でも summary 書き出しまで実施。受信時処理は
  parse → `host_receive_ns` 付与 → schema validation → append のみ（解析は後段分離）。
- **受信の信頼境界**（ゲート1指摘対応・第2回で精緻化）: ログはチューニング判断の入力に
  なるため、無条件受け入れにしない。ただし**認証ではなく誤混入検出**が Phase 1 の設計点
  であることを明示する。
  - 既定で **first-packet device pinning**: session 開始 packet の source IP に固定し、
    以降それ以外の source からの packet は保存せず拒否。
  - `--device-ip <ip>` で明示 allowlist 指定も可能（pinning より優先）。
  - packet の `dev`（デバイス ID）・`fw`（firmware ハッシュ）を `metadata.json` に記録し、
    session 途中で値が変わったら event として記録（別デバイス・別ビルドの混入検出）。
  - 拒否 packet・validation 失敗は件数・source 別に集計し `summary.json` の
    `rejected_packets` に記録（黙って捨てない）。
  - **threat model の明示**: source IP pinning は誤設定・複数デバイス・simulator の混入を
    防ぐ仕組みであり、同一 LAN 上の能動的攻撃者（IP spoofing・偽 packet 注入）への認証
    境界ではない。よって全 session の `metadata.json` に `"authenticated": false` を必ず
    記録し、workflow 文書に「本 framework は信頼できる私有 LAN での運用を前提とする。
    敵対的 LAN では HMAC 対応（フォローアップ）まで使用しない」と明記する。
  - packet への HMAC 署名（firmware: mbedtls HMAC-SHA256 / PC: stdlib `hmac`、共有鍵は
    `wifi_secrets.h` と receiver 引数）は Phase 1 では見送り、上記文書化で信頼境界を
    明示することで代替する。設計概要をフォローアップ課題として workflow 文書に残す。
- **analyzer**: raw.jsonl から再生成可能な純関数として実装。
  - loss: `seq` ギャップ検出・欠落率
  - タイミング: `dt_us`（サンプル値）に加え、**`dt_h`/`ovr`/`stale`/`read_fail` の隣接差分
    から全 200 Hz サイクルを被覆する区間 dt 分布・overrun・読取異常を再構成**
    （uint32 wrap は modulo 差分。負の差分や `seq`/`t_us`/`loop` の巻き戻りはデバイス再起動
    と判定し、session 内 reboot event として記録して差分計算を再アンカーする）
  - 異常判別: `seq` ギャップ = UDP 損失、`snap_valid:false`/`tick`−`seq` 乖離 = seqlock
    競合、`loop` 差分ゼロ = ControlTask ストール、完全沈黙 = Wi-Fi 断/電源断、を区別して
    レポート
  - event 検出: `fsm`/`fault` 遷移（転倒・FAULT）、`sat`/`i2t` の連続区間、overrun 増加区間
  - `summary.json` に `recommended_windows`（agent が最初に見るべき seq 区間）を含める
- **report**: summary + events + 推奨 window の raw 抜粋を markdown で stdout 出力。
  coding agent はこれを読むか、`raw.jsonl` を直接 bounded read する。
- **実行例**:

```bash
python3 tools/telemetry/bsl_telemetry/receiver.py --port 45678 --log-root logs/telemetry
python3 tools/telemetry/bsl_telemetry/analyzer.py logs/telemetry/<session_id>
python3 tools/telemetry/bsl_telemetry/report.py logs/telemetry/<session_id>
python3 tools/telemetry/bsl_telemetry/simulator.py --port 45678 --scenario fall
```

### 3.3 ドキュメント

- `docs/telemetry_workflow.md`（新規）: coding agent 向け実験→観測→チューニングの手順書。
  - 実験セッションの回し方（receiver 起動 → 実機実験 → analyzer → report）
  - agent への引き渡し順序（metadata → summary → events → 推奨 window → bounded raw）
  - パラメータ変更の手段（`app_config.h` 編集+再ビルド / 実機タッチパネル+SAVE）と
    実験条件の記録方法（metadata の tags/notes）
  - 実機検証チェックリスト（telemetry 有効/無効の dt_max・overrun 比較、20 Hz loss 率、
    receiver 停止時に制御が継続すること）
- `docs/telemetry_agent_debug_survey.md` を本 PR でコミット（現在 untracked。基礎資料として
  記録。ただし旧構成前提である旨の注記を冒頭に 1 行追記）

## 4. 実装ステップ（Sonnet サブエージェント委譲単位）

ファイル集合が互いに素なため並行実行可能:

1. **FW**: `telemetry_task.{h,cpp}` + `wifi_secrets.h.example` + `main.cpp` への task 生成追加
   + `app_config.h` への telemetry 定数追加 + `Snapshot`/ControlTask publish 部への累積
   カウンタ・`arm_pending` 追加（additive のみ、§3.1）
   + **`SharedState::publish()/read()` の portMUX critical section 置換（§3.1 前提修正。
   fence 方式は不採用 — 旧記述の「writer fence 修正」は本方式に置き換え済み）**
   + **`SafetyFsm` への additive アクセサ `armPending()` 追加**（直立ホールド進行中を返す。
   FSM の遷移ロジック自体は変更しない）
   + `platformio.ini` の platform バージョン固定 + `scripts/verify_builds.sh`
   + 対応 host テスト + `.gitignore` 更新
2. **PC**: `tools/telemetry/` 一式 + unittest
3. **Docs**: `telemetry_workflow.md` + survey 注記 + `README.md`

統合後に監査（Fable）→ ゲート 2（`/codex:review`）→ PR。

## 5. 検証（PR 前に必須）

- `pio run -e m5stack-core2` ビルド成功 — **enabled/disabled 両経路をローカル状態に依存せず
  （hermetic に）検証**する。`scripts/verify_builds.sh` を新設し、
  (1) 既存 `include/wifi_secrets.h` を退避 → secrets なしビルド（no-op 経路）、
  (2) `wifi_secrets.h.example` からダミー生成 → 有効経路ビルド、
  (3) 退避した元ファイルを復元、
  を常にこの順で実行する（ゲート1第2回指摘: 残留ファイルで片経路が未検証になる問題の排除）
- `pio test -e native` 既存テスト全パス（回帰なし）。Snapshot への累積カウンタ追加に
  対応する host テスト（dt 分類の境界値）を追加
- `python3 -m unittest discover tools/telemetry/tests` 全パス
- E2E: simulator → receiver → analyzer → report をローカルループバックで実行し、
  session dir・summary.json・events.jsonl の生成を確認
- 実機検証（Wi-Fi 実環境・倒立動作）はユーザ実施とし、手順を `telemetry_workflow.md` に
  明記。**タイミング受け入れ基準は dt ヒストグラム bin 比率で定義**する（p95 単一値では
  4 bin 粗ヒストグラムから計算できないというゲート1指摘に対し、8 bin 化した上で基準も
  bin ベースに変更）:
  - telemetry 有効時、`dt > 1.1x` 周期のサイクル比率が無効時比で悪化しない（同条件・
    同時間の比較で有意差なし）
  - `overrun_total` の増加がない（Balancing 中）
  - 20 Hz telemetry の loss < 5%
  - receiver 停止・Wi-Fi AP 停止で制御が継続する
  - AP 停止中の Balancing（非 CONNECTING）で telemetry FSM 起点の `WiFi.begin()`/mode 変更/
    `disconnect()` が発行されない（autoReconnect/persistent 無効化の実機確認）
  - CONNECTING 中の Balancing 遷移ではちょうど 1 回の abort `disconnect()` のみ発行され、
    fresh 非 Balancing snapshot 観測まで Wi-Fi 操作が再開されない
  - GOT_IP 後の AP 喪失 × Balancing 中: ライブラリ one-shot 再接続が発火しても
    cancel-watchdog が有界時間（≦約 1 telemetry 周期 + disconnect 処理）でキャンセルする
  - `fsm == Balancing || arm_pending` 中に telemetry 経路の malloc/socket 作成が発生しない
    （prewarm 済み資源の再利用のみ。計装ビルドで確認）
  - 直立ホールド（arm_pending）中の AP 喪失で one-shot 再接続が発火しても、cancel-watchdog
    が 1 回の abort でホールド中に中断しフラグをクリアする

## 6. Git 運用

- `feat/udp-telemetry-phase1` を `dev/current-mode-xl330-fable5` から作成
- コミットは日本語・1 目的 1 コミット・`Co-Authored-By` 付与
- PR base = `dev/current-mode-xl330-fable5`。base が `develop/*` でないため自動マージ対象外
  （マージはユーザ判断）
- `docs/reviews/`・`prompt_memo.md`（既存 untracked、本件無関係）には触れない

## 7. 非スコープ（明示）

- PC→Core2 コマンドチャネル（runtime ゲイン変更）— 安全設計込みで別 PR
- MCP サーバ — Phase 2
- binary packet 化・DYNAMIXEL present velocity の高頻度読み — 実測で問題が出てから
- packet の HMAC 署名（telemetry 完全性保証）— 信頼境界の文書化で代替し、別 PR
- 制御タスク内の p95 直接計測・dt_max リセット — 累積ヒストグラム差分で PC 側再構成が
  可能になるため優先度低。必要なら別 PR
