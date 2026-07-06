# Wi-Fi ガード計装（trace）ビルド実装計画 — ベンチ検証の機械化

作成日: 2026-07-06
ブランチ: `feat/wifi-guard-trace`（`feat/udp-telemetry-phase1` から作成。PR base = 同ブランチのスタック PR。PR #27 マージ後は base 変更で追随）
上位文書: [`docs/plans/2026-07-05-udp-telemetry-phase1.md`](2026-07-05-udp-telemetry-phase1.md)（§3.1 WIFI_QUIET 不変条件・abort 状態機械・prewarm/udp_ready が設計の正）
対象チェックリスト: [`docs/telemetry_workflow.md`](../telemetry_workflow.md) §6 のうち計装ビルドがないと確認不能な 5 項目

## 1. 目的・スコープ

UDP telemetry Phase 1（PR #27）の実機検証チェックリストのうち、外部から観測不能な
Wi-Fi ガード内部動作に関する以下 5 項目を、ベンチ（スタンド上・検証時のみ USB シリアル
併用可）で**機械的に PASS/FAIL 判定可能**にする。

| ID | チェックリスト項目（telemetry_workflow.md §6） | 判定規則（§5） |
|---|---|---|
| C1 | AP 停止中の Balancing（非 CONNECTING）で telemetry FSM 起点の `begin`/mode 変更/`disconnect` が発行されない | R1 |
| C2 | CONNECTING 中の Balancing 遷移でちょうど 1 回の abort `disconnect()` のみ。fresh 非 Balancing 観測まで Wi-Fi 操作再開なし | R2 |
| C3 | GOT_IP 後の AP 喪失 × Balancing 中、ライブラリ one-shot 再接続を有界時間でキャンセル（burnt 済みの通常 AP 喪失では Wi-Fi 操作なし） | R3 / R3b |
| C4 | `fsm == Balancing \|\| arm_pending` 中に telemetry 経路の malloc/socket 作成が発生しない（判定範囲 = WiFiUDP セットアップ資源。§5 R4 の範囲明確化参照） | R4（全シナリオ横断） |
| C5 | 直立ホールド（`arm_pending`）中の AP 喪失で one-shot が発火しても 1 回の abort でホールド中に中断・フラグクリア | R5 |

**非スコープ**: 制御則・安全機構・Wi-Fi ガードの挙動変更は一切行わない（§3 の
additive アクセサ 1 件を除きプロダクションコードのロジック不変）。dt ヒストグラム比較・
loss 率等の残チェックリスト項目は既存 telemetry 経路で確認可能なため対象外。
ESP-IDF heap hook / heap tracing の導入は行わない（§4 D2 の代替観測で判定）。

## 2. 設計原則

1. **リリースビルド影響ゼロ**: 全 trace コードはコンパイル時フラグ
   `BSL_WIFI_GUARD_TRACE` の `#if` 内。フラグなしビルド（`[env:m5stack-core2]`）の
   バイナリは本 PR 適用前と不変であること（§7 で検証方法を規定）。
2. **wifi_guard.h（core 層）を汚さない**: trace は `telemetry_task.cpp` 側の
   **WifiOps 実装ラッパ**＋ **tick 後の公開アクセサ差分出力**で実装する。
   `core/wifi_guard.h` への変更は additive な const アクセサ **2 件のみ**:
   `bool lastWifiQuiet() const`（既存 `last_wifi_quiet_` の読み出し）と
   `uint32_t drainedEpoch() const`（既存 `epoch_` = drain 済みイベント累積数の読み出し。
   §5 の drain 帰属に使用）。いずれもロジック・タイミング変更なし。
   SafetyFsm::armPending() と同じ「additive アクセサ」パターン。`#if` ゲートは付けず
   native テストで検証する。
3. **Serial 出力は telemetry task 文脈のみ**: ControlTask / UiTask / main.cpp には
   一切触れない。Wi-Fi イベントコールバック（イベントタスク文脈）でも Serial を
   呼ばない（§4 D10: trace 専用 SPSC リングに積み、telemetry task が印字）。
4. **1 行 1 レコードの機械可読形式**（§4 D3）＋ **PC 側チェッカスクリプト**（§5）で
   合否判定を自動化する。目視判定を残さない。
5. **1 シナリオ = 1 キャプチャログ**: 判定規則の適用単位を明確にする。

## 3. 変更ファイル一覧

| ファイル | 変更 | リリース影響 |
|---|---|---|
| `src/tasks/telemetry_task.cpp` | trace 実装一式（`#if BSL_WIFI_GUARD_TRACE` 内、かつ既存の `BSL_TELEMETRY_SECRETS_AVAILABLE` 領域の内側に限定。整形/シーケンシングは trace_emitter へ委譲） | なし（フラグなしで不変） |
| `src/core/trace_emitter.h` | 新規: trace 行整形 + tick 内シーケンシング（Arduino 非依存、D11） | なし（リリース TU から include されない） |
| `scripts/gen_trace_fixtures.sh` | 新規: native ハーネスから emitter 生成ログ fixture を再生成 | なし |
| `tools/telemetry/tests/fixtures/` | 新規: emitter 生成ログ fixture（D11 の相互検証入力） | なし |
| `src/core/wifi_guard.h` | `lastWifiQuiet()` / `drainedEpoch()` const アクセサ追加（各 1 行 + コメント） | コード追加のみ・呼び出しなし（インラインで未使用なら排出されない） |
| `platformio.ini` | `[env:m5stack-core2-trace]` 追加（base env を `extends`、`-DBSL_WIFI_GUARD_TRACE=1`・`monitor_speed=921600`・ホールド延長 `-D`（§6 T5）追加） | なし（既存 env 不変） |
| `src/app_config.h` | ホールド時間定数の `#ifndef` ガード化（§6 T5。リリース値・型・意味は不変） | なし（マクロ未定義時は現行値） |
| `scripts/verify_builds.sh` | 手順 (2b): trace env × ダミー secrets ビルドを追加 | なし |
| `tools/telemetry/bsl_telemetry/trace_check.py` | 新規: trace ログの機械判定チェッカ（stdlib のみ） | なし |
| `tools/telemetry/tests/test_trace_check.py` | 新規: 合成ログによる R1〜R5 の PASS/FAIL 両系テスト | なし |
| `docs/telemetry_workflow.md` | §6 の C1〜C5 に「計装ビルド手順（新 §）参照」を付記 + C4 文言の精密化（§5 R4 範囲一致、改訂注記付き） + 新 § 追加（ビルド・キャプチャ・シナリオ手順・チェッカ実行） | なし |
| `test/test_core/test_main.cpp` | `lastWifiQuiet()`/`drainedEpoch()` の反映検証 + D11 emitter テスト追加 | なし |

## 4. 設計判断（D1〜D11）

- **D1 — trace の配置**: `telemetry_task.cpp` の匿名 namespace に trace 用の静的状態
  （直近 snapshot 文脈 `g_trace_fsm`/`g_trace_arm`、前回ガード状態タプル、trace 用
  イベントリング、tick カウンタ）を置く。すべて telemetry task 単一スレッドで読み書き
  （イベントリングのみ SPSC）。既存の `BSL_TELEMETRY_SECRETS_AVAILABLE` 有効領域の
  内側に閉じ込めることで、**trace ∧ ¬secrets ≡ ¬secrets（no-op）を構造的に保証**する。
- **D2 — malloc/socket 観測は beginPacket プロキシ**: ESP-IDF heap hook は
  大掛かりなため導入しない。pin 済み framework（arduinoespressif32
  3.20017.241212+sha.dcc1105b）の `WiFiUdp.cpp` 実装では、`beginPacket()` が
  「`tx_buffer`（1460 B）未確保時のみ malloc」「socket 未作成時のみ `socket()` 作成」を
  行い、`write()`/`endPacket()` は確保済みバッファへの書き込み/送出のみで追加確保しない
  （`stop()` で解放。本 firmware は `stop()` を呼ばない）。よって
  **「malloc/socket 作成が起き得るのは prewarm-class の beginPacket（`udp_ready==false`
  での呼び出し）と、ログ全体で最初の beginPacket のみ」**であり、R4 はこれを判定する。
  **2026-07-06 に framework-arduinoespressif32 3.20017.241212+sha.dcc1105b の実ソースで
  確認済み**（`WiFiUdp.cpp`: `beginPacket()` は `!tx_buffer` 時のみ `malloc(1460)`、
  `udp_server == -1` 時のみ `socket()`、解放は `stop()` のみ。`updateBaudRate` は
  `HardwareSerial.h:232` に存在）。実装時にも同バージョンで再確認し、確認済み
  バージョン番号をコードコメントへ記録する（wifi_guard.h 冒頭の既存記録と同じ運用）。
  補助証拠として hb 行に `esp_get_free_heap_size()`/`esp_get_minimum_free_heap_size()`
  を含める（他タスクの確保と区別できないため**参考情報であり合否条件にしない**）。
- **D3 — 出力形式（grammar v1、タグ `[WG1]`）**: 全行
  `[WG1] s=<n> t=<esp_timer_us>` で開始（**`s` = 全レコード共通の単調増加通し番号**。
  ゲート1第15回指摘1対応 — デバイス側の drop 検査だけではホスト側 UART/monitor/tee
  での行喪失・破損を検出できず、「禁止 op 行がちょうど 1 行消えたログ」が不在証拠で
  偽 PASS する。trace_check は boot から最終 hb まで `s` の**連続性**を要求し、欠番・
  重複・`[WG1]` で始まるが文法不一致の行（破損）を INVALID とする）、以降
  `key=value` の空白区切り。7 種（boot / ev / op / tk / st / hb / drop）:
  - `boot` 行（**telemetry task 起動直後、一切の guard tick / Wi-Fi 操作より前**に
    1 回だけ出力。ゲート1第14回指摘1対応 — キャプチャがブートから始まっている
    ことの機械的証明）:
    `[WG1] s=<n> t=<us> boot v=1 fw=<git hash> dev=<id>`
    trace_check は **boot 行より前に他の `[WG1]` 行が存在する、または boot 行が
    存在しないログを INVALID** とする（先頭欠落ログで C4 が部分主張化することを
    防ぐ。§6 の「モニタ接続 → リセット」手順とセット）。
  - `ev` 行（Wi-Fi イベント。イベントコールバック時刻で記録）:
    `[WG1] s=<n> t=<us> ev=<gotip|disc|stop|start|conn|scan> r=<reason|-> i=<n>`
    （`start`/`conn`/`scan` = ARDUINO_EVENT_WIFI_STA_START / STA_CONNECTED /
    SCAN_DONE。**trace 専用の追加購読**であり guard への `pushEvent` 対象は従来の
    3 種から変更しない。ライブラリ内部の再接続活動（telemetry の Ops を経由しない
    `disconnect();begin();`）を op 行の不在に頼らずイベント面で可視化するため —
    ゲート1第3回指摘1対応）。
    `i` = **guard 対象 3 種（gotip/disc/stop）にのみ振る到着通し番号**（1 起点。
    コールバックは guard `pushEvent` → trace ring の順に単一タスクで push するため、
    この番号は guard キュー内の順序と一致する。trace 専用購読の start/conn/scan は
    `i=-`。§5 の drain 帰属に使用 — ゲート1第7回指摘対応）
  - `op` 行（Ops ラッパ。`t`/`dur` は実 API 呼び出しの開始時刻/所要時間 µs — C2/C3 の
    「disconnect 処理時間」実測に使う。`res` は戻り値）:
    `[WG1] s=<n> t=<us> op=<begin|disc|radio_off|bp> fsm=<0-5|-> arm=<0|1|-> q=<0|1> udpr=<0|1> res=<n|-> dur=<us>`
    さらに **`op=bp` の行のみ末尾に `fsm2=<0-5|-> arm2=<0|1|-> fsm3=<0-5|-> arm3=<0|1|->`
    を必須で付加**する（`fsm2`/`arm2` = bp **直前**、`fsm3`/`arm3` = bp **戻り直後**の
    SharedState 即時再サンプル。ゲート1第14回指摘2対応 — malloc/socket 確保は
    beginPacket 実行中（`dur` 区間内）に起きるため、pre のみでは確保区間を挟めない。
    pre/post の**両方**が非窓であれば「確保完了時点でも実窓外」= 確保が窓開始より
    先行したことが証明される。§5 R4 の判定入力。ゲート1第12回指摘3対応で grammar に
    正式化 — trace_check は op=bp に全 4 フィールドの存在を要求し、emitter fixture
    でも存在・解釈を assert する）。
    （`fsm`/`arm` はこの tick で guard に渡した snapshot 文脈（read 失敗時は `-`）、
    `q` は op 時点の `lastWifiQuiet()`（ゲート1第1回指摘4対応: stale snapshot による
    QUIET も判定に含めるため）。`bp` = beginPacket。`write`/`endPacket` は D2 のとおり
    確保を伴わないため op 行は出さず、失敗は st 行の `sf`/`pwf` カウンタで観測する）
  - `tk` 行（**毎 tick、guard 処理前の tick 冒頭**で記録。ゲート1第5回指摘1対応 —
    tick 内で発行される op は post-tick の st より時系列上先に来るため、遷移 tick の
    アンカーを guard 処理より前の時刻で与える）:
    `[WG1] s=<n> t=<us> tk fsm=<0-5|-> arm=<0|1|-> ep=<n>`
    （この tick で guard に渡す snapshot 文脈。`ep` = **この tick の drain 実行前**の
    `WifiGuard::drainedEpoch()`（= guard がこれまでに drain 消費したイベント累積数）。
    20 Hz × 約 50 B ≈ 1 KB/s で 921600 baud の 1% 強。§5 の全 tick 単位デッドラインは
    tk の t を基準に、イベントの drain 帰属は ep を基準に測る）
  - `st` 行（tick + 送信試行の完了後、状態タプルが前回から変化した時のみ）:
    `[WG1] s=<n> t=<us> st fsm=<n|-> arm=<0|1|-> q=<0|1> conn=<0|1> cing=<0|1> udpr=<0|1> librp=<0|1> ab=<0|1> roff=<0|1> latch=<0|1> pwf=<n> sf=<n> evo=<n>`
    （`q` = `lastWifiQuiet()`、`evo` = `eventOverflowTotal()`）。
    **st は tick 完了後の状態スナップショットであり、tick 内で完結した過渡
    （例: 同一 drain 内で librp が 1→0、abort の開始と完了）は st には現れない**
    （ゲート1第1回指摘2対応）。過渡を含む意味論の判定は ev/op 記録に基づいて行い
    （§5 R3/R5）、st/hb は文脈追跡と最終状態の健全性確認に使う。
  - `hb` 行（20 tick = 1 s ごと。st と同フィールド + `heap= heapmin= evdrop= flmax= stkmin= tick=`。
    `evdrop` は trace リング/行バッファの破棄数合計、`flmax` は前回 hb 以降の最大
    フラッシュ所要時間 µs、`stkmin` は telemetry task の最小スタック残量バイト
    （`uxTaskGetStackHighWaterMark` 換算）— いずれも判定有効性の検査に使う）
  **印字タイミング（ゲート1第1回指摘1対応 — probe effect の排除）**: ev/op/st/hb の
  全レコードは生成時に**印字せず** tick ローカルの**バイト単位バッファ（4096 B +
  drop マーカー予約 64 B。ゲート1第13回指摘2対応 — 行数ベースの 8 行では trace
  イベントリング満杯 16 件の正当バーストを収容できず、密な Wi-Fi 活動時に自己
  INVALID 化する）**へ蓄積し、**guard tick と trySend が完了した後に 1 箇所でまとめて
  Serial へフラッシュ**する。宣言最悪バースト = 1 tk(≈50 B) + 16 ev(≈60 B each) +
  4 op(≈120 B each) + st(≈130 B) + hb(≈170 B) + drop(≈40 B) ≈ 2.1 KB < 4096 B、
  921600 baud で ≈ 22 ms < flmax 予算 40 ms。**レコード種別ごとの最大行長は emitter の
  native テストで assert し、この予算計算をコードに固定**する。
  **スタック予算（ゲート1第15回指摘3対応）**: 行バッファ（4096+64 B）は telemetry
  task のスタックに置かず **`#if BSL_WIFI_GUARD_TRACE` 内の static 領域**に確保する
  （既存 8192 B スタック + 1024 B packet バッファ等との競合を構造的に回避）。加えて
  hb 行に `stkmin=<uxTaskGetStackHighWaterMark() バイト換算>` を追加し、チェッカは
  `stkmin < 1024` のログを INVALID とする（検証ビルド自身のスタック余裕を常時実測）。Ops ラッパ内・guard 処理前に Serial ブロッキングを一切挿入しない
  （abort 発行タイミング等の被測定経路を遅延させない）。行バッファ溢れは `evdrop` に
  計上して破棄する（黙って欠落させない）。
  **欠落の非遅延報告（ゲート1第3回指摘2対応）**: 行バッファには **drop マーカー専用の
  予約スロットを 1 つ**確保し、リング/行バッファの破棄が発生した tick のフラッシュでは
  必ず `[WG1] s=<n> t=<us> drop n=<累積evdrop>` 行を出力する（予約スロットにより drop
  マーカー自身は絶対に欠落しない）。1 Hz の hb を待たずに、欠落の発生がログ上で
  その tick 中に自己申告される。
  **フラッシュ時間の予算保証（ゲート1第2回指摘1対応 — 次 tick への持ち越し遅延の
  排除）**: 最悪バースト（8 行 × 160 B = 1280 B）が 115200 baud では ≈ 111 ms と
  telemetry 周期 50 ms を超過し、次 tick の guard 処理を遅らせ得る。対策として
  trace ビルドでは **Serial を 921600 baud で運用**する（telemetry task 起動時に
  `Serial.updateBaudRate(921600)`（trace フラグ内、main.cpp は変更しない）、trace env の
  `monitor_speed = 921600`。CP2104 は 2 Mbaud まで対応。切替前のブート出力は文字化け
  するが `[WG1]` 行はすべて切替後であり、チェッカは非 `[WG1]` 行を無視する）。
  921600 baud での最悪バーストは ≈ 14 ms で周期内に数学的に収まる。加えて毎 tick の
  フラッシュ所要時間を計測し、hb 行の `flmax=<us>`（前回 hb 以降の最大値）で公開する。
  チェッカは `flmax > 40000`（周期 50 ms − マージン 10 ms）のログを INVALID として
  再取得を要求する（予算超過の黙認をしない）。
- **D4 — Ops ラッパ**: `#if BSL_WIFI_GUARD_TRACE` 時のみ、`WifiOps` に bind する関数を
  trace 版（`TraceOpsBegin` 等 = **行バッファへの記録** + 実 Ops 委譲）に差し替える。
  `setRadioOff` は op=radio_off として記録。実 API の呼び出し順序・回数・引数は完全に
  不変。ラッパ内では Serial を呼ばない（D3 印字タイミング参照）。
- **D5 — ビルド env**: `[env:m5stack-core2-trace]` は `extends = env:m5stack-core2` とし
  `build_flags = ${env:m5stack-core2.build_flags}` + `-DBSL_WIFI_GUARD_TRACE=1`、
  `monitor_speed = 921600`（D3 フラッシュ予算保証）。
  base の build_flags は `!python3` 動的フラグのため、展開が per-line で評価されることを
  実装時に `pio run -v -e m5stack-core2-trace` のコンパイルコマンドで確認する
  （`-DBSL_FW_GIT` と `-DBSL_WIFI_GUARD_TRACE` の両方が入ること）。
- **D6 — verify_builds.sh**: 既存 (1)(2) の後に「(2b) trace env × ダミー secrets
  ビルド」を追加（計 3 ビルド）。trace × ¬secrets は D1 の構造的保証（trace コードが
  secrets 有効領域の内側にしか存在しない）によりビルド対象へ加えない（コメントで根拠を
  明記。レビューでこの構造を検査する）。
- **D7 — バイナリ不変性の検証**（一回性の検証。恒常ゲートにはしない）: 手順は §7。
- **D8 — 手順書**: `docs/telemetry_workflow.md` へ新節「計装（trace）ビルドによる
  ガード検証」を追加し、C1〜C5 ↔ シナリオ T1〜T5 ↔ 規則 R1〜R5 の対応表・ベンチ手順・
  チェッカ実行例を記載。§6 チェックリストの該当 5 項目に新節参照を付記する。
- **D9 — Serial 予算**: `Serial.begin(115200)` は main.cpp 既存（変更しない。trace
  ビルドのみ telemetry task 起動時に `updateBaudRate(921600)`、D3）。全印字は
  telemetry task（core 1, prio 1, 周期 50 ms）文脈で、**tick 末尾の一括フラッシュのみ**
  （D3）。1 行 ≈ 40–170 B、921600 baud で ≈ 0.4–1.8 ms/行、宣言最悪バースト
  （≈ 2.1 KB、D3 参照）でも ≈ 22 ms < flmax 予算 40 ms < 50 ms 周期（フラッシュ
  所要は `flmax` で常時実測・チェッカが予算超過を INVALID 化）。フラッシュは guard 処理・送信完了後なので被測定経路（abort 発行等）を
  遅延させない。ControlTask（core 0）への影響なし。
- **D11 — trace emitter の純ロジック化とエミッタ生成ログの相互検証（ゲート1第8回
  指摘1対応 — ep の読み出しタイミングや行順序は firmware 側実装の要であり、手書き
  合成ログだけのテストでは「tk を tick 後に出す」「post-drain の epoch を読む」等の
  実装ミスを素通しして第7回指摘の偽 PASS 窓が再発する）**: 行整形と tick 内
  シーケンシング（tk（ep = guard tick **前**の `drainedEpoch()`）→ trace リング
  drain（ev 行化）→ guard tick / trySend（op 記録）→ st 差分 → hb/drop → フラッシュ）
  を **`src/core/trace_emitter.h`（新規、Arduino 非依存）**へ切り出す。
  telemetry_task.cpp は trace フラグ時に現在時刻・snapshot 文脈・`WifiGuard` 参照を
  emitter へ渡し、得られた行バッファを Serial へ流すだけにする（リリースビルドは
  本ヘッダを include しない）。検証:
  - native テスト: `WifiGuard` + emitter + fake ops を第7回指摘の**両インターリーブ**
    （tk 後・drain 前のイベント到着 / drain 後・trySend 前の到着）で駆動し、
    (a) tk の `ep` が常に pre-drain 値であること、(b) 行順序・全フィールドが
    grammar v1 に一致することを assert する。
  - emitter の native テストは**全生成行が `[WG1] s=` で始まり `s` が連続である**
    ことも assert する（ゲート1第16回指摘1対応 — s なしの旧例に従う実装を遮断）。
  - 同ハーネスから代表シナリオのログを生成して `tools/telemetry/tests/fixtures/` に
    保存し（`scripts/gen_trace_fixtures.sh` で再生成可能、§7 で「再生成して diff
    ゼロ」を検証）、Python 側 unittest は**手書き合成ログに加えて emitter 生成ログ**を
    trace_check に通し、PASS/FAIL/INVALID が仕様どおりであることを相互検証する。
- **D10 — イベント trace リング**: `onWifiEvent()` 内では guard への `pushEvent` に加え、
  trace 専用 SPSC リング（容量 16、`{t_us, kind, reason}`、atomic head/tail —
  wifi_guard の既存キューと同型）へ push のみ行う。telemetry task は毎 tick 冒頭で
  リングを drain して**行バッファへ変換のみ**行い（Serial には触れない）、印字は
  D3 の一括フラッシュに委ねる。**イベントタスク文脈での Serial 呼び出しゼロ、
  guard 処理前の Serial ブロッキングもゼロ**。リング満杯時は `evdrop` カウンタを
  増やして破棄（チェッカは evdrop>0 のログを INVALID 扱いにして再取得を要求）。
  ev 行の t はコールバック時点の時刻なので、印字順と時系列が食い違ってもチェッカは
  t でソートして判定する。

## 5. チェッカ判定規則（trace_check.py）

入力: 1 キャプチャログ（`[WG1]` 行以外は無視）。**前処理（ゲート1第15回指摘2対応 —
t ソートはリセット前の残存レコードを新 boot の後ろへ隠すため、boot 構造は物理
ファイル順で先に検証する）**: (0) 物理行順で、最初の `[WG1]` 行がちょうど 1 本の
boot 行であること・boot 行が複数ないこと（多重リセット/多重エポックのログは
INVALID）・boot より前に `[WG1]` 行がないことを検証し、その単一エポック内でのみ
以降の処理を行う。(1) `s` の連続性・文法適合を検証（欠番/重複/破損 = INVALID）。
(2) その後に全レコードを `t` で安定ソートし、`st`/`hb` 行から文脈
（fsm/arm/q/ガード状態）を追跡する。CLI:
`python3 tools/telemetry/bsl_telemetry/trace_check.py --scenario t1|t2|t3|t3b|t5 <log>`
（R4 と妥当性検査は全シナリオで常時実行）。出力: 規則ごとに PASS/FAIL/INVALID と
根拠行の引用。終了コード 0 = 全 PASS。

- **共通妥当性（全シナリオ）**: `drop` 行が 1 件もない かつ 全 hb で `evdrop == 0` かつ
  `evo == 0`、かつ全 hb の `flmax ≤ 40000` µs（フラッシュ予算内、D3。超過は
  probe effect の疑いがあるため INVALID）、かつ**全 hb に `stkmin` が存在し
  `stkmin ≥ 1024`**（trace ビルド自身のスタック余裕の実測保証、D3 スタック予算。
  フィールド欠落・閾値未満はともに INVALID。ゲート1第16回指摘2対応 — 合成テストに
  「stkmin 欠落 → INVALID」「stkmin=512 → INVALID」「stkmin=2048 → PASS」を含める）。
  **latch/radio-off の分類（ゲート1第10回指摘2対応 — 最高リスクの観測結果を
  「取り直し」に格下げしない）**: R2/R3/R5 の**有効なヒットエピソード内**で
  `radio_off`/`latch=1` に到達した場合は **FAIL（理由: 有界キャンセル未完了・終端
  radio-off 到達 = §3.1 の wifi_abort_failed 経路）**とし、ゲートをブロックする。
  ヒットエピソード外（ログ冒頭から latch=1 が立っている等、判定対象の挙動と無関係に
  証拠能力が疑われる場合）に限り INVALID とする。INVALID は「証拠を信頼できない
  ログ（欠落・不完全・probe 超過）」専用の分類とする。
  **ログ末尾・カバレッジの完全性（ゲート1第3回指摘2対応、第14回指摘3で強化 —
  ev/op が 1 件もない T1 成功系で hb 検査が空成立することを防ぐ）**: 全シナリオ
  共通で (a) boot 行がログ先頭にある（D3）、(b) **hb 行が 2 本以上存在**する、
  (c) **時系列最後の `[WG1]` 行が hb**である（= 全区間の evdrop/flmax 報告で
  セッションが閉じている）、(d) 時系列最後の ev/op 行の後に少なくとも 1 本の hb が
  ある、をすべて満たさないログは INVALID とする。§6 の共通手順で「最後の操作から
  2 秒以上待ってからキャプチャを停止する」ことを必須化し、これを機械側で強制する。
  ヒット窓が 1 件も見つからない場合はそのシナリオを FAIL（「シナリオ不成立」）とする。
- **disconnect 所要時間の予算検査（ゲート1第4回指摘2対応 — dur を記録するだけでは
  遅い/ストールした disconnect が PASS の陰に隠れる）**: R2/R3/R5 の abort `op=disc`
  は、開始時刻の条件（≤ 2 tick）に加えて
  (a) `dur ≤ 100 ms`（`kDiscDurBudgetUs = 100000`、チェッカ定数・TUNE。上位設計の
  「≦約 1 telemetry 周期 + disconnect 処理時間」の "処理時間" に明示上限を与える）、
  (b) 完了期限 `t + dur ≤ 遷移時刻 + 2 tick + 100 ms`
  の両方を満たすこと。違反は FAIL（実測 dur を根拠として提示）。合成テストに
  「遅い disconnect（dur 超過）」の FAIL 系を R2/R3/R5 それぞれに用意する。
- **abort エピソードの追跡範囲（ゲート1第2回指摘3対応、第12回指摘1で完了証明を
  厳格化）**: R2/R3/R5 の禁止窓は `q` が 0 に戻った時点で打ち切らず、**abort
  エピソードの完了まで**追跡する。**完了の証明は post-abort 確認イベント
  （`ev=disc r=8` または `ev=stop`。drain 帰属が abort op より後であること）のみ**を
  認める — st/hb の `ab=0` は完了イベント観測後の最終状態サニティチェックであり、
  完了の代替証明にはしない（イベントなしで ab が 0 に戻る系列は「確認なしでガードが
  解除された」ことを意味し FAIL）。完了イベントがリトライ規則の許容範囲内に
  観測されないエピソードは FAIL（有界キャンセル未完了）。この要求は R2/R3/R5 の
  全ヒットエピソードに共通で適用する。エピソード内で `q=0`（WIFI_QUIET 非継続）の文脈で発行された追加の
  `op=disc`（リトライ）または `op=radio_off` は **FAIL** とする（上位設計 §3.1 は
  リトライを「WIFI_QUIET 継続中に限り」許可しており、quiet 解除後のリトライ/終端
  radio-off は設計逸脱として顕在化させる。§9 の注記参照）。合成テストに
  「abort 確認前に quiet が解除され、その後リトライが発行される」FAIL 系を含める。
- **有界リトライの扱い（ゲート1第6回指摘2対応 — §3.1 は確認未達・disconnect 失敗時の
  有界リトライを正当な fail-closed 経路として許可しており、「disc ちょうど 1 件」を
  無条件に強制すると準拠実装を棄却してしまう）**: R2/R3/R5 の禁止窓内の追加の
  `op=disc` は、以下を**すべて**満たす場合に限り PASS とし、リトライ回数を出力に
  注記する: (i) 各リトライ時点の文脈が `q=1`（WIFI_QUIET 継続中）、(ii) disc 総数 ≤
  1 + `abort_max_retries`（= 4 件）、(iii) 窓内に start-class op（begin / prewarm-class
  bp）がない。違反（q=0 でのリトライ・総数超過）は FAIL。リトライ上限到達後の
  `radio_off` + latch は **FAIL（理由: 有界キャンセル未完了・終端 radio-off 到達。
  共通妥当性の分類規則参照 — ゲート1第10回指摘2対応）**とし、「ガードの設計どおりの
  終端経路に到達した = ベンチ環境の Wi-Fi 異常または disconnect 経路の不全」である
  ことを根拠行付きで報告してエスカレーションを促す。合成テスト: 確認遅延 + q=1
  リトライ 1 回 → PASS（注記付き）、q=0 後のリトライ → FAIL、上限到達 → radio_off →
  FAIL（区別理由付き）。
- **ev ストリームの静粛検査（ゲート1第11回指摘対応 — ev=start/conn/scan/gotip は
  Ops ラッパを経由しないライブラリ起源の Wi-Fi 活動を可視化するために購読しており、
  R3b だけでなく全 quiet 窓規則がこれを消費しなければ「op なし・ライブラリ活動あり」
  のログが PASS してしまう）**:
  - R1 の窓内では ev∈{start, conn, scan, gotip} および `disc`（全 reason）も 0 件で
    あること（op 行の不在だけでは §3.1 の禁止負荷（scan/認証バースト）を証明しない）。
    窓進入は手順上 settled（`cing=0`・backoff 消化）で行うため、進入直前操作の遅延
    イベントが窓内に落ちた場合は根拠行付きで FAIL となり再取得で対処する。
  - R2/R3/R5 では、エピソード開始〜abort 完了確認の間は in-flight 接続の残骸イベント
    （scan / disc（全 reason）/ conn / gotip — conn/gotip は established-connection
    ラッチによる追加 abort 規則の対象）を許容し、**abort 完了確認後〜q=0 復帰までは
    ev∈{start, conn, scan, gotip} と disc（全 reason）が 0 件**であることを要求する
    （キャンセルの有界性をイベント面でも検査）。違反 = ライブラリ起源活動の漏れとして
    FAIL。
  - 合成テスト: 「op なしだがライブラリ起源 scan/conn/gotip が quiet 窓内に出る →
    R1 FAIL」「abort 完了後の追加 disc/scan → R2/R3/R5 FAIL」を追加。
- **R1（T1, C1）**: 各 Balancing 窓（文脈 fsm が 2 になってから外れるまで）のうち、
  窓開始時点で `cing=0 ∧ librp=0 ∧ ab=0` のもの: 窓内に `op∈{begin, disc, radio_off}`
  が 0 件、かつ窓内の `bp` はすべて `udpr=1`、かつ ev ストリーム静粛（上記）。
- **tick アンカーと証拠順序（ゲート1第5回指摘1対応）**: 「q の 0→1 遷移」等の
  エピソード開始は、**その遷移の証拠（q=1 の st 行、または q=1 の op 行）を含む tick の
  `tk` 行の t** をアンカーとする。同一 tick 内では op（guard 処理中）が st（処理後）
  より先に記録されるのが正常であり、チェッカは st の出現順ではなく tk アンカーからの
  経過時間で全デッドライン（≤ N tick、dur 完了期限）を測る。abort が遷移 tick 内で
  即時発行されるケース（op=disc の t が最初の q=1 st より前）は正しい挙動として PASS
  させる合成テストを用意する。
  **イベント起点窓の drain アンカー（ゲート1第6回指摘1対応、第7回指摘で厳密化 —
  tk タイムスタンプからの推定は「tk 記録後〜guard 内部 drain 前」に到着したイベントを
  誤って次 tick 扱いにする観測レースを持つため、推定を排除し guard 自身のカウンタで
  帰属を決める）**: R3/R5 の禁止窓（`ev=disc r≠8` 起点）は、**epoch 突合による厳密な
  drain 帰属**でアンカーする:
  - ev 行の `i`（guard 対象イベントの到着通し番号）と tk 行の `ep`（drain **前**の
    `drainedEpoch()` = 直前 tick までの消費累計）を突合し、**イベント i の drain
    tick T は `tk(T).ep < i ≤ tk(T+1).ep` を満たす一意の tick**（= pre-drain の
    `ep ≥ i` を初めて満たす tk の**直前の** tick。ゲート1第9回指摘の off-by-one を
    排除した正式定義）と機械的に決定する。ログ末尾で tk(T+1) が存在しない場合、
    その区間のイベントは drain 帰属不能として当該エピソードを INVALID にする
    （末尾 hb 要件と併せて運用上は発生しない）。tk や ev の時刻からの推定は
    行わない。
  - 禁止窓は drain tick T の tk アンカーから開始し、abort デッドライン（≤ 2 tick）も
    そこから測る。T より前（pre-drain）の op は禁止窓に含めない。
  - キュー対応の前提（コールバックが guard → trace ring の順に push、単一
    producer）により i と guard キュー順序は一致する。いずれかのキューで破棄が
    起きたログは既存の共通妥当性（evo/evdrop）で INVALID になるため、対応の破れは
    黙認されない。
  - 合成テスト（両インターリーブ必須）: (1)「tk 記録後・guard drain 前にイベント
    到着（tk(T).ep < i ≤ tk(T+1).ep、ev.t > tk(T).t）→ 同 tick T の trySend の bp は
    禁止窓内 → FAIL」、(2)「guard drain 後・trySend 前にイベント到着
    （tk(T+1).ep < i、すなわち T+1 帰属、ev.t > tk(T).t）→ tick T の bp は
    pre-drain → PASS」。**(1) は手書き合成ログに加えて D11 の emitter 生成 fixture
    でも再現し、Python trace_check が FAIL を報告することを assert する**
    （ゲート1第9回指摘対応 — 帰属規則と emitter 実出力の境界一致を fixture で固定）。
- **R2（T2, C2）**: `cing=1` の状態で `q` が 0→1 に遷移するエピソード: 遷移 tick の
  tk アンカーから次の「`q=0 ∧ fsm≠2 ∧ arm=0`」文脈（またはログ末尾）までの
  **禁止窓**において、
  op 行の**全集合が abort `op=disc` のみ**であること（`begin`・`bp`・`radio_off` を
  含む他の一切の op 行が 0 件。ゲート1第1回指摘3対応 — 「Wi-Fi 操作が再開されない」の
  完全検査）。健全系の期待は disc 1 件のみで、2 件目以降は「有界リトライの扱い」
  （下記）に従う場合のみ許容。初回 disc は遷移から ≤ 2 tick（≈120 ms）以内。窓内に
  `radio_off` または `latch=1` が現れたら「有界リトライの扱い」の区別報告に従う。
- **R3（T3, C3 前半）**: `conn=1 ∧ fsm=2` の文脈で `ev=disc r≠8`（非自発的切断 =
  one-shot 発火確定）が発生するエピソード: 判定は **ev/op 記録ベース**で行う
  （librp の 1→0 は同一 tick 内で完結し st に現れない場合があるため、過渡状態の
  観測は要求しない。ゲート1第1回指摘2対応）:
  (a) drain アンカーから ≤ 2 tick 以内に `op=disc` が発行され、エピソードの禁止窓
  （Balancing 継続中）の op 全集合が abort `disc` のみ（R2 と同一の完全検査。健全系の
  期待は 1 件のみで、リトライは「有界リトライの扱い」に従う場合のみ許容）、
  (b) post-abort の `ev=disc r=8` または `ev=stop`（完了確認イベント）が存在、
  (c) エピソード終了後の最初の st/hb で `ab=0 ∧ librp=0 ∧ roff=0 ∧ latch=0`
  （最終状態の健全性のみを st で確認。中間の `librp=1`/`ab=1` の観測は不要）。
- **R3b（T3b, C3 後半）**: 空成立を認めない**ヒット条件を明示**する（ゲート1第2回
  指摘2対応 — 「過去に one-shot 完結 + Balancing 窓に op なし」だけでは burnt 経路を
  実際に試験したことにならない）。ヒット = 以下の**全て**を時系列順に観測:
  (i) 最初の one-shot エピソードの完結（R3 のエピソード終了条件）、
  (ii) その後の **再接続の成立**（`q=0` 文脈での `ev=gotip`、および st/hb で
  `conn=1 ∧ udpr=1`）、
  (iii) 再接続後、`fsm=2 ∨ arm=1` の文脈での**新たな** `ev=disc r≠8`（burnt 状態での
  AP 喪失）。
  判定（ゲート1第3回指摘1対応 — op 行の不在だけでは telemetry Ops を経由しない
  ライブラリ内部の `disconnect();begin();` を観測できないため、**イベント面の静粛も
  要求**する）: (iii) 以降、当該 Balancing/arm 窓内で
  (a) op 行が 0 件、**かつ** (b) Wi-Fi 活動を示す ev 行が 0 件 — 具体的には
  `ev∈{start, conn, scan, gotip}`、および (iii) の hit disc 以外の**追加の** `ev=disc`
  （内部 disconnect の r=8、内部 begin 失敗の r≠8 いずれも）。
  (b) に該当するイベントが観測された場合は FAIL とし、「one-shot 再アームの疑い
  （前提モデルの反証）」として §9 のエスカレーション対象であることを出力に明記する。
  **観測窓の下限（ゲート1第4回指摘1対応 — in-flight な内部接続試行の失敗イベントは
  数秒〜十数秒遅れて届き得るため、短いテールでは「活動なし」を証明できない）**:
  (iii) の hit から **20 秒以上**（lib reconnect timeout 15 s + マージン）、
  `fsm=2 ∨ arm=1` の文脈を維持したまま記録が続いていること（t 差分で機械検査）。
  これに満たないログは INVALID（「観測窓不足」）。合成テストに「hit の 18 秒後に
  活動イベントが届く遅延失敗系列 → FAIL」を含める。
  (i)〜(iii) が揃わないログは FAIL（「シナリオ不成立」）。
- **R4（全ログ, C4）**: (a) `udpr=0` の `bp`（prewarm-class = malloc/socket 作成が
  起き得る呼び出し）はすべて **`q=0`**（∴ fresh ∧ fsm≠2 ∧ arm=0）で発行されている。
  (b) ログ中で時系列最初の `bp` も **`q=0`** で発行されている。
  (c) `fsm=2 ∨ arm=1` の `bp` はすべて `udpr=1`（(a) の対偶側の明示検査）。
  （ゲート1第1回指摘4対応: stale/read-fail snapshot による QUIET（fsm/arm が
  非 Balancing に見えるケース）も op 行の `q` フィールドで判定に含める）
  **R4 が証明する範囲の明確化（ゲート1第5回指摘3対応 — 過大主張の排除）**: R4 は
  「**WiFiUDP レベルのセットアップ資源（tx_buffer 1460 B の malloc と UDP socket の
  作成）が `!WIFI_QUIET` 下に限定される**」ことを証明する。これは上位計画 §3.1 が
  「重い操作」と定義した対象（socket 作成・初回 beginPacket の遅延確保）そのもの
  であり、C4 のチェックリスト項目はこの意味で判定する。一方、許可された Balancing 中
  送信（`udpr=1` の write/endPacket）が内部で行う lwIP/ドライバ層の一時確保
  （pbuf 等）は設計上 Balancing 中にも発生する正常動作であり、本計装の検証対象外
  （§9 残余リスクに明記。hb の heap/heapmin は参考観測として残す）。
  **上位文書との文言一致（ゲート1第8回指摘2対応）**: `docs/telemetry_workflow.md` §6 の
  C4 項目の文言を本 PR で「telemetry 経路の **WiFiUDP セットアップ資源（UDP socket・
  1460 B tx_buffer）の作成**が発生しない（prewarm 済み資源の再利用のみ）」へ精密化
  する（§3.1 の「重い操作」定義が正であり、チェックリスト行をそれに整合させる改訂。
  受け入れ基準の黙った縮小ではなく、判定範囲と文言の一致を取る変更であることを
  改訂注記に明記する）。
  **sampled 契約の明示と実時間再サンプル観測（ゲート1第10回指摘1対応 — snapshot
  読出しから bp までの間に ControlTask が arm/Balancing を publish するサブ tick
  レースがあり、op 行の q/fsm/arm（tick 冒頭のサンプル値）だけでは実時間の窓を
  観測していない）**: R4 が検証する契約は §3.1 どおり **sampled WIFI_QUIET**
  （20 Hz サンプリング + 検出遅延 ≤ 1 tick は §3.1 が明示的に受容し、arm_pending
  ホールド窓の先行マージンで吸収する設計）であることを C4 文言・手順書に明記する。
  加えて trace ビルドの `bp` op 行には **op 直前の SharedState 即時再サンプル**
  `fsm2=<n|-> arm2=<0|1|->` を追加記録する（観測のみ、ガード判断には一切不使用 —
  リリース挙動と等価性を保つ）。判定（ゲート1第13回指摘1対応 — セットアップ資源の
  確保が**実時間の** Balancing/arm 窓に落ちたという結果事実を WARN で流さない）:
  - **セットアップ資源クラスの bp（`udpr=0`、およびログ先頭の bp）**で
    `fsm2=2 ∨ arm2=1 ∨ fsm3=2 ∨ arm3=1`（pre/post いずれかが実窓内 = 確保区間
    `[t, t+dur]` が窓開始と重なった可能性を排除できない）→ **FAIL**（C4 の実時間
    主張への違反。sampled 契約に準拠した正しいガードでもサブ tick レースで理論上
    起こり得るが、その場合も「確保が実窓と重なった」事実は変わらないため合格させ
    ない。発生確率は数 ms 窓であり再取得で対処、根拠行と経緯（sampled 文脈は
    !QUIET だった旨）を出力に明記）。pre/post とも非窓なら「確保完了時点でも窓外」
    として PASS。
  - **`udpr=1` の再利用送信 bp** で `fsm2=2 ∨ arm2=1 ∨ fsm3=2 ∨ arm3=1` → WARN
    （確保を伴わない送信であり §3.1 が許可する Balancing 中送信そのもの。件数・
    根拠行付きで報告のみ）。
- **R5（T5, C5）**: `arm=1` 文脈で `ev=disc r≠8` が発生するエピソード: R3 と同一の
  判定（ev/op ベース、禁止窓の op 全集合 = disc 1 件のみ、完了確認イベント、
  最終状態健全性）を `arm=1 ∨ fsm=2` の連続窓に対して適用する。
- **合成ログテストの必須ケース**（ゲート1第1回指摘2対応）: 実ガードの host テストで
  既に表現されているレースを checker テストにも対で用意する —
  「同一 tick 内の disc(r≠8)+gotip（librp が st に一度も現れない正しいキャンセル）」
  「abort 開始と完了が近接し st では ab=1 を観測しない系列」「stale snapshot 中の
  prewarm 試行（q=1 ∧ fsm=1 ∧ arm=0 の bp → R4 FAIL）」「禁止窓内の bp/radio_off
  混入（R2/R3 FAIL）」。それぞれ PASS 系と FAIL 系の両方を持つこと。

## 6. ベンチ手順（telemetry_workflow.md 新節の骨子）

共通: `cp include/wifi_secrets.h.example include/wifi_secrets.h` 後に実験環境の値を設定、
`pio run -e m5stack-core2-trace -t upload`、キャプチャは
`pio device monitor -e m5stack-core2-trace | tee logs/trace/<scenario>.log`
（921600 baud、`logs/` は既存 .gitignore 対象）。**キャプチャは必ず「モニタ起動 →
Core2 の RST ボタンでリセット」の順**で開始し、boot 行からログに含める（先頭欠落は
チェッカが INVALID にする。ゲート1第14回指摘1対応）。機体はスタンド上。「Balancing に
入れる」は機体をベンチ上で直立保持して FSM を遷移させる。hb/st 行で状態を確認しながら
操作する。**各シナリオとも、最後の操作から 2 秒以上（hb 2 本分）待ってからキャプチャを
停止する**（ログ末尾の完全性要件、§5 共通妥当性 — 満たさないログはチェッカが
INVALID にする）。

- **T1**: AP を停止したまま起動 → Idle で begin/失敗/バックオフが落ち着くのを待つ
  （st で `cing=0` を確認）→ 直立保持で Balancing へ → 10 s 保持 → ログ終了。
- **T2**: secrets を実在しない SSID（または誤パスワード）にして書込み → 起動 →
  st で `cing=1` を確認した瞬間に直立保持で arm → Balancing。CONNECTING 窓（数秒）を
  外した場合は Idle に戻して再試行（チェッカがエピソード不成立を報告する）。
- **T3+T3b（単一の連続キャプチャ必須。**ゲート1第5回指摘2対応 — R3b のヒット条件 (i)
  は同一ログ内に最初の one-shot エピソード完結を要求するため、キャプチャを途中で
  区切ると T3b は構造的に不成立になる）: 正しい secrets で起動 → Idle で GOT_IP・
  prewarm 完了（`udpr=1`）を確認 → Balancing へ → **AP の電源断** → one-shot 発火と
  キャンセルを記録（ここまでが R3 の対象）→ キャプチャを継続したまま **AP を復電**し
  機体を Idle に戻す → st/hb で `conn=1 ∧ udpr=1`（再接続 + prewarm 完了）を確認 →
  再び Balancing へ → **2 度目の AP 電源断** → そのまま**直立保持を 20 秒以上継続**
  （R3b の観測窓下限。遅延して届く内部接続失敗イベントの検出に必要）→ ログ終了。
  同一ログを `--scenario t3` と `--scenario t3b` の両方に掛けて判定する。
- **T5**: 正しい secrets で起動 → Idle で接続完了 → 直立保持の**ホールド中**
  （st で `arm=1` の間）に AP 喪失イベントを発生させる → そのまま Balancing まで
  遷移 → 最終操作から 2 秒以上待ってログ終了。
  **決定的にするための trace 専用ホールド延長（ゲート1第12回指摘2対応 — リリース
  構成のホールド約 1 s に対し AP 電源断の STA_DISCONNECTED 配送（ビーコン喪失
  タイムアウト）は非同期数秒で、電源断の反復では窓内到達を保証できない）**:
  ホールド時間定数（`app_config.h`、実装時に特定）を `#ifndef` ガード化し
  （リリース値は不変）、trace env のみ `-DBSL_TRACE_ARM_HOLD_S=5.0` 相当で **5 s へ
  延長**する。ホールド延長は arm_pending 窓を広げるだけで C5 の検証意味論
  （arm_pending 中の abort 挙動）を変えない（他シナリオへの影響も quiet 窓が
  広がる方向のみ）ことを手順書に注記する。AP 側にクライアント切断（kick/deauth）
  機能があればそれを第一選択とし、なければ「AP 電源断 → ホールド継続」で
  ビーコン喪失タイムアウト到達を待つ。`arm=1`（または延長ホールド）中の hit が
  得られないログはシナリオ不成立の FAIL（再試行指示はチェッカ出力に含める）。

## 7. 検証（PR 前に必須）

1. `scripts/verify_builds.sh` 全パス（(0) バージョン assert / (1) ¬secrets /
   (2) secrets / (2b) trace × secrets の 3 ビルド）。
2. `pio test -e native` 全パス（既存回帰なし + `lastWifiQuiet()`/`drainedEpoch()`
   反映テスト + D11 emitter 検証（両インターリーブでの ep pre-drain 保証・grammar
   一致）追加）。
3. `python3 -m unittest discover tools/telemetry/tests` 全パス（trace_check の
   R1〜R5 各 PASS/FAIL/INVALID 系合成ログテスト + emitter 生成 fixture の相互検証
   追加）。`scripts/gen_trace_fixtures.sh` を再実行して fixture の diff がゼロで
   あること（fixture の鮮度確認）。
4. **リリースバイナリ不変性**（一回性検証、結果を PR 本文に記録。手順は hermetic に
   行う — ゲート1第1回指摘5対応）:
   - **クリーン worktree × 2**: `git worktree add` で base（`feat/udp-telemetry-phase1`
     HEAD）と本 PR HEAD をそれぞれ**新規ディレクトリ**に checkout する（`.pio/build`
     キャッシュの再利用・stale 入力の混入を構造的に排除。各 worktree は初回ビルド =
     クリーンビルド）。
   - **secrets 状態を両側で明示的に固定し、両経路を比較**: 各 worktree で
     (i) `include/wifi_secrets.h` なし（no-op 経路）、(ii)
     `cp include/wifi_secrets.h.example include/wifi_secrets.h`（有効経路、ダミー値は
     .example 由来で両 worktree 同一）の **2 variant** を順にビルドし、variant ごとに
     base ↔ PR を比較する（(ii) のビルド前に (i) の成果物ディレクトリを削除するか
     variant 別 `PLATFORMIO_BUILD_DIR` を使い、variant 間の artifact 再利用も排除）。
   - `BSL_FW_GIT` の差を除去するため全ビルドに
     `PLATFORMIO_BUILD_FLAGS='-UBSL_FW_GIT -DBSL_FW_GIT=\"00000000\"'` を付与
     （env 由来フラグが ini 由来より後段に来ることを `pio run -v` で確認。逆順なら
     ビルドログから確認できた順序を記録し、代替として両バイナリの各自ハッシュ文字列を
     同一プレースホルダへ置換して比較する）。
   - 比較方法（variant ごと）: (a) `firmware.bin` の esp_app_desc_t 領域（コンパイル
     時刻・日付を含むファイルオフセット 0x20 から 256 B）をゼロ埋めした上で SHA-256
     一致、(b) `xtensa-esp32-elf-nm --size-sort firmware.elf` の完全一致。
   - 2 variant × (a)(b) の全一致 = 「フラグなしビルドのコード・データ不変」と判定する。
5. trace env の動作確認はベンチ実機（ユーザ実施、§6 手順）。PR には合成ログによる
   チェッカのテスト結果と、上記 1〜4 の結果を記載する。

## 8. Git 運用

- ブランチ `feat/wifi-guard-trace` を `feat/udp-telemetry-phase1` から作成。
  スタック PR（base = `feat/udp-telemetry-phase1`）。PR #27 が先にマージされた場合は
  base を `dev/current-mode-xl330-fable5` へ付け替える。
- コミットは日本語・1 目的 1 コミット・`Co-Authored-By` トレーラ。
- `docs/plans/2026-07-06-handover-bench-instrumentation-and-audit.md`（作業指示書）・
  `docs/reviews/`・`prompt_memo.md`・`claude_best_fable.sh` はコミットしない。

## 9. リスクと限界

| リスク | 対応 |
|---|---|
| beginPacket プロキシはライブラリ実装依存（将来の framework 更新で確保タイミングが変わり得る） | platform pin + verify_builds のバージョン assert が既にあるため、更新時に D2 の再確認が強制される。確認済みバージョンをコードコメントに記録 |
| R4 の証明範囲は WiFiUDP セットアップ資源（tx_buffer/socket）に限定され、lwIP/ドライバ層の per-send 一時確保（pbuf 等）は対象外 | 許可された Balancing 中送信に内在する正常動作であり、上位計画 §3.1 の「重い操作」定義（socket 作成・初回 beginPacket）の外。C4 の判定範囲として §1・R4 に明記し、hb の heap/heapmin を参考観測として残す（合否条件にはしない） |
| Serial 印字による probe effect（telemetry task 内 最悪 ≈ 30 ms/tick のブロック） | フラッシュは guard 処理・送信完了後の tick 末尾 1 箇所のみで、被測定経路（abort 発行等）には挿入しない（D3/D9）。core 1 / prio 1 の telemetry task に閉じており ControlTask へは波及しない。trace ビルドは検証専用でリリースには存在しない。dt ヒストグラム比較（C 以外の既存チェック項目）は非 trace ビルドで実施する旨を手順書に明記 |
| trace リング/ガードキューの overflow で判定不能 | evdrop/evo をチェッカの INVALID 条件にし、黙って PASS しない |
| T2/T5 はタイミング依存でシナリオ成立に複数回試行が必要 | チェッカが「エピソード不成立」を明示報告。手順書に再試行手順を記載 |
| ベンチの「直立保持で Balancing」は実飛行（自立倒立）と厳密には異なる | 検証対象は Wi-Fi ガードの状態機械であり FSM 状態と Wi-Fi イベントの組み合わせのみが判定に関与するため、ベンチ遷移で等価。手順書に注記 |

**既知の実装/設計間の疑義（本計画のスコープ外だが判定規則に関係）**:

- `WifiGuard::handleAbortTick()`（`src/core/wifi_guard.h`）は abort リトライ時に
  WIFI_QUIET を再評価せず、quiet 解除後もリトライ・radio-off 終端へ進み得る。上位設計
  §3.1 の字句は「WIFI_QUIET **継続中に限り**有界リトライ」。チェッカはこの逸脱を
  FAIL として顕在化させる（§5 abort エピソード追跡）。ベンチで実測された場合は
  黙認せず、PR #27 系列の別課題としてユーザへ報告・エスカレーションする（本 PR では
  wifi_guard.h のロジックを変更しない）。
- T3b は「ライブラリ first_connect one-shot はブート後一度きり」という Phase 1 検証済み
  前提の**実機再検証**を兼ねる。仮に one-shot が `begin()` で再アームされる実装だった
  場合、telemetry の Ops を経由しない内部 `disconnect();begin();` が走るため op 行には
  現れないが、**イベントシグネチャ**（内部 disconnect の `ev=disc r=8`、begin 失敗の
  追加 `ev=disc r≠8`、AP 生存時は `ev=conn`/`ev=gotip`）として R3b (b) が検出し FAIL
  する（AP 生存時に接続が成立してしまうケースは guard の established-connection
  ラッチが有界キャンセルする — その abort op も同様に FAIL 証拠になる）。これは計装の
  誤りではなく前提モデルの反証であり、エスカレーション対象とする（手順書に判別方法を
  注記）。
