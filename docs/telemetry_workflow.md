# UDP テレメトリ実験ワークフロー（coding agent 向け）

対象: Phase 1 UDP テレメトリ機構（`docs/plans/2026-07-05-udp-telemetry-phase1.md` が設計の正）。
本文書は「実験を回して、ログを observability の高い形で agent に渡し、パラメータを
チューニングする」までの手順を coding agent（Claude Code / Codex）向けにまとめる。
schema・Wi-Fi 安全設計の詳細は計画書を参照し、本文書と計画書が矛盾する場合は計画書を正とする。

## 0. 信頼境界（必読）

- 本 framework は**信頼できる私有 LAN**での運用を前提とする。**敵対的 LAN では使用しない。**
- 受信側の source IP pinning（先頭 packet の送信元 IP に固定、または `--device-ip` 明示指定）は
  **別デバイス・simulator の誤混入を防ぐための仕組みであり、認証ではない**。同一 LAN 上の
  能動的攻撃者（IP spoofing・偽 packet 注入）に対する防御は提供しない。
- 全 session の `metadata.json` には `"authenticated": false` が常に記録される。この値が
  `true` に変わっていない限り、ログの完全性は「私有 LAN 上に他デバイスがいない」という
  運用上の前提にのみ依存している。
- **HMAC 対応（フォローアップ、Phase 1 スコープ外）**: packet に HMAC-SHA256 署名を付与し
  認証境界を実現する設計を今後追加する。firmware 側は `mbedtls`（ESP-IDF 同梱）の
  HMAC-SHA256、PC 側は標準ライブラリ `hmac` モジュールを使う想定。共有鍵は firmware 側
  `include/wifi_secrets.h`、PC 側は receiver の起動引数（例: `--hmac-key-file`）で与える。
  これが実装されるまでは、本節の信頼境界を超えた環境（学校・共用オフィス Wi-Fi 等）での
  実験ログをチューニング判断の根拠にしない。

## 1. 実験の回し方（基本フロー）

1. **receiver 起動**（PC 側、実験開始前に立ち上げておく）。
2. **Core2 の電源を ON** にする。
3. **Idle（横倒し/スタンド上）で Wi-Fi 接続完了を待つ。** これは運用上の制約ではなく
   Wi-Fi 安全設計（計画書 §3.1 `WIFI_QUIET` 述語）の直接の帰結である。telemetry task は
   `fsm == Balancing` または `arm_pending == true`（直立ホールド進行中）の間は
   `WiFi.begin()` を一切発行しない。**auto-arm で Wi-Fi 接続前に Balancing が始まってしまうと、
   非 Balancing に戻るまで telemetry は開始されない。** そのため、機体を横倒しまたは
   スタンド上に置いた Idle 状態のまま、Core2 のタッチパネル表示や receiver 側のログで
   packet 受信が始まったことを確認してから倒立を開始すること。
4. **倒立実験を実施する。** Balancing 中も UDP 送信済み資源（`udp_ready`）が確保済みであれば
   送信は継続する。制御ループとは物理的に分離されたタスクなので、telemetry の状態を気にして
   実験を中断する必要はない（receiver を落としても制御は継続する — §6 参照）。
5. **実験終了。** 機体を停止・回収する。
6. **receiver を終了する。** 無通信が既定 10 秒続けば自動で session が close するほか、
   `Ctrl-C` でも `summary.json` まで書き出してから終了する。複数本を続けて実験する場合は
   receiver を起動したままにして良い（無通信 10 秒ごとに session が区切られる）。
7. **analyzer を実行**し、`summary.json` / `events.jsonl` を生成する。
8. **report を実行**し、agent へ渡す（§3 の引き渡し順序に従う）。

## 2. コマンド例

```bash
# 1) receiver 起動（実験前に立ち上げ、実験中は起動したままにする）
python3 tools/telemetry/bsl_telemetry/receiver.py --port 45678 --log-root logs/telemetry

# 2) 実験終了後: 生成された session を解析
python3 tools/telemetry/bsl_telemetry/analyzer.py logs/telemetry/<session_id>

# 3) agent へ渡す digest を生成
python3 tools/telemetry/bsl_telemetry/report.py logs/telemetry/<session_id>

# 実機なしで receiver / analyzer / report を検証したい場合（simulator）
python3 tools/telemetry/bsl_telemetry/simulator.py --port 45678 --scenario fall
```

`tools/telemetry/README.md` にオプション一覧（`--device-ip` 等）を記載する。`logs/` は
`.gitignore` 対象であり、実験ログはリポジトリにコミットしない。

firmware 側で telemetry を有効化するには、送信先 PC IP・Wi-Fi 認証情報を
`include/wifi_secrets.h` に定義する（`.gitignore` 対象、実体はコミットしない）:

```bash
cp include/wifi_secrets.h.example include/wifi_secrets.h
# SSID / password / PC IP / UDP port を実験環境に合わせて編集してからビルド・書込
```

`include/wifi_secrets.h` が存在しない状態でビルドすると telemetry task は no-op（無効）に
なる。有効/無効いずれの経路も `scripts/verify_builds.sh` で hermetic に検証されている
（計画書 §5）。

## 3. agent への引き渡し順序（token 節約）

保存済みログは大きくなり得るため、次の順で必要最小限だけ読む。**`raw.jsonl` を先頭から
全読み込みすることは禁止**（1 session で数千行になり得る）。

1. `logs/telemetry/<session_id>/metadata.json` — 実験条件・`device_id`・firmware 版数・
   `tags`/`notes`。
2. `logs/telemetry/<session_id>/summary.json` — loss 率・dt 統計・event 件数・
   `recommended_windows`（agent が最初に見るべき `seq` 区間）。
3. `logs/telemetry/<session_id>/events.jsonl` — 転倒・FAULT 遷移・飽和区間・overrun 増加区間
   などの検出済み event。
4. `recommended_windows` に挙げられた区間の raw packet 抜粋のみを読む。
5. それでも情報が足りない場合に限り、`report.py` の出力や `analyzer.py` の結果を根拠に
   **bounded な raw window**（明示した `seq` 範囲）だけを追加で読む。

`report.py` は summary + events + 推奨 window の raw 抜粋を markdown で stdout に出力するので、
通常はこれを読むだけで十分である。

## 4. パラメータ変更の 2 手段

### a) `app_config.h` 編集 + 再ビルド + USB 書込（実験の合間）

`src/app_config.h`（全定数の SSOT）の値を編集し、`pio run -e m5stack-core2 -t upload` で
書き込む。ビルドし直すため `fw`（packet 内の git 短縮ハッシュ）が変わり、どのビルドでの
実験かが telemetry ログ側から自動的に判別できる。ゲイン初期値（`kPitchKp` / `kPitchKi` /
`kPitchKd` 等）や FSM 閾値（`kFallThresholdRad` 等）のように、実機タッチパネルから
触れない定数はこの手段でのみ変更できる。実験中の変更はできない（USB 接続・再起動を伴う）。

### b) 実機タッチパネル + SAVE（実験中に随時可能）

UI（`src/tasks/ui_task.cpp`）は `Kp` / `Ki` / `Kd` / `PitchEq` の増減ボタンと `SAVE` ボタンを
提供する。タッチ操作は即座に `SharedState::pushParam()` 経由で ControlTask に反映され、
`SAVE` を押すと NVS（`hw::ParamStore`）に永続化される。再ビルド不要でその場の実験結果を見ながら
微調整できる一方、`fw` ハッシュは変化しないため、この経路での変更は telemetry ログからは
自動判別できない。

### 実験条件の記録方法

- タッチパネルでゲインを変更した場合は、**その回の session を receiver から見て操作者が
  `metadata.json` の `notes`（または `tags`）に手動で記録する**（例:
  `"notes": "Kp=1.8 Ki=0.0 Kd=0.10 をタッチパネルで変更し SAVE 済み"`）。
- `app_config.h` を変更して再ビルドした場合は、packet の `fw` フィールド（git 短縮ハッシュ）と
  `metadata.json` の firmware 記録を突き合わせれば、どのソース状態での実験かを機械的に
  再構成できる。ただし「どの定数をどの値に変えたか」は git log 側の情報であり、
  `notes`/`tags` に実験の目的を書いておくと後から追跡しやすい。
- `dev`（デバイス ID、eFuse MAC 由来）も `metadata.json` に記録される。複数の Core2 個体を
  使い分ける場合の突合に使う。

## 5. 異常判別表

| 観測される現象 | 意味 | 補足 |
|---|---|---|
| `seq` にギャップがある | UDP パケット損失 | 20 Hz 前提で loss 率を算出。目安 5% 未満 |
| `snap_valid:false`, `reason:"read_fail"` | telemetry task 側の seqlock 競合（read 失敗） | 診断 datagram。`read_fail` 累積カウンタで頻度を確認 |
| `snap_valid:false`, `reason:"trunc"` | JSON 生成時のバッファ超過（フォーマット溢れ） | seqlock とは無関係。packet フォーマット側の不具合を疑う |
| `loop`（`loop_count`）の隣接 packet 差分がゼロ | ControlTask がストールしている | Wi-Fi/telemetry 側とは独立の制御ループ異常。最優先で調査 |
| 完全沈黙（一定時間 packet が来ない） | Wi-Fi 断、または Core2 の電源断 | receiver は無通信 10 秒（既定）で session を自動 close する |
| `seq` / `t_us` / `loop` が負の差分（巻き戻り） | デバイス再起動 | analyzer が reboot event として記録し、差分計算を再アンカーする |

参考: packet の `fsm` は `core::FsmState`（`Initializing=0, Idle=1, Balancing=2, Fallen=3,
Disarmed=4, Fault=5`）、`fault` は `core::FaultReason`（`None=0` 以降は
`src/core/safety_fsm.h` 参照）をそのまま整数化したものである。

## 6. 実機検証チェックリスト（計画書 §5 受け入れ基準・全転記）

実機検証（Wi-Fi 実環境・倒立動作）はユーザ実施とする。以下をすべて確認する:

- [ ] telemetry 有効時、`dt > 1.1x` 周期のサイクル比率が無効時比で悪化しない（同条件・
      同時間の比較で有意差なし。累積 `dt_h` ヒストグラムの bin 比率で比較する）。
- [ ] `overrun_total` の増加がない（Balancing 中）。
- [ ] 20 Hz telemetry の loss < 5%。
- [ ] receiver 停止・Wi-Fi AP 停止で制御が継続する。
- [ ] AP 停止中の Balancing（非 CONNECTING）で telemetry FSM 起点の `WiFi.begin()`/mode 変更/
      `disconnect()` が発行されない（autoReconnect/persistent 無効化の実機確認）。
      → §7「計装（trace）ビルドによるガード検証」のシナリオ T1・規則 R1 で機械判定。
- [ ] CONNECTING 中の Balancing 遷移ではちょうど 1 回の abort `disconnect()` のみ発行され、
      fresh 非 Balancing snapshot 観測まで Wi-Fi 操作が再開されない。
      → §7「計装（trace）ビルドによるガード検証」のシナリオ T2・規則 R2 で機械判定。
- [ ] GOT_IP 後の AP 喪失 × Balancing 中: ライブラリ one-shot 再接続が発火しても
      cancel-watchdog が有界時間（目安 ≦ 約 1 telemetry 周期 + disconnect 処理）で
      キャンセルする。
      → §7「計装（trace）ビルドによるガード検証」のシナリオ T3・T3b・規則 R3・R3b で機械判定。
- [ ] `fsm == Balancing || arm_pending` 中に telemetry 経路の **WiFiUDP セットアップ資源
      （UDP socket・1460 B tx_buffer）の作成**が発生しない（prewarm 済み資源の再利用のみ）。
      → §7「計装（trace）ビルドによるガード検証」の全シナリオ横断・規則 R4 で機械判定。

      > **改訂注記**: 本項目は当初「malloc/socket 作成」という一般表現だったが、
      > [`docs/plans/2026-07-06-wifi-guard-trace-bench.md`](plans/2026-07-06-wifi-guard-trace-bench.md)
      > §5 R4「上位文書との文言一致」に基づき、判定対象を WiFiUDP レベルのセットアップ資源
      > （socket 作成・`tx_buffer` の遅延確保）へ精密化した。これは同計画書 §3.1 が
      > 「重い操作」と定義した対象そのものであり、受け入れ基準を黙って縮小したものではなく
      > 判定範囲と文言を一致させる改訂である。判定対象は sampled 契約（同計画書 §5 R4
      > 「sampled 契約の明示」）である点にも留意する — R4 は §3.1 どおりの **sampled
      > WIFI_QUIET**（20 Hz サンプリング + 検出遅延 ≤ 1 tick を明示的に許容し、
      > `arm_pending` ホールド窓の先行マージンで吸収する設計）を検証するものであり、
      > lwIP/ドライバ層の per-send 一時確保（pbuf 等）は判定対象外（同計画書 §9 参照）。
- [ ] 直立ホールド（`arm_pending`）中の AP 喪失で one-shot 再接続が発火しても、
      cancel-watchdog が 1 回の abort でホールド中に中断しフラグをクリアする。
      → §7「計装（trace）ビルドによるガード検証」のシナリオ T5・規則 R5 で機械判定。

PR 前に必須の非実機検証（`pio run -e m5stack-core2` の enabled/disabled 両経路ビルド、
`pio test -e native`、`python3 -m unittest discover tools/telemetry/tests`、simulator による
E2E ループバック）は計画書 §5 を参照。

## 7. 計装（trace）ビルドによるガード検証

§6 チェックリストのうち、外部から観測できない Wi-Fi ガード内部動作に関する 5 項目
（C1〜C5、上表の → 参照）は、通常の telemetry ビルドでは目視確認しかできない。本節は
これを**計装（trace）専用ビルド**とその出力を機械判定するチェッカスクリプトで
PASS/FAIL/INVALID 判定に置き換える手順を定める。設計の正は
[`docs/plans/2026-07-06-wifi-guard-trace-bench.md`](plans/2026-07-06-wifi-guard-trace-bench.md)
（以下「trace 計画書」）であり、本節と矛盾する場合は trace 計画書を正とする。

trace ビルドは**検証専用**であり、リリース env（`m5stack-core2`）のバイナリには一切
影響しない（`BSL_WIFI_GUARD_TRACE` フラグの `#if` 内に閉じ込められている。trace 計画書
§2 設計原則 1・§7 検証参照）。実運用フローで trace env を書き込んではならない。

### 7.1 ビルド・書込

```bash
cp include/wifi_secrets.h.example include/wifi_secrets.h
# SSID / password / PC IP / UDP port を実験環境に合わせて編集
pio run -e m5stack-core2-trace -t upload
```

`m5stack-core2-trace` は `m5stack-core2` を `extends` した専用 env で、trace 計装
（`-DBSL_WIFI_GUARD_TRACE=1`）とモニタボーレート 921600、および T5 専用のホールド延長
（7.3 節参照）のみを追加する。それ以外のビルド設定はリリース env と同一である。

### 7.2 キャプチャ手順

1. **モニタを先に起動**し、その後 Core2 の **RST ボタンでリセット**する（この順序が
   必須。逆にするとブート直後の `boot` 行を取りこぼし、チェッカがログ全体を
   INVALID と判定する）:
   ```bash
   pio device monitor -e m5stack-core2-trace | tee logs/trace/<scenario>.log
   ```
   （921600 baud。`logs/` は既存 `.gitignore` 対象でありコミットしない）。
2. 機体はスタンド上に固定する。「Balancing に入れる」は機体をベンチ上で直立保持して
   FSM を遷移させることを指す。
3. `hb`/`st` 行（20 tick=1 s ごとの心拍・状態変化行）を見ながら操作し、シナリオごとの
   目的の窓（`cing`/`fsm`/`arm` 等）に入ったことを確認してから次の操作へ進む。
4. **各シナリオとも、最後の操作から 2 秒以上（`hb` 2 本分）待ってからキャプチャを
   停止する**。これはログ末尾の完全性要件（trace 計画書 §5 共通妥当性）であり、
   満たさないログはチェッカが機械的に INVALID とする。

### 7.3 シナリオ T1/T2/T3+T3b/T5

- **T1**（→ R1・C1）: AP を停止したまま起動 → Idle で `begin`/失敗/バックオフが落ち着く
  のを待つ（`st` で `cing=0` を確認）→ 直立保持で Balancing へ → 10 s 保持 → ログ終了。
- **T2**（→ R2・C2）: secrets を実在しない SSID（または誤パスワード）にして書込み →
  起動 → `st` で `cing=1` を確認した瞬間に直立保持で arm → Balancing。CONNECTING 窓
  （数秒）を外した場合は Idle に戻して再試行する（チェッカがエピソード不成立を報告する）。
- **T3+T3b**（→ R3/R3b・C3。**単一の連続キャプチャが必須**。R3b のヒット条件は同一ログ内
  での最初の one-shot エピソード完結を前提とするため、途中でキャプチャを区切ると T3b は
  構造的に不成立になる）:
  1. 正しい secrets で起動 → Idle で GOT_IP・prewarm 完了（`udpr=1`）を確認。
  2. Balancing へ → **AP の電源断** → one-shot 発火とキャンセルを記録（ここまでが R3 の
     対象）。
  3. キャプチャを継続したまま **AP を復電**し機体を Idle に戻す → `st`/`hb` で
     `conn=1 ∧ udpr=1`（**再接続必須**）を確認する。
  4. 再び Balancing へ → **2 度目の AP 電源断** → そのまま**直立保持を 20 秒以上継続**
     （R3b の観測窓下限。ライブラリ内部の遅延失敗イベント検出に必要）→ ログ終了。
  同一ログを `--scenario t3` と `--scenario t3b` の両方に掛けて判定する。
- **T5**（→ R5・C5）: 正しい secrets で起動 → Idle で接続完了 → 直立保持の**ホールド中**
  （`st` で `arm=1` の間）に AP 喪失イベントを発生させる → そのまま Balancing まで
  遷移 → 最終操作から 2 秒以上待ってログ終了。
  **trace 専用ホールド延長**: リリース構成のホールド（約 1 s）では AP 電源断からの
  `STA_DISCONNECTED` 配送が非同期数秒かかるため窓内到達を保証できない。trace env のみ
  `-DBSL_TRACE_ARM_HOLD_S=5.0` 相当でホールドを **5 s へ延長**する（`app_config.h` の
  `#ifndef` ガードによりリリース値・型・意味は不変）。この延長は `arm_pending` 窓を
  広げるだけであり、**C5 の検証意味論（arm_pending 中の abort 挙動）は変えない**
  （trace 計画書 D8/§6 T5 参照）。AP 側にクライアント切断（kick/deauth）機能があれば
  それを第一選択とし、なければ「AP 電源断 → ホールド継続」でビーコン喪失タイムアウト
  到達を待つ。

### 7.4 チェッカ実行例

```bash
python3 tools/telemetry/bsl_telemetry/trace_check.py --scenario t1 logs/trace/t1.log
```

`--scenario` は `t1|t2|t3|t3b|t5` を指定する。R4（WiFiUDP セットアップ資源の判定）と
ログ妥当性検査（`s` 連続性・`evdrop`/`evo`/`flmax`/`stkmin`・ログ末尾完全性等）は
`--scenario` によらず全シナリオで常時実行される。T3+T3b は同一ログファイルに対して
`--scenario t3` と `--scenario t3b` を別々に実行する。終了コード 0 = 全 PASS。

### 7.5 C×↔T×↔R× 対応表

| チェックリスト項目（§6） | シナリオ | 判定規則 |
|---|---|---|
| C1 | T1 | R1 |
| C2 | T2 | R2 |
| C3 | T3 / T3b | R3 / R3b |
| C4 | 全シナリオ横断 | R4 |
| C5 | T5 | R5 |

### 7.6 判定分類（PASS/FAIL/INVALID/WARN）

- **PASS**: 該当規則の条件をすべて満たした。
- **FAIL**: 該当規則への違反を検出した。ゲートをブロックする。**有効なヒットエピソード
  内**で `radio_off`/`latch=1`（有界キャンセル未完了・終端 radio-off 到達）に達した
  場合も FAIL（「取り直し」扱いに格下げしない）とし、エスカレーション対象として
  報告する（7.7 節・trace 計画書 §5 latch/radio-off の分類参照）。
- **INVALID**: ログそのものの証拠能力が疑わしい（`boot` 行の欠落・多重、`s` の欠番/
  重複/破損、`evdrop`/`evo` の増加、`flmax` 超過、`stkmin` 不足、ログ末尾の不完全、
  観測窓不足等）。**再取得**して判定をやり直す。ヒットエピソードと無関係な箇所
  （例: ログ冒頭から `latch=1` が立っている等）で証拠能力が疑われる場合も INVALID
  とする。
- **WARN**: 判定をブロックしない参考報告。`udpr=1`（確保を伴わない再利用送信）の
  `bp` が実時間の Balancing/arm 窓と重なった場合など、設計上許可された正常動作に
  対する情報提示に限る。

### 7.7 エスカレーション対象（trace 計画書 §9 の要約）

チェッカが以下を検出した場合、本 PR の実装不備ではなく**別課題として利用者へ報告**する
（wifi_guard.h のロジック変更は本 PR のスコープ外）:

- **abort リトライ時の WIFI_QUIET 再評価**: `WifiGuard::handleAbortTick()` は abort
  リトライ時に WIFI_QUIET を再評価しない実装になっており、quiet 解除後もリトライ・
  radio-off 終端へ進み得る（上位設計は「WIFI_QUIET 継続中に限り」有界リトライを許可）。
  R2/R3/R5 のエピソード追跡でこの逸脱がベンチで実測された場合、黙認せず PR #27 系列の
  別課題としてエスカレーションする。
- **R3b FAIL = one-shot 再アームの疑い**: R3b は「ライブラリ first_connect one-shot は
  ブート後一度きり」という前提の実機再検証を兼ねる。R3b がイベントシグネチャ
  （内部 disconnect の `ev=disc r=8`、begin 失敗の追加 `ev=disc r≠8`、AP 生存時の
  `ev=conn`/`ev=gotip`）を検出して FAIL した場合、計装の誤りではなく**前提モデルの
  反証**であり、エスカレーション対象とする。

## 8. その他フォローアップ（Phase 1 スコープ外）

- **MCP サーバ**（Phase 2）: session 一覧・summary・event・raw window を tool/resource として
  agent に公開する。
- **binary packet 化**: 現状は JSON 1 行/packet。packet size や生成コストが実測で問題になった
  場合に検討する。
- **DYNAMIXEL present velocity の高頻度読み**: 現状は Snapshot 経由の値のみ。専用の高頻度
  読み取りはバス負荷とのトレードオフがあるため、必要になってから追加する。

その他の非スコープ項目（PC→Core2 コマンドチャネル、packet の HMAC 署名の実装本体、
制御タスク内の p95 直接計測等）は計画書 §7 を参照。
