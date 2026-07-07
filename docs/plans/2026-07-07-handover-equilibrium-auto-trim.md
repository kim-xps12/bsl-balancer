# ハンドオーバー: 実機ブリングアップ完了 → 平衡点auto-trim実装 (2026-07-07)

前セッションからの引き継ぎ文書。作業ブランチ: **dev/udp-telemetry** (origin同期済み)。

## 1. このセッションで完了したこと

### ブランチ/PR
- PR #30 (実機ブリングアップ: コミッショニング機構削除+DXL実機適合) → dev/current-mode-xl330-fable5 へ**マージ済み**
- dev/udp-telemetry へ同内容をマージ済み (9303bb3)。以降のコミット:
  - `12edfeb` WifiGuard: WiFi.begin由来の一時停滞で自己abortする実測バグ修正 (FreshnessTracker 4tick耐性)
  - `1d5782e` 平衡点較正 kPitchEqDefaultRad=0.0925 (+5.3°実測) / 未検証トルク窓 即ラッチ→窓一本化 10→50ms
  - `b0f428c` 未検証トルク窓 50→150ms
  - `14d0d9a` ゲイン Kp 1.5→3.0, Kd 0.08→0.15 (※次項の経緯で1.5へ戻す予定)

### 実機で確定した事実 (詳細はメモリ bsl-balancer-hw-facts.md にも記録)
- DXL物理構成: RX=GPIO33/TX=GPIO32・1Mbaud (masterのPIO実装が正。for_arduino_ideは旧世代・参照禁止)
- Return Delay Time=0は不可 → raw25(50µs)。実バスは単発数%+バースト(>50msもあり)のトランザクション喪失
- 初期化はステップ毎5回+全体3回リトライで4/4ブート安定
- 「連続N周期成立」系条件は実バスで成立しない → 欠落周期は「情報なし=状態維持」で扱う (uprightHold修正済み)
- WiFi.begin()はcore0制御ループを最大~170ms停止させる (WifiGuard耐性化済み。ただしBALANCE中のWiFi接続処理は設計上保留のまま)
- E2Eテレメトリ疎通確認済み: 機体→UDP 20Hz→receiver→JSONLセッション

### 制御性能の診断 (テレメトリ実測)
- 旧ゲイン(Kp1.5): 3.2Hz・±7.5°(p2p15°)の**振幅一定リミットサイクル**。電流は同位相(r=0.77@lag0)・最大0.20A/0.9A・飽和1/81 → 発散ではなく権限不足の波形
- ユーザ実機観測: 「たまにピタッと静止→微小外乱で一気に振動」 = **スティクション(リレー非線形)リミットサイクルの決定的証拠** (線形不安定なら完全静止は不可能)
- 電流のゼロ点は振動折り返しの自然なゼロ交差 (コースト説は撤回済み)。電流デッドバンドは実測されず(摩擦は機構側)
- Kp=3.0での挙動: BALANCE進入直後にテレメトリ途絶+再起動 (発散目撃あり、ただし時系列は未記録)
- 診断グラフArtifact: https://claude.ai/code/artifact/53460202-0814-49a0-8fec-ae02877810e9

### 未解決の問題
1. **倒立中の再起動が複数回発生** (tickリセットで再起動自体は確証)。原因はブラウンアウト仮説だが未確定。
   → 対策候補: [BOOT]行に esp_reset_reason() 出力を追加すれば一発で判別 (未実装)。**バッテリー充電状態の確認も必須**
2. Kp=3.0が書き込まれたまま (較正計画ではKp=1.5へ戻して単変量評価する)
3. ベンチ計装 ([ST]/[WF]/[DXL]シリアル出力、バススキャン) が本番コードに残存 — いずれ整理要

## 2. 次のタスク: 平衡点auto-trim実装

- **設計計画**: `docs/plans/2026-07-07-equilibrium-auto-trim.md` (3層+イネーブラ構成、実装順序・検証手順・未決事項付き)
- **サーベイ全文**: `docs/research/2026-07-07-equilibrium-auto-trim-survey.md` (文献+OSS実装15件)
- 進行: ゲート1 (/codex:adversarial-review で計画書を叩く) → ユーザ確認 → Sonnet実装 → ゲート2 (/codex:review) → 実機検証 → PR
- 計画書の未決事項4件 (trim更新符号の確定方法 / Fallen時のtrim保持判断 / 自動保存と§9.1保存ゲートの整合 / FFとプラウジビリティ監視の閾値干渉) はゲート1の重点論点

## 3. 運用ノウハウ (実験手順)

- **電源投入/RSTは機体を寝かせた状態で行い、15秒待ってWi-Fi接続後に立てる** (直立ブートすると即auto-arm→BALANCE→WIFI_QUIETでWi-Fiが永遠に繋がらない)
- 受信: `python3 tools/telemetry/bsl_telemetry/receiver.py --port 45678 --log-root logs/telemetry` (PC IP: 192.168.0.19)
- include/wifi_secrets.h は設定済み (.gitignore対象。SSID/パスワード入り)
- シリアル書込/監視ポート: /dev/cu.usbserial-556F0043751。pio は `~/.platformio/penv/bin/pio`
- 監視スクリプトはセッションログ(logs/telemetry/*/raw.jsonl)をtail追尾する形式が確立済み (状態遷移・10秒サマリ・BALANCE統計)
- FAULTはラッチ (リセットのみで復帰)。転倒3回/30秒でFallEscalationラッチに注意
- native テスト: `~/.platformio/penv/bin/pio test -e native` (現在68/68 PASS)

## 4. 新規セッション開始プロンプト

```
/goal docs/plans/2026-07-07-handover-equilibrium-auto-trim.md を熟読し、平衡点auto-trim実装
(docs/plans/2026-07-07-equilibrium-auto-trim.md) をゲート1から実機検証まで完遂すること

- 進行: ゲート1(/codex:adversarial-review、計画書の未決事項4件を重点) → ユーザ確認 →
  Sonnet実装(段階E→L2→L3、各段でnativeテスト追加) → ゲート2(/codex:review) → 実機検証 → PR発行
- 実装前の下準備として: (1) Kp既定値を1.5へ戻す (2) [BOOT]行にesp_reset_reason()を追加
  (倒立中再起動の原因確定用)。この2点は軽微なのでゲート1と並行してよい
- 実機検証はテレメトリ(logs/telemetry/)で定量評価する。手順・運用ノウハウはハンドオーバー§3参照。
  ベンチ検証の各段判定基準は計画書§4に従うこと
- レビューは10ラウンド到達で必ず中間報告し継続可否を確認すること
- メインセッション(Fable 5)は設計・監査・レビューに専念し、実装・調査はSonnet5サブエージェント
  (Dynamic Workflows)へ委譲すること
- 不明瞭な点があれば私に適宜その都度忌憚なく質問してヒアリングすること
```
