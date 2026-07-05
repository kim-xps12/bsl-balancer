# bsl_telemetry — PC 側 UDP テレメトリツール（Phase 1）

M5Stack Core2 firmware（`src/tasks/telemetry_task.{h,cpp}`）から 20 Hz で送られる
UDP テレメトリ（packet schema v1）を受信・保存・解析するための PC 側ツール一式。
依存ゼロ（Python 3.11+ 標準ライブラリのみ）で `python3` から直接実行できる。

設計の詳細は
[`docs/plans/2026-07-05-udp-telemetry-phase1.md`](../../docs/plans/2026-07-05-udp-telemetry-phase1.md)
の section 3.2・3.1 を参照。

## 構成

```
tools/telemetry/
  bsl_telemetry/
    schema.py        # packet schema v1 定義・validation（full / diagnostic の2 variant）
    receiver.py       # UDP 受信 -> session dir 作成 -> raw.jsonl append
    analyzer.py       # post-run: raw.jsonl -> events.jsonl, summary.json（純関数・再生成可能）
    report.py         # session の markdown digest を stdout へ（agent/人間向け第一資料）
    simulator.py      # 合成 packet 送信（実機なしで receiver/analyzer を検証）
  tests/
    test_schema.py
    test_analyzer.py
    test_e2e.py       # simulator -> receiver -> analyzer -> report をループバックで通す
  README.md
```

## クイックスタート

```bash
# 1) receiver を起動（実験前に立ち上げておく。Ctrl-C で summary.json まで書いて終了）
python3 tools/telemetry/bsl_telemetry/receiver.py --port 45678 --log-root logs/telemetry

# 2) 実機なしで動作確認したい場合は、別ターミナルから合成 packet を送る
python3 tools/telemetry/bsl_telemetry/simulator.py --port 45678 --scenario fall

# 3) 実験終了後（receiver が無通信10秒で自動close、または Ctrl-C 後）に再解析したい場合
python3 tools/telemetry/bsl_telemetry/analyzer.py logs/telemetry/<session_id>

# 4) coding agent に渡す最初の資料（markdown）を作る
python3 tools/telemetry/bsl_telemetry/report.py logs/telemetry/<session_id>
```

`receiver.py` は最初の valid packet（full/diagnostic どちらでも可）で session を自動
開始し、close 時に `analyzer.py` を内部で呼んで `events.jsonl`/`summary.json` を書き出す。
そのため通常は `analyzer.py` を手動実行する必要はない。raw.jsonl さえ残っていれば
`analyzer.py <session_dir>` でいつでも再生成できる（`events.jsonl`/`summary.json` は
raw.jsonl + metadata.json から再生成可能な派生物という位置づけ）。

## session ディレクトリ構成

```
logs/telemetry/<session_id>/          # session_id = YYYYMMDD-HHMMSS_<dev>_run-NNN
  metadata.json   # session識別情報、pinning設定、rejected_packets集計、authenticated:false
  raw.jsonl       # 受理された packet のみ。1行 = {"host_receive_ns", "source_ip", "packet"}
  events.jsonl    # analyzer が raw.jsonl から検出した event（loss/reboot/fsm遷移/sat連続区間等）
  summary.json    # agent が最初に読む要約（loss率、dt統計、recommended_windows 等）
```

## receiver の受信ポリシー（信頼境界）

- 既定は **first-packet source IP pinning**: session を開始した packet の source IP
  に固定し、以降それ以外の source からの packet は保存せず拒否（件数・source別に
  `summary.json` の `rejected_packets` へ集計、黙って捨てない）。
- `--device-ip <ip>` で明示 allowlist 指定も可能（pinning より優先）。
- **これは認証ではない**: source IP pinning は誤設定・複数デバイス・simulator の
  混入を防ぐための仕組みであり、同一 LAN 上の能動的攻撃者（IP spoofing 等）への
  認証境界ではない。すべての session の `metadata.json` に `"authenticated": false`
  が必ず記録される。信頼できる私有 LAN での運用を前提とする。

## packet schema v1

`snap_valid` で判別する2 variant（`bsl_telemetry/schema.py` が正式定義）:

- **full**（`snap_valid: true`）: 完全な Snapshot 由来のサンプル
  （`theta`/`fsm`/`dt_h` 等）。
- **diagnostic**（`snap_valid: false`）: seqlock read 失敗（`reason: "read_fail"`）
  または送信buffer truncation（`reason: "trunc"`）で firmware が代わりに送る沈黙防止用
  packet。`seq`/`tick`/`t_us`/`dev`/`fw`/`read_fail`/`trunc` は両 variant 共通必須。

## analyzer が再構成する情報

- `seq` ギャップ = UDP 損失（uint32 wrap-aware、reboot とは区別）
- `dt_h`/`ovr`/`stale`/`read_fail`/`trunc` の隣接 full-packet 間差分から
  全 200 Hz サイクルの dt 分布・overrun・IMU stale を再構成（20 Hz サンプリングでも
  漏れなく被覆できるのは、これらが ControlTask 側の累積 monotonic カウンタだから）
- `seq`/`t_us`/`loop` の巻き戻り検出 = デバイス再起動と判定し、差分計算を再アンカー
  （reboot 境界をまたいで delta を計算しない）
- `loop` 差分ゼロの連続区間 = ControlTask ストール
- `fsm`/`fault` 遷移、`sat`/`i2t` の連続区間を event として検出
- `summary.json` の `recommended_windows` に agent が最初に見るべき seq 区間を提示
  （reboot・stall・fault 遷移を loss/saturation より優先）

## テスト

```bash
python3 -m unittest discover -s tools/telemetry/tests
```

`test_e2e.py` はループバック UDP（ephemeral port）で
simulator → receiver → analyzer → report を通し、session dir 生成・
「first packet が diagnostic でも session 開始」・IP pinning 拒否集計・
reboot 再アンカーを検証する。

## 直接実行 / `-m` 実行の両対応について

各スクリプトは以下のどちらでも動く:

```bash
python3 tools/telemetry/bsl_telemetry/receiver.py --port 45678
# または (tools/telemetry を cwd にして)
cd tools/telemetry && python3 -m bsl_telemetry.receiver --port 45678
```

`receiver.py`/`simulator.py` は先頭で `__package__` を見て、直接実行時のみ
`tools/telemetry` を `sys.path` に挿入してから絶対 import する小さな shim を持つ
（パッケージ実行時は通常の相対 import を使う）。`schema.py`/`analyzer.py`/`report.py`
は他モジュールに依存しない自己完結型なので shim は不要。
