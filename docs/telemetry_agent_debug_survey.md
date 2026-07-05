# M5Stack Core2 UDP telemetry and coding-agent debug survey

調査日: 2026-06-29

## 結論

M5Stack Core2 から Wi-Fi/UDP で PC へテレメトリを送信し、PC 側のローカルサーバが実験ログとして蓄積し、その保存済みログを coding agent に解析させる仕組みは実現可能である。

coding agent はリアルタイムに遅延なく観測する必要はない。むしろ PC 側 receiver を信頼できるログ収集器として作り、実験が終わってから agent が session summary、event window、raw telemetry を必要な粒度で読む構成が扱いやすい。倒立制御そのものは M5Stack Core2 内で完結させ、agent は「保存済み実験ログの解析者」と「デバッグ支援者」として扱う。

推奨構成は次の通り。

```text
M5Stack Core2 firmware
  10 ms control loop
  telemetry queue / snapshot
  UDP telemetry task, 20-50 Hz
        |
        | UDP, one-way, JSONL or compact binary
        v
PC local receiver
  UDP socket
  schema validation
  append-only session log
  session metadata
  post-run summary and event index
        |
        | saved logs, MCP resources/tools, or codex exec stdin
        v
coding agent
  saved-session inspection
  failure analysis
  code-change proposal
  test/simulation command execution
```

## Current firmware constraints

このリポジトリは `platformio.ini` で `m5stack-core2`、Arduino framework、`M5Unified`、`Dynamixel2Arduino`、Kalman filter、PS4 controller host を使っている。

現在の制御上重要な点は以下である。

- `src/main.cpp` は `xPeriodMs = 10` として 10 ms 周期の制御を想定している。
- `calcPID()` は IMU 取得、Kalman 更新、PID 計算、DYNAMIXEL 速度指令までを同じ経路で実行している。
- `controlLoopTask` は core 0、priority 5 で `calcPID()` を呼び、`vTaskDelayUntil()` で周期実行している。
- `uiLoopTask` は core 1、priority 2 で UI と PS4 controller polling を扱っている。
- DYNAMIXEL の現在速度読み取りは `ENABLE_DEBUG_PRINT` の内側にあり、常時読む設計ではない。

したがって UDP 送信を `calcPID()` に直書きするのは避けるべきである。制御ループでは軽い snapshot だけを作り、UDP 送信は別タスクまたは低頻度の非同期経路に逃がすのが安全である。

## Web survey summary

### M5Stack Core2 / ESP32 side

M5Stack Core2 は ESP32 系 Core2 デバイスで、Wi-Fi/Bluetooth を前提にした Arduino/ESP32 開発が可能である。現リポジトリも PlatformIO の `espressif32` platform と Arduino framework を使っているため、追加依存を大きく増やさず `WiFi.h` と UDP API を利用できる。

Relevant sources:

- M5Stack Core2 product documentation: https://docs.m5stack.com/en/core/core2
- Arduino-ESP32 Wi-Fi API: https://docs.espressif.com/projects/arduino-esp32/en/latest/api/wifi.html
- Arduino-ESP32 UDP implementation: https://github.com/espressif/arduino-esp32/blob/master/libraries/Network/src/NetworkUdp.h
- ESP32 Wi-Fi/Bluetooth coexistence guide: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/coexist.html

UDP は軽量で、今回のような 20-50 Hz、数百 byte 程度の観測データ送信には十分である。一方で UDP は到達保証・順序保証がないため、packet に `seq` と `t_us` を必ず入れ、PC 側で欠落率と遅延を観測する必要がある。

PS4 controller host は Bluetooth を使うため、Wi-Fi と Bluetooth の 2.4 GHz 共存が実機リスクになる。PS4 操縦あり/なしで telemetry loss、制御周期 jitter、転倒頻度を分けて測るべきである。

### PC local receiver

PC 側は Python の UDP datagram endpoint で十分に受信できる。最初の実装は `asyncio` か `socketserver` で UDP を受け、append-only の session log を保存する小さなローカルプロセスでよい。

Relevant source:

- Python asyncio protocols and transports, datagram endpoint: https://docs.python.org/3/library/asyncio-protocol.html

receiver は通信路そのものより、時系列データの扱いやすさが重要である。最低限、次の機能を持たせる。

- UDP bind: `0.0.0.0:45678` など。
- source IP allowlist: 想定 Core2 の IP 以外を無視する。
- schema validation: version と必須 key を確認する。
- append-only session log: `logs/telemetry/{session_id}/raw.jsonl` のように保存する。
- session metadata: 実験条件、firmware version、PID gains、PS4 有無、operator notes を残す。
- post-run metrics: `seq` 欠落数、loop dt max、fall guard 発火、pitch error max、rpm saturation time を実験後に計算する。

### Coding-agent integration

coding agent へは live stream ではなく、保存済み session を渡す。方法は 2 段階で考えるとよい。

1. まずは `codex exec` などの非対話実行へ、session summary と抽出済み event window を標準入力で渡す。
2. 継続運用では MCP server を PC receiver または log archive に併設し、agent が必要なときに session 一覧、summary、event window、raw telemetry の限定範囲を tool/resource として読む。

OpenAI Codex は MCP server を CLI と IDE extension の両方で扱える。MCP には stdio server と streamable HTTP server の選択肢があり、ローカルの telemetry log archive には stdio server か `127.0.0.1` の HTTP server が合う。

Relevant sources:

- Model Context Protocol introduction: https://modelcontextprotocol.io/introduction
- MCP server development guide: https://modelcontextprotocol.io/docs/develop/build-server
- OpenAI Codex MCP documentation: https://developers.openai.com/codex/mcp
- OpenAI Codex non-interactive mode: https://developers.openai.com/codex/noninteractive

推奨 MCP interface:

```text
Resources:
  telemetry://sessions
  telemetry://session/{id}/summary
  telemetry://session/{id}/events
  telemetry://session/{id}/window/{start_seq}-{end_seq}

Tools:
  list_sessions(limit, tags)
  summarize_session(session_id)
  detect_instability(session_id)
  export_window(session_id, start_seq, end_seq, format)
  compare_sessions(session_a, session_b)
```

Codex の project-scoped config に置く場合の概形:

```toml
[mcp_servers.bsl_telemetry]
command = "uv"
args = ["run", "bsl-telemetry-mcp", "--log-root", "logs/telemetry"]
cwd = "/Users/b-sky-lab/Projects/bsl-fuzzy-balancer/firmware/bsl-balancer"
startup_timeout_sec = 10
tool_timeout_sec = 30
```

## Firmware design recommendation

### Packet fields

最初の packet は JSONL でよい。binary にするのは、JSON の生成コストや packet size が実測で問題になってからでよい。

Recommended fields:

```json
{
  "v": 1,
  "seq": 1234,
  "t_us": 987654321,
  "dt_us": 10021,
  "pitch_deg": 85.7,
  "pitch_dot_dps": -12.4,
  "target_deg": 86.0,
  "p": 0.3,
  "i": 0.08,
  "d": -1.1,
  "kp": 50.0,
  "ki": 1.0,
  "kd": 1.0,
  "rpm_cmd": 24,
  "rpm_l_cmd": -24,
  "rpm_r_cmd": 24,
  "turn_rpm": 0.0,
  "saturated": false,
  "fall_guard": false,
  "ps4_connected": true,
  "ps4_lx": 0,
  "battery_pct": 83,
  "rssi_dbm": -55
}
```

`rpm_l_cmd` / `rpm_r_cmd` は `driveTire()` 内で決まるため、telemetry で扱いやすいように最後の command 値を global snapshot に残すのがよい。DYNAMIXEL の present velocity は通信負荷が増えるため、初期版では毎周期読まず、必要時だけ 5-10 Hz で読む。

### Task split

制御ループの中では次の処理だけに抑える。

```text
calcPID()
  read IMU
  update Kalman/PID
  compute motor command
  driveTire()
  update telemetry snapshot
```

UDP task は別周期で snapshot を読み、`snprintf` で固定長 buffer に詰めて送る。

```text
telemetryTask()
  wait 20-50 ms
  copy latest snapshot
  if Wi-Fi connected:
    udp.beginPacket(pc_ip, pc_port)
    udp.write(buffer, length)
    udp.endPacket()
```

`String` や heap allocation を制御経路で使わない。Wi-Fi 再接続も制御タスクから分離する。

### Configuration

初期実験では `build_flags` や `secrets.h` で SSID、password、PC IP、port を与えるのが簡単である。ただし公開リポジトリに秘密情報を入れない。運用段階では `Preferences` か serial provisioning を検討する。

## Offline-first PC server design

PC 側の中心は「リアルタイム可視化サーバ」ではなく「実験 session logger」である。receiver は UDP datagram を受けたら即座に append-only で保存し、重い解析は受信経路から分離する。

### Responsibilities

```text
UdpTelemetryReceiver
  bind UDP socket
  accept datagrams from allowlisted Core2 IPs
  decode JSON packet
  attach host_receive_ns
  validate schema version
  append raw JSONL record
  update in-memory counters only

SessionManager
  create session id
  write metadata.json
  rotate raw.jsonl if size grows
  close session cleanly on SIGINT / command
  write summary.json on close

PostRunAnalyzer
  read raw.jsonl
  compute loss and timing metrics
  detect fall/oscillation/saturation windows
  write events.jsonl
  write derived_summary.json

AgentInterface
  provide compact session summary
  provide bounded raw windows
  compare sessions
```

受信時にやる処理は「parse、時刻付与、validation、保存」までに抑える。fall detection やグラフ生成、agent 用要約は後段でよい。この分離により、PC 側の解析処理が詰まっても UDP 受信とログ保存の信頼性を落としにくい。

### Session directory layout

最小構成は JSONL と JSON metadata で十分である。SQLite や NPZ は必要になってから追加する。

```text
logs/telemetry/
  20260629-213015_core2-a4cf_run-001/
    metadata.json
    raw.jsonl
    events.jsonl
    summary.json
    notes.md
```

`metadata.json`:

```json
{
  "schema": "bsl-telemetry-session-v1",
  "session_id": "20260629-213015_core2-a4cf_run-001",
  "started_at": "2026-06-29T21:30:15+09:00",
  "ended_at": "2026-06-29T21:31:45+09:00",
  "robot": "bsl-balancer-core2",
  "device_id": "core2-a4cf",
  "firmware_git": "unknown",
  "firmware_build": "unknown",
  "telemetry_hz": 20,
  "udp_port": 45678,
  "operator": "local",
  "tags": ["pid-baseline", "ps4-off"],
  "notes": "Kp=50 Ki=1 Kd=1 pitch_target=86"
}
```

`raw.jsonl` は firmware packet に PC 受信情報を足して保存する。

```jsonl
{"host_receive_ns":1782736215123456789,"source_ip":"192.168.10.42","packet":{"v":1,"seq":1234,"t_us":987654321,"dt_us":10021,"pitch_deg":85.7,"rpm_cmd":24,"fall_guard":false}}
{"host_receive_ns":1782736215173490123,"source_ip":"192.168.10.42","packet":{"v":1,"seq":1235,"t_us":987704328,"dt_us":10007,"pitch_deg":85.9,"rpm_cmd":25,"fall_guard":false}}
```

`events.jsonl` は post-run analyzer が作る。

```jsonl
{"type":"loss","severity":"warn","start_seq":1240,"end_seq":1243,"lost_packets":3}
{"type":"saturation","severity":"info","start_seq":1501,"end_seq":1542,"duration_ms":2050,"field":"rpm_cmd"}
{"type":"fall_guard","severity":"error","start_seq":1888,"end_seq":1892,"duration_ms":200}
```

`summary.json` は agent に最初に渡す主資料である。

```json
{
  "session_id": "20260629-213015_core2-a4cf_run-001",
  "duration_s": 90.2,
  "packet_count": 1804,
  "lost_packets": 4,
  "loss_rate": 0.0022,
  "dt_us": {"mean": 10004, "p95": 10080, "max": 13200},
  "pitch_error_deg": {"rms": 1.8, "max_abs": 12.4},
  "rpm_cmd": {"max_abs": 300, "saturation_ratio": 0.08},
  "fall_guard_count": 1,
  "event_counts": {"loss": 1, "saturation": 1, "fall_guard": 1},
  "recommended_windows": [
    {"label": "fall_guard", "start_seq": 1840, "end_seq": 1900},
    {"label": "largest_pitch_error", "start_seq": 1720, "end_seq": 1780}
  ]
}
```

### Session control

PC server は自動 session と手動 session の両方を扱えるとよい。

- 自動開始: 最初の valid packet を受けたら session を作る。
- 自動終了: 一定秒数 packet が来なければ session を close する。
- 手動 marker: keyboard、HTTP、または CLI で `mark_event("released hand")` のような operator event を打てる。
- 手動 close: 実験終了時に `Ctrl-C` しても `summary.json` まで書いて終了する。

最初の CLI 例:

```bash
uv run bsl-telemetry-receiver --udp-port 45678 --log-root logs/telemetry --device core2-a4cf
uv run bsl-telemetry-analyze logs/telemetry/20260629-213015_core2-a4cf_run-001
uv run bsl-telemetry-report logs/telemetry/20260629-213015_core2-a4cf_run-001
```

### Agent handoff

agent には raw log 全体ではなく、次の順で渡す。

1. `metadata.json`
2. `summary.json`
3. `events.jsonl`
4. `recommended_windows` の raw packet 抜粋
5. agent が追加で要求した bounded window

`codex exec` へ渡す場合の概形:

```bash
uv run bsl-telemetry-report logs/telemetry/20260629-213015_core2-a4cf_run-001 \
  | codex exec "この実機ログから転倒直前の主因候補を3つに絞り、該当する firmware の確認箇所と次の実験条件を提案して"
```

MCP 化する場合も同じ思想で、tool の default response は summary にする。raw telemetry は `export_window()` のように範囲指定された場合だけ返す。

## PC / MCP design recommendation

receiver と MCP server は同じ Python process でもよいが、責務は分ける。

```text
UdpTelemetryReceiver
  parse datagrams
  validate schema
  append raw session log
  update lightweight counters

TelemetryAnalyzer
  compute loss rate
  compute dt statistics
  detect fall/oscillation/saturation

McpTelemetryServer
  expose resources/tools
  return compact summaries by default
  export raw windows only on request
```

agent に毎 packet を読ませる必要はない。通常は `summarize_session(session_id)` や `detect_instability(session_id)` のような tool で要約し、必要な区間だけ raw JSONL を出す方が token と時間の無駄が少ない。

## Verification plan

1. PC 上の simulator から UDP packet を投げ、receiver の schema validation、loss detection、JSONL 保存を確認する。
2. Core2 を Wi-Fi 接続だけさせ、motor torque off の状態で 20 Hz telemetry を送る。
3. 制御有効、telemetry 20 Hz で `dt_us` の max と packet loss を測る。
4. telemetry 50 Hz に上げて、制御周期 jitter と転倒しやすさに変化があるか確認する。
5. PS4 controller 接続あり/なしで Wi-Fi loss と制御挙動を比較する。
6. 実験終了後に `summary.json` と `events.jsonl` が生成されることを確認する。
7. MCP server から `telemetry://sessions`、`telemetry://session/{id}/summary`、`detect_instability(session_id)` が読めることを確認する。
8. Codex に「この session で転倒直前の要因を要約し、該当する firmware の修正候補を出す」と依頼し、保存済み data に基づく解析ができることを確認する。

Acceptance criteria:

- control loop の `dt_us` が通常 10 ms 近傍に収まり、telemetry 有効化で大きく悪化しない。
- 20 Hz telemetry で packet loss が実験環境内で十分低い。目安として通常運用 5% 未満。
- PS4 接続時にも制御の明確な劣化がない。劣化する場合は telemetry 周期を下げるか、Wi-Fi channel/AP 配置を見直す。
- receiver が落ちても Core2 の制御は継続する。
- agent は保存済み telemetry を読めるが、motor command など危険な実機操作は直接送れない。
- 実験 session を閉じた後、raw log だけから summary と event index を再生成できる。

## Risks and mitigations

| Risk | Impact | Mitigation |
| --- | --- | --- |
| UDP loss / reordering | agent analysis が欠落区間を誤解する | `seq`, `t_us`, receiver-side loss metrics を必須化する |
| Wi-Fi/Bluetooth coexistence | PS4 操縦時に telemetry loss や jitter が増える | PS4 あり/なしで比較し、telemetry 周期を下げる |
| control-loop perturbation | 倒立安定性が落ちる | UDP 送信を別タスク化し、制御ループでは snapshot のみ更新する |
| DYNAMIXEL read overhead | servo bus 待ちで制御が遅れる | present velocity は低頻度、または初期版では送らない |
| secret leakage | Wi-Fi password が repo に混入する | `secrets.h` を gitignore、または Preferences provisioning |
| unsafe command channel | agent が実機に危険な command を送る | 初期版は one-way telemetry のみ。command channel は別設計で manual approval 必須 |
| token bloat | raw telemetry が agent context を圧迫する | MCP tool は summary を default にし、raw window は明示要求時だけ返す |
| corrupt/incomplete session | PC 側停止時にログが中途半端になる | append-only raw log を主データにし、summary/events は再生成可能にする |

## Implementation phases

Phase 1: survey-to-prototype

- `src/main.cpp` に telemetry snapshot 構造体と last motor command を追加する。
- `WiFi.h` / `WiFiUdp.h` を追加し、20 Hz の UDP telemetry task を作る。
- PC 側に Python UDP receiver を追加し、session directory、`metadata.json`、`raw.jsonl` 保存を実装する。
- simulator sender を用意して receiver を単体確認する。
- post-run analyzer を追加し、`summary.json` と `events.jsonl` を raw log から生成する。

Phase 2: agent integration

- log archive を読む MCP server を追加する。
- `telemetry://sessions`、`telemetry://session/{id}/summary`、`detect_instability(session_id)` を実装する。
- project-scoped `.codex/config.toml` のサンプルを追加する。
- Codex から MCP tool を呼び、保存済み session に基づく解析ができるか確認する。

Phase 3: closed debug workflow

- 実機 session ID、firmware git commit、tuning parameters を telemetry log に入れる。
- 転倒・飽和・大振動などの event detector を追加する。
- agent が「ログを読む -> 仮説を出す -> firmware/simulation 修正案を作る -> test command を走らせる」流れを文書化する。

## Recommendation

まず作るべき最小構成は「one-way UDP telemetry + PC session logger + post-run analyzer + `codex exec` によるログ要約」である。これだけで実機挙動を coding agent に渡す価値が出る。

その次に MCP server を追加し、Codex が保存済み session の summary、event、必要な telemetry window を自分で取得できるようにする。MCP 化は有用だが、最初から MCP を含めて firmware と receiver を同時に作ると切り分けが難しくなる。まずは raw log を確実に残し、summary/events を後から再生成できる状態を作るのが安全である。
