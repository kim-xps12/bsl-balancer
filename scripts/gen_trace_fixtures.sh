#!/usr/bin/env bash
# scripts/gen_trace_fixtures.sh — Wi-Fi ガード計装 (trace) の代表シナリオログを
# tools/telemetry/fixture_gen/gen_trace_log.cpp (native ハーネス、Arduino 非依存
# の src/core/ のみに依存) から再生成し、tools/telemetry/tests/fixtures/ へ
# 書き出す (wifi-guard-trace 計画書 §4 D11)。
#
# 命名規約: <scenario>__<EXPECTED>__<desc>.log
#   scenario ∈ {t1, t2, t3, t3b, t5, common}
#   EXPECTED ∈ {PASS, FAIL, INVALID}
#
# 出力はすべて固定シードの決定的シーケンス (壁時計・乱数不使用) のため、
# 再実行しても生成内容は完全に同一になる (`git diff --stat` がゼロ行である
# ことで鮮度を確認できる。計画書 §7 検証手順3)。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

SRC="tools/telemetry/fixture_gen/gen_trace_log.cpp"
OUT_DIR="tools/telemetry/tests/fixtures"
BIN="$(mktemp -d)/gen_trace_log"

CXX="${CXX:-g++}"
"$CXX" -std=gnu++17 -Wall -Wextra -O1 -o "$BIN" "$SRC"

mkdir -p "$OUT_DIR"

gen() {
  local scenario="$1"
  local filename="$2"
  echo "[gen_trace_fixtures] ${filename} (scenario=${scenario})"
  "$BIN" "$scenario" > "$OUT_DIR/$filename"
}

# ---- 正常系 PASS (計画書 §6 T1/T2/T3+T3b/T5、および共通妥当性のみを見る
# common) 各 1 ----
# t3/t3b: trace_check.py の R3 実装は「conn=1∧fsm=2∧ev=disc(r!=8)」に一致する
# 全エピソードを走査するため (相互検証で確認済み)、burnt 後の 2 度目の喪失
# (本来 one-shot 再発火せず abort 不要 = R3b が要求する静粛性) を含む単一ログを
# --scenario t3 に掛けると FAIL する (R3 と R3b が要求する挙動が両立しない)。
# よって R3 単体を試験する episode-1-only 版と、R3b を試験する 2 episode
# 通し版を別ファイルとして生成する (gen_trace_log.cpp のコメント参照)。
gen t1 "t1__PASS__ap_down_settled_before_balancing.log"
gen t2 "t2__PASS__connecting_immediate_abort_exactly_once.log"
gen t3 "t3__PASS__oneshot_disconnect_cancelled_episode1_only.log"
gen t3b "t3b__PASS__reconnect_then_burnt_disconnect_no_reactivity.log"
gen t5 "t5__PASS__arm_pending_hold_abort_once.log"
gen common "common__PASS__baseline_session_no_wifi_activity.log"

# ---- 計画書 §5 必須の 2 ケース ----
# 「abort が遷移 tick 内で即時発行される (op が st より先行する)」正しい挙動は
# PASS させる合成テスト (t2 の CONNECTING→Balancing 同一 tick 即時 abort が
# まさにこの代表例であるため、t2 の主 fixture をそのまま用いる。ゲート1第5回
# 指摘1 「tick アンカーと証拠順序」参照)。
gen t2_immediate_abort "t2__PASS__immediate_abort_op_precedes_st.log"
# 「tk 記録後・guard drain 前にイベント到着 + 同 tick 禁止窓内に start-class op」
# → FAIL (ゲート1第7回・第9回指摘の D11 相互検証 fixture)。実装が自然に発行
# しない不正な begin を意図的に注入した合成ログであり、trace_check がこれを
# 禁止窓内の op として FAIL 報告することを検証する入力。
gen t3_race_predrain_forbidden_op "t3__FAIL__predrain_event_forbidden_begin_in_window.log"

echo "[gen_trace_fixtures] done: $(ls -1 "$OUT_DIR" | wc -l | tr -d ' ') fixtures in $OUT_DIR"
