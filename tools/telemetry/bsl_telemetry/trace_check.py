"""Machine judge for Wi-Fi guard trace captures (grammar v1, tag `[WG1]`).

See docs/plans/2026-07-06-wifi-guard-trace-bench.md section 4 (D3, grammar)
and section 5 (checker judgement rules R1-R5 + common validity) for the full
specification this module implements. Every function below cites the (a)/(b)/
... sub-clause of section 5 it encodes, so a reviewer can match code to spec
line-by-line.

Pure, deterministic, stdlib-only (Python 3.11+): given the same log text this
always produces the same verdicts -- no wall-clock/network dependency.

CLI usage:
    python3 tools/telemetry/bsl_telemetry/trace_check.py --scenario t1 <log>
    python3 tools/telemetry/bsl_telemetry/trace_check.py --scenario t3b <log>

`--scenario` selects exactly one of R1 (t1) / R2 (t2) / R3 (t3) / R3b (t3b) /
R5 (t5); common validity and R4 always run regardless of `--scenario` (section
5 intro: "R4 と妥当性検査は全シナリオで常時実行"). Exit code 0 iff every
executed check is PASS or WARN (WARN never blocks; FAIL/INVALID always do).
"""

from __future__ import annotations

import argparse
import bisect
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

WG1_TAG = "[WG1]"

_BARE_KINDS = {"boot", "tk", "st", "hb", "drop"}
EV_TYPES = {"gotip", "disc", "stop", "start", "conn", "scan"}
OP_TYPES = {"begin", "disc", "radio_off", "bp"}
# ev types that represent library-origin Wi-Fi activity the quiet-window
# rules must see zero of (section 5 "ev ストリームの静粛検査"). `disc` (any
# reason) is handled separately since its rules differ by context.
LIBRARY_ACTIVITY_EV_TYPES = {"start", "conn", "scan", "gotip"}

# Tunable constants named after the spec's own vocabulary so a reviewer can
# grep the plan text and find the matching constant.
FLMAX_BUDGET_US = 40_000  # section 5 common validity: flmax <= 40000us
STKMIN_MIN_BYTES = 1024  # section 5 common validity: stkmin >= 1024
DISC_DUR_BUDGET_US = 100_000  # kDiscDurBudgetUs, section 5 "disconnect 所要時間の予算検査"
ANCHOR_TICK_BUDGET = 2  # "初回 disc は遷移から <= 2 tick"
ABORT_MAX_RETRIES = 4  # abort_max_retries, section 5 "有界リトライの扱い"
R3B_OBSERVATION_WINDOW_US = 20_000_000  # section 5 R3b "20 秒以上"
_DEFAULT_TICK_PERIOD_US = 50_000  # 20 Hz nominal telemetry tick (section 3)

RULE_BY_SCENARIO = {"t1": "R1", "t2": "R2", "t3": "R3", "t3b": "R3b", "t5": "R5"}


class ParseError(Exception):
    """Raised internally while converting one [WG1] line's fields."""


def _to_int(v: str) -> int:
    if not re.fullmatch(r"-?\d+", v):
        raise ParseError(f"not an integer: {v!r}")
    return int(v)


def _to_opt_int(v: str) -> Optional[int]:
    return None if v == "-" else _to_int(v)


def _to_bit(v: str) -> int:
    if v not in ("0", "1"):
        raise ParseError(f"not 0/1: {v!r}")
    return int(v)


def _to_opt_bit(v: str) -> Optional[int]:
    return None if v == "-" else _to_bit(v)


def _to_str(v: str) -> str:
    return v


def _enum_conv(allowed: Sequence[str]):
    def _conv(v: str) -> str:
        if v not in allowed:
            raise ParseError(f"value {v!r} not one of {sorted(allowed)}")
        return v

    return _conv


_BOOT_FIELDS = {"v": _to_int, "fw": _to_str, "dev": _to_str}
_TK_FIELDS = {"fsm": _to_opt_int, "arm": _to_opt_bit, "ep": _to_int}
_ST_FIELDS = {
    "fsm": _to_opt_int,
    "arm": _to_opt_bit,
    "q": _to_bit,
    "conn": _to_bit,
    "cing": _to_bit,
    "udpr": _to_bit,
    "librp": _to_bit,
    "ab": _to_bit,
    "roff": _to_bit,
    "latch": _to_bit,
    "pwf": _to_int,
    "sf": _to_int,
    "evo": _to_int,
}
_HB_EXTRA_FIELDS = {
    "heap": _to_int,
    "heapmin": _to_int,
    "evdrop": _to_int,
    "flmax": _to_int,
    "stkmin": _to_int,
    "tick": _to_int,
}
_HB_FIELDS = {**_ST_FIELDS, **_HB_EXTRA_FIELDS}
_EV_FIELDS = {"ev": _enum_conv(EV_TYPES), "r": _to_opt_int, "i": _to_opt_int}
_OP_BASE_FIELDS = {
    "op": _enum_conv(OP_TYPES),
    "fsm": _to_opt_int,
    "arm": _to_opt_bit,
    "q": _to_bit,
    "udpr": _to_bit,
    "res": _to_opt_int,
    "dur": _to_int,
}
_OP_BP_EXTRA_FIELDS = {
    "fsm2": _to_opt_int,
    "arm2": _to_opt_bit,
    "fsm3": _to_opt_int,
    "arm3": _to_opt_bit,
}
_DROP_FIELDS = {"n": _to_int}

CONTEXT_KEYS = ("fsm", "arm", "q", "conn", "cing", "udpr", "librp", "ab", "roff", "latch")


@dataclass
class Record:
    line_no: int
    raw: str
    s: int
    t: int
    kind: str  # boot / ev / op / tk / st / hb / drop
    f: Dict[str, Any]

    def cite(self) -> str:
        return f"L{self.line_no}: {self.raw}"


def _convert_fields(kind: str, raw_fields: Dict[str, str]) -> Dict[str, Any]:
    if kind == "boot":
        spec = _BOOT_FIELDS
    elif kind == "tk":
        spec = _TK_FIELDS
    elif kind == "st":
        spec = _ST_FIELDS
    elif kind == "hb":
        spec = _HB_FIELDS
    elif kind == "drop":
        spec = _DROP_FIELDS
    elif kind == "ev":
        spec = _EV_FIELDS
    elif kind == "op":
        op_type = raw_fields.get("op")
        if op_type not in OP_TYPES:
            raise ParseError(f"unknown op type: {op_type!r}")
        spec = dict(_OP_BASE_FIELDS)
        if op_type == "bp":
            spec.update(_OP_BP_EXTRA_FIELDS)
    else:  # pragma: no cover - defensive, kind is always one we set below
        raise ParseError(f"unknown record kind {kind!r}")

    out: Dict[str, Any] = {}
    for key, conv in spec.items():
        if key not in raw_fields:
            raise ParseError(f"{kind} record missing required field {key!r}")
        out[key] = conv(raw_fields[key])
    return out


def parse_line(stripped: str, line_no: int) -> Tuple[Optional[Record], Optional[str]]:
    """Parse one already-`[WG1]`-prefixed, whitespace-stripped line.

    Returns (Record, None) on success, or (None, reason) if the line starts
    with the tag but is not grammar-conformant (section 4 D3 / section 5
    "文法不一致の行（破損）を INVALID とする").
    """
    tokens = stripped.split()
    if len(tokens) < 4:
        return None, "too few fields"
    if not tokens[1].startswith("s="):
        return None, "missing s= field"
    if not tokens[2].startswith("t="):
        return None, "missing t= field"
    try:
        s_val = _to_int(tokens[1][2:])
        t_val = _to_int(tokens[2][2:])
    except ParseError as exc:
        return None, str(exc)

    disc = tokens[3]
    if disc in _BARE_KINDS:
        kind = disc
        kv_tokens = tokens[4:]
    elif disc.startswith("ev="):
        kind = "ev"
        kv_tokens = tokens[3:]
    elif disc.startswith("op="):
        kind = "op"
        kv_tokens = tokens[3:]
    else:
        return None, f"unrecognized record discriminator {disc!r}"

    raw_fields: Dict[str, str] = {}
    for tok in kv_tokens:
        if "=" not in tok:
            return None, f"malformed field token {tok!r}"
        k, v = tok.split("=", 1)
        if not k:
            return None, f"malformed field token {tok!r}"
        raw_fields[k] = v

    try:
        fields = _convert_fields(kind, raw_fields)
    except ParseError as exc:
        return None, str(exc)

    return Record(line_no=line_no, raw=stripped, s=s_val, t=t_val, kind=kind, f=fields), None


def load_wg1_lines(text: str) -> Tuple[List[Record], List[Tuple[int, str, str]]]:
    """Scan raw log text for `[WG1]` lines.

    Non-`[WG1]` lines (e.g. pre-baud-switch boot garbage, section 4 D3) are
    ignored outright. Returns (records, malformed) in physical file order;
    `malformed` holds (line_no, raw_line, reason) for every `[WG1]`-tagged
    line that fails grammar validation.
    """
    records: List[Record] = []
    malformed: List[Tuple[int, str, str]] = []
    for line_no, raw in enumerate(text.splitlines(), start=1):
        stripped = raw.strip()
        if not stripped.startswith(WG1_TAG):
            continue
        record, err = parse_line(stripped, line_no)
        if record is None:
            malformed.append((line_no, stripped, err or "unknown parse error"))
        else:
            records.append(record)
    return records, malformed


def check_boot_structure(records: Sequence[Record], malformed: Sequence[Tuple[int, str, str]]) -> Optional[str]:
    """Section 5 preprocessing (0): boot-first-and-only, physical file order.

    Returns None if OK, else an INVALID reason string.
    """
    combined: List[Tuple[int, str, Optional[Record]]] = [(r.line_no, "record", r) for r in records]
    combined += [(ln, "malformed", None) for ln, _raw, _reason in malformed]
    combined.sort(key=lambda item: item[0])
    if not combined:
        return "no [WG1] lines found in log"
    first_line_no, first_kind, first_record = combined[0]
    if first_kind != "record" or first_record.kind != "boot":
        return f"first [WG1] line (L{first_line_no}) is not a boot line"
    boot_line_nos = [ln for ln, kind, rec in combined if kind == "record" and rec.kind == "boot"]
    if len(boot_line_nos) > 1:
        return f"multiple boot lines found: lines {boot_line_nos}"
    return None


def check_s_continuity(records: Sequence[Record], malformed: Sequence[Tuple[int, str, str]]) -> Optional[str]:
    """Section 5 preprocessing (1): `s` continuity + grammar conformance.

    Returns None if OK, else an INVALID reason string. Any malformed line at
    all is itself a "破損" -> INVALID (checked before the continuity scan).
    """
    if malformed:
        line_no, raw, reason = malformed[0]
        return f"malformed [WG1] line at L{line_no} ({reason}): {raw!r}"
    if not records:
        return "no [WG1] records found"
    seen: set = set()
    prev: Optional[int] = None
    for r in records:
        if r.s in seen:
            return f"duplicate s={r.s} (L{r.line_no})"
        seen.add(r.s)
        if prev is not None and r.s != prev + 1:
            return f"gap in s sequence: {prev} -> {r.s} (L{r.line_no})"
        prev = r.s
    return None


def build_context_timeline(sorted_records: Sequence[Record]) -> List[Dict[str, Optional[int]]]:
    """For each position in `sorted_records`, the most recent st/hb-derived
    guard context (fsm/arm/q/conn/cing/udpr/librp/ab/roff/latch), including
    this record's own update if it is itself an st/hb row.
    """
    ctx: Dict[str, Optional[int]] = {k: None for k in CONTEXT_KEYS}
    timeline: List[Dict[str, Optional[int]]] = []
    for r in sorted_records:
        if r.kind in ("st", "hb"):
            for k in CONTEXT_KEYS:
                ctx[k] = r.f.get(k)
        timeline.append(dict(ctx))
    return timeline


Tick = Tuple[int, Record]  # (position in sorted_records, the tk Record)


def build_ticks(sorted_records: Sequence[Record]) -> List[Tick]:
    return [(pos, r) for pos, r in enumerate(sorted_records) if r.kind == "tk"]


def _nominal_tick_period_us(ticks: Sequence[Tick]) -> int:
    if len(ticks) < 2:
        return _DEFAULT_TICK_PERIOD_US
    diffs = [b[1].t - a[1].t for a, b in zip(ticks, ticks[1:]) if b[1].t > a[1].t]
    if not diffs:
        return _DEFAULT_TICK_PERIOD_US
    diffs.sort()
    return diffs[len(diffs) // 2]


def owning_tick_idx(ticks: Sequence[Tick], tk_ts: Sequence[int], t: int) -> Optional[int]:
    """Index into `ticks` of the last tick whose tk.t <= t (the tick that was
    "current" when a record at time `t` was produced), or None if `t`
    precedes the first tick.
    """
    idx = bisect.bisect_right(tk_ts, t) - 1
    return idx if idx >= 0 else None


def find_drain_tick(ticks: Sequence[Tick], event_i: int) -> Optional[int]:
    """Section 5 "イベント起点窓の drain アンカー": the tick T (index into
    `ticks`) such that ticks[T].ep < event_i <= ticks[T+1].ep, i.e. the tick
    during which guard drained event `event_i`. Returns None if no such T
    exists (either malformed epoch bookkeeping, or -- the expected case at
    log truncation -- no tick T+1 was ever recorded, i.e. drain attribution
    is impossible for a tail event; section 5: "当該エピソードを INVALID に
    する").
    """
    eps = [r.f["ep"] for _pos, r in ticks]
    j = bisect.bisect_left(eps, event_i)
    if j == 0 or j >= len(ticks):
        return None
    return j - 1


@dataclass
class Outcome:
    name: str
    status: str  # PASS / FAIL / INVALID / WARN (WARN only ever used as a note bucket on R4)
    evidence: List[str] = field(default_factory=list)


@dataclass
class EpisodeEval:
    status: str
    evidence: List[str]
    window_end_t: Optional[int] = None
    confirm_t: Optional[int] = None


def _evaluate_abort_episode(
    sorted_records: Sequence[Record],
    ticks: Sequence[Tick],
    tk_ts: Sequence[int],
    anchor_tick_idx: int,
    rule_label: str,
) -> EpisodeEval:
    """Shared abort-episode judge used by R2/R3/R5 (section 5: "この要求は
    R2/R3/R5 の全ヒットエピソードに共通で適用する"). `anchor_tick_idx` is the
    index into `ticks` of the tk row anchoring the forbidden window
    (section 5 "tick アンカーと証拠順序").

    All "before/after" comparisons below use *position* in `sorted_records`
    (which is t-sorted with original file/`s` order as a stable tiebreak)
    rather than raw `t` values, so two records legitimately sharing the same
    microsecond (e.g. an op recorded in the same tick as its confirming ev)
    are still ordered correctly instead of colliding on a `<=`/`>` compare.
    """
    anchor_pos = ticks[anchor_tick_idx][0]
    anchor_t = ticks[anchor_tick_idx][1].t
    tick_period = _nominal_tick_period_us(ticks)

    # Forbidden window end: first later st/hb context showing q=0 ∧ fsm≠2 ∧
    # arm=0 (section 5 "abort エピソードの追跡範囲": not cut off at q=0 alone,
    # tracked through completion -- this is merely the *upper bound* used to
    # scope which op/ev rows are inspected; completion is verified below).
    window_end_pos = len(sorted_records) - 1
    for pos in range(anchor_pos + 1, len(sorted_records)):
        r = sorted_records[pos]
        if r.kind not in ("st", "hb"):
            continue
        if r.f.get("q") == 0 and r.f.get("fsm") != 2 and r.f.get("arm") == 0:
            window_end_pos = pos
            break

    op_rows = [(pos, r) for pos, r in enumerate(sorted_records) if anchor_pos <= pos <= window_end_pos and r.kind == "op"]
    ev_rows = [(pos, r) for pos, r in enumerate(sorted_records) if anchor_pos <= pos <= window_end_pos and r.kind == "ev"]

    forbidden = [(pos, r) for pos, r in op_rows if r.f["op"] in ("begin", "bp")]
    if forbidden:
        r0 = forbidden[0][1]
        return EpisodeEval(
            "FAIL",
            [f"{rule_label}: forbidden op={r0.f['op']} inside abort window ({r0.cite()})"],
        )

    disc_rows = [(pos, r) for pos, r in op_rows if r.f["op"] == "disc"]
    radio_off_rows = [(pos, r) for pos, r in op_rows if r.f["op"] == "radio_off"]

    if not disc_rows:
        return EpisodeEval(
            "FAIL",
            [f"{rule_label}: no abort op=disc issued in forbidden window (anchor {ticks[anchor_tick_idx][1].cite()})"],
        )

    first_disc_pos, first_disc = disc_rows[0]
    first_disc_tick = owning_tick_idx(ticks, tk_ts, first_disc.t)
    ticks_elapsed = None if first_disc_tick is None else first_disc_tick - anchor_tick_idx
    if ticks_elapsed is None or ticks_elapsed > ANCHOR_TICK_BUDGET or ticks_elapsed < 0:
        return EpisodeEval(
            "FAIL",
            [
                f"{rule_label}: first abort disc issued {ticks_elapsed} tick(s) after anchor "
                f"(budget {ANCHOR_TICK_BUDGET}) ({first_disc.cite()})"
            ],
        )

    # (a) every individual disc call must stay within its own dur budget.
    for _pos, d in disc_rows:
        if d.f["dur"] > DISC_DUR_BUDGET_US:
            return EpisodeEval(
                "FAIL",
                [f"{rule_label}: disc dur={d.f['dur']}us exceeds budget {DISC_DUR_BUDGET_US}us ({d.cite()})"],
            )
    # (b) the anchor-relative completion deadline applies to the *first*
    # disc only -- bounded retries (below) are issued one per subsequent
    # tick and are governed by their own q=1/count<=4 criteria instead, not
    # by the original transition's deadline (section 5 "有界リトライの扱い"
    # introduces no additional anchor-relative deadline for retries).
    deadline = anchor_t + ANCHOR_TICK_BUDGET * tick_period + DISC_DUR_BUDGET_US
    if first_disc.t + first_disc.f["dur"] > deadline:
        return EpisodeEval(
            "FAIL",
            [f"{rule_label}: disc completion deadline exceeded ({first_disc.cite()})"],
        )

    notes: List[str] = []
    retries = disc_rows[1:]
    if retries:
        if len(disc_rows) > 1 + ABORT_MAX_RETRIES:
            return EpisodeEval(
                "FAIL",
                [f"{rule_label}: retry count {len(retries)} exceeds bound {ABORT_MAX_RETRIES} (last: {retries[-1][1].cite()})"],
            )
        for _pos, rd in retries:
            if rd.f["q"] != 1:
                return EpisodeEval(
                    "FAIL",
                    [f"{rule_label}: retry disc issued while q=0 (WIFI_QUIET not continued) ({rd.cite()})"],
                )
        notes.append(f"{rule_label}: {len(retries)} bounded retry(ies) accepted, all issued with q=1")

    if radio_off_rows:
        r0 = radio_off_rows[0][1]
        return EpisodeEval(
            "FAIL",
            [
                f"{rule_label}: radio_off reached inside abort episode "
                f"(bounded cancellation not completed -- terminal fail-closed path) ({r0.cite()})"
            ],
        )

    last_disc_pos, last_disc = disc_rows[-1]
    last_disc_tick = owning_tick_idx(ticks, tk_ts, last_disc.t)
    confirm_pos: Optional[int] = None
    confirm: Optional[Record] = None
    for pos, r in ev_rows:
        if pos <= last_disc_pos:
            continue
        if not ((r.f["ev"] == "disc" and r.f["r"] == 8) or r.f["ev"] == "stop"):
            continue
        if r.f["i"] is None:
            continue
        r_tick = find_drain_tick(ticks, r.f["i"])
        if r_tick is None or last_disc_tick is None or r_tick <= last_disc_tick:
            continue
        confirm_pos, confirm = pos, r
        break
    if confirm is None:
        return EpisodeEval(
            "FAIL",
            [f"{rule_label}: no post-abort completion event (ev=disc r=8 or ev=stop) observed after {last_disc.cite()}"],
        )

    for pos, r in ev_rows:
        if pos <= confirm_pos:
            continue
        if r.f["ev"] in LIBRARY_ACTIVITY_EV_TYPES or r.f["ev"] == "disc":
            return EpisodeEval(
                "FAIL",
                [f"{rule_label}: library activity ev={r.f['ev']} observed after abort completion confirmation ({r.cite()})"],
            )

    final_ctx: Optional[Record] = None
    for pos in range(window_end_pos, len(sorted_records)):
        r = sorted_records[pos]
        if r.kind in ("st", "hb"):
            final_ctx = r
            break
    if final_ctx is not None:
        f = final_ctx.f
        if f.get("ab") != 0 or f.get("librp") != 0 or f.get("roff") != 0 or f.get("latch") != 0:
            return EpisodeEval(
                "FAIL",
                [f"{rule_label}: final state not healthy (ab/librp/roff/latch not all cleared) ({final_ctx.cite()})"],
            )

    notes.append(f"{rule_label}: abort episode completed cleanly, confirmed by {confirm.cite()}")
    return EpisodeEval("PASS", notes, window_end_t=sorted_records[window_end_pos].t, confirm_t=confirm.t)


def _find_q_rising_transitions(
    sorted_records: Sequence[Record],
    context_timeline: Sequence[Dict[str, Optional[int]]],
    require_cing: Optional[int],
) -> List[int]:
    """Positions (into sorted_records) of every q: 0 -> 1 rising edge seen on
    an op/st/hb row, optionally gated on the `cing` context effective at that
    position (section 5 R2 trigger: "cing=1 の状態で q が 0->1 に遷移").
    """
    positions: List[int] = []
    last_q: Optional[int] = None
    for pos, r in enumerate(sorted_records):
        if r.kind not in ("op", "st", "hb"):
            continue
        q = r.f.get("q")
        if q is None:
            continue
        if last_q == 0 and q == 1:
            if require_cing is None or context_timeline[pos].get("cing") == require_cing:
                positions.append(pos)
        last_q = q
    return positions


@dataclass
class TriggerHit:
    ev: Record
    drain_tick_idx: int


def _find_disc_reconnect_triggers(
    sorted_records: Sequence[Record],
    ticks: Sequence[Tick],
    context_timeline: Sequence[Dict[str, Optional[int]]],
    mode: str,
    after_t: Optional[int] = None,
) -> Tuple[List[TriggerHit], List[Record]]:
    """`ev=disc r!=8` events qualifying as a one-shot reconnect trigger for
    R3 (mode="R3": `conn=1 ∧ fsm=2` at the drain tick) or R5 (mode="R5":
    `arm=1 ∨ fsm=2`). Returns (hits, attribution_failures): a qualifying
    event whose drain tick cannot be determined (section 5 "末尾帰属不能")
    is reported separately so callers can surface INVALID.
    """
    hits: List[TriggerHit] = []
    failures: List[Record] = []
    for pos, r in enumerate(sorted_records):
        if r.kind != "ev" or r.f["ev"] != "disc" or r.f["r"] == 8:
            continue
        if after_t is not None and r.t <= after_t:
            continue
        if r.f["i"] is None:
            continue
        drain_idx = find_drain_tick(ticks, r.f["i"])
        if drain_idx is None:
            failures.append(r)
            continue
        tick_pos, tick_rec = ticks[drain_idx]
        fsm_ctx = tick_rec.f.get("fsm")
        arm_ctx = tick_rec.f.get("arm")
        if mode == "R3":
            conn_ctx = context_timeline[tick_pos].get("conn")
            qualifies = fsm_ctx == 2 and conn_ctx == 1
        elif mode == "R5":
            qualifies = fsm_ctx == 2 or arm_ctx == 1
        else:  # pragma: no cover - internal misuse only
            raise ValueError(f"unknown mode {mode!r}")
        if qualifies:
            hits.append(TriggerHit(ev=r, drain_tick_idx=drain_idx))
    return hits, failures


def _evaluate_r1(sorted_records: Sequence[Record]) -> Outcome:
    """R1 (T1, C1): op+ev stream quiet during every clean-entry Balancing window."""
    windows: List[Tuple[int, int]] = []
    in_window = False
    start_pos = 0
    for pos, r in enumerate(sorted_records):
        if r.kind not in ("st", "hb"):
            continue
        fsm = r.f.get("fsm")
        if not in_window and fsm == 2:
            in_window = True
            start_pos = pos
        elif in_window and fsm != 2:
            windows.append((start_pos, pos))
            in_window = False
    if in_window:
        windows.append((start_pos, len(sorted_records)))

    eligible = []
    for start, end in windows:
        f = sorted_records[start].f
        if f.get("cing") == 0 and f.get("librp") == 0 and f.get("ab") == 0:
            eligible.append((start, end))

    if not eligible:
        return Outcome("R1", "FAIL", ["R1: no eligible Balancing window (cing=0∧librp=0∧ab=0 at entry) found"])

    for start, end in eligible:
        for pos in range(start, end):
            r = sorted_records[pos]
            if r.kind == "op":
                if r.f["op"] in ("begin", "disc", "radio_off"):
                    return Outcome("R1", "FAIL", [f"R1: forbidden op={r.f['op']} inside quiet window ({r.cite()})"])
                if r.f["op"] == "bp" and r.f["udpr"] != 1:
                    return Outcome("R1", "FAIL", [f"R1: bp with udpr=0 inside quiet window ({r.cite()})"])
            elif r.kind == "ev":
                if r.f["ev"] in LIBRARY_ACTIVITY_EV_TYPES or r.f["ev"] == "disc":
                    return Outcome("R1", "FAIL", [f"R1: library-origin ev={r.f['ev']} inside quiet window ({r.cite()})"])

    return Outcome("R1", "PASS", [f"R1: {len(eligible)} eligible Balancing window(s), op+ev streams quiet"])


def _evaluate_r2(sorted_records: Sequence[Record], ticks: Sequence[Tick], tk_ts: Sequence[int],
                  context_timeline: Sequence[Dict[str, Optional[int]]]) -> Outcome:
    """R2 (T2, C2): CONNECTING interrupted by Balancing -> exactly-bounded abort.

    Only the *first* qualifying transition is judged (mirrors R3/R5/R3b: a
    T2 capture is expected to exercise the scenario once; see _evaluate_r3's
    docstring for why "first hit only" is the correct multi-episode
    semantics whenever more than one qualifying transition is present).
    """
    positions = _find_q_rising_transitions(sorted_records, context_timeline, require_cing=1)
    if not positions:
        return Outcome("R2", "FAIL", ["R2: no cing=1 -> q:0->1 transition found (scenario not exercised)"])

    anchor_idx = owning_tick_idx(ticks, tk_ts, sorted_records[positions[0]].t)
    if anchor_idx is None:
        return Outcome("R2", "INVALID", [f"R2: transition at {sorted_records[positions[0]].cite()} precedes any tk row"])
    result = _evaluate_abort_episode(sorted_records, ticks, tk_ts, anchor_idx, "R2")
    return Outcome("R2", result.status, result.evidence)


def _evaluate_r3(sorted_records: Sequence[Record], ticks: Sequence[Tick], tk_ts: Sequence[int],
                  context_timeline: Sequence[Dict[str, Optional[int]]]) -> Outcome:
    """R3 (T3, C3 前半): established-connection one-shot reconnect, bounded cancel.

    Only the *first* qualifying `ev=disc(r!=8)` episode is judged. A T3+T3b
    combined capture (section 6: "単一の連続キャプチャ必須") legitimately
    contains a *second* such event later on (the burnt-AP re-loss that R3b's
    trigger (iii) also matches on) which is -- by T3b's own design --
    expected to have zero op activity; that second occurrence is R3b's
    concern, not a second R3 episode to hold to R3's own-abort-and-confirm
    standard (R3b's docstring: "最初の one-shot エピソードの完結" already
    names R3's target as "the first").
    """
    hits, failures = _find_disc_reconnect_triggers(sorted_records, ticks, context_timeline, mode="R3")
    if failures:
        return Outcome("R3", "INVALID", [f"R3: drain attribution failed for {failures[0].cite()}"])
    if not hits:
        return Outcome("R3", "FAIL", ["R3: no conn=1∧fsm=2 ev=disc(r!=8) episode found (scenario not exercised)"])

    result = _evaluate_abort_episode(sorted_records, ticks, tk_ts, hits[0].drain_tick_idx, "R3")
    return Outcome("R3", result.status, result.evidence)


def _evaluate_r5(sorted_records: Sequence[Record], ticks: Sequence[Tick], tk_ts: Sequence[int],
                  context_timeline: Sequence[Dict[str, Optional[int]]]) -> Outcome:
    """R5 (T5, C5): arm_pending one-shot reconnect, bounded cancel (same judge as R3).

    Only the *first* qualifying episode is judged -- see _evaluate_r3's
    docstring for the multi-episode rationale.
    """
    hits, failures = _find_disc_reconnect_triggers(sorted_records, ticks, context_timeline, mode="R5")
    if failures:
        return Outcome("R5", "INVALID", [f"R5: drain attribution failed for {failures[0].cite()}"])
    if not hits:
        return Outcome("R5", "FAIL", ["R5: no arm=1∨fsm=2 ev=disc(r!=8) episode found (scenario not exercised)"])

    result = _evaluate_abort_episode(sorted_records, ticks, tk_ts, hits[0].drain_tick_idx, "R5")
    return Outcome("R5", result.status, result.evidence)


def _evaluate_r3b(sorted_records: Sequence[Record], ticks: Sequence[Tick], tk_ts: Sequence[int],
                   context_timeline: Sequence[Dict[str, Optional[int]]]) -> Outcome:
    """R3b (T3b, C3 後半): burnt-AP re-loss after a completed reconnect, 20s+ quiet."""
    hits, failures = _find_disc_reconnect_triggers(sorted_records, ticks, context_timeline, mode="R3")
    if failures:
        return Outcome("R3b", "INVALID", [f"R3b: drain attribution failed for {failures[0].cite()}"])
    if not hits:
        return Outcome("R3b", "FAIL", ["R3b: (i) no initial one-shot episode found (scenario not exercised)"])

    first_hit = hits[0]
    episode = _evaluate_abort_episode(sorted_records, ticks, tk_ts, first_hit.drain_tick_idx, "R3b(i)")
    if episode.status != "PASS":
        return Outcome("R3b", episode.status, ["R3b: (i) initial one-shot episode did not complete cleanly"] + episode.evidence)

    anchor_t = episode.confirm_t if episode.confirm_t is not None else episode.window_end_t

    # (ii) reconnection: ev=gotip in q=0 context, then st/hb showing conn=1∧udpr=1.
    gotip: Optional[Record] = None
    for pos, r in enumerate(sorted_records):
        if r.t <= anchor_t or r.kind != "ev" or r.f["ev"] != "gotip":
            continue
        if context_timeline[pos].get("q") != 0:
            continue
        gotip = r
        break
    if gotip is None:
        return Outcome("R3b", "FAIL", ["R3b: (ii) no post-abort ev=gotip (q=0 context) reconnection observed"])

    reconnect_row: Optional[Record] = None
    for r in sorted_records:
        if r.t <= gotip.t or r.kind not in ("st", "hb"):
            continue
        if r.f.get("conn") == 1 and r.f.get("udpr") == 1:
            reconnect_row = r
            break
    if reconnect_row is None:
        return Outcome("R3b", "FAIL", ["R3b: (ii) no st/hb with conn=1∧udpr=1 after reconnection ev=gotip"])
    reconnect_t = reconnect_row.t

    # (iii) new ev=disc(r!=8) in fsm=2∨arm=1 context after reconnection.
    hits2, failures2 = _find_disc_reconnect_triggers(
        sorted_records, ticks, context_timeline, mode="R5", after_t=reconnect_t
    )
    if failures2:
        return Outcome("R3b", "INVALID", [f"R3b: (iii) drain attribution failed for {failures2[0].cite()}"])
    if not hits2:
        return Outcome("R3b", "FAIL", ["R3b: (iii) no burnt-AP re-loss ev=disc(r!=8) found after reconnection"])

    hit_ev = hits2[0].ev
    hit_t = hit_ev.t
    # The hit's own physical disconnection can legitimately surface as more
    # than one ev=disc(r!=8) row very close together (e.g. a raw beacon-
    # timeout-style reason event immediately followed by a duplicate/re-
    # delivered non-self-initiated disconnect callback a tick or so later)
    # without guard ever issuing an op -- that is the same underlying
    # AP-loss "hit" settling, not new library activity. Tolerate ev=disc
    # *only when r!=8* within a small ANCHOR_TICK_BUDGET grace after the
    # hit's own drain tick.
    #
    # r=8 (WIFI_REASON_ASSOC_LEAVE) is never tolerated, in or out of the
    # grace window: in a burnt (one-shot-already-consumed) scenario nothing
    # on this device calls disconnect(), so a self-initiated ("we left the
    # association") disconnect event cannot occur spontaneously -- its mere
    # appearance is direct evidence of an internal `disconnect(); begin();`
    # re-arm (the exact signature R3b (b) / section 9's escalation note are
    # watching for), so it must FAIL even inside the settling grace period.
    # Anything beyond the grace window (in particular the 18s-delayed-
    # activity case) still counts as a violation regardless of reason.
    hit_drain_idx = hits2[0].drain_tick_idx
    grace_tick_idx = min(len(ticks) - 1, hit_drain_idx + ANCHOR_TICK_BUDGET)
    grace_end_t = ticks[grace_tick_idx][1].t

    # Observation window: fsm=2∨arm=1 context must hold continuously from the
    # hit onward for >= 20s (section 5 "観測窓の下限").
    break_row: Optional[Record] = None
    for r in sorted_records:
        if r.t <= hit_t or r.kind not in ("st", "hb"):
            continue
        if r.f.get("fsm") != 2 and r.f.get("arm") != 1:
            break_row = r
            break

    last_t = sorted_records[-1].t
    effective_end_t = break_row.t if break_row is not None else last_t
    observed_span = effective_end_t - hit_t
    if observed_span < R3B_OBSERVATION_WINDOW_US:
        reason = "observation window insufficient"
        if break_row is not None:
            reason += f" (fsm=2∨arm=1 context broke at {break_row.cite()})"
        return Outcome("R3b", "INVALID", [f"R3b: {reason}: {observed_span}us < {R3B_OBSERVATION_WINDOW_US}us"])

    for r in sorted_records:
        if r.t <= hit_t or r.t > effective_end_t:
            continue
        if r.kind == "op":
            return Outcome("R3b", "FAIL", [f"R3b: op activity observed during 20s+ post-hit quiet window ({r.cite()})"])
        if r.kind == "ev" and r.f["ev"] in LIBRARY_ACTIVITY_EV_TYPES:
            return Outcome("R3b", "FAIL", [f"R3b: library activity ev={r.f['ev']} observed during post-hit quiet window ({r.cite()})"])
        if r.kind == "ev" and r.f["ev"] == "disc":
            if r.f["r"] == 8:
                return Outcome(
                    "R3b",
                    "FAIL",
                    [f"R3b: ev=disc r=8 (self-initiated ASSOC_LEAVE) observed -- one-shot re-arm signature ({r.cite()})"],
                )
            if r.t > grace_end_t:
                return Outcome(
                    "R3b",
                    "FAIL",
                    [f"R3b: additional ev=disc(r!=8) observed beyond the hit's own settling window ({r.cite()})"],
                )

    return Outcome(
        "R3b",
        "PASS",
        [f"R3b: burnt-AP re-loss ({hit_ev.cite()}) followed by {observed_span}us of confirmed quiet (op+ev)"],
    )


def _evaluate_r4(sorted_records: Sequence[Record]) -> Outcome:
    """R4 (all logs, C4): WiFiUDP setup-resource (tx_buffer/socket) bp gating."""
    bp_ops = [r for r in sorted_records if r.kind == "op" and r.f["op"] == "bp"]
    if not bp_ops:
        return Outcome("R4", "PASS", ["R4: no bp op rows present (no WiFiUDP setup/send observed)"])

    warn_notes: List[str] = []
    for idx, r in enumerate(bp_ops):
        is_first_overall = idx == 0
        is_setup_class = r.f["udpr"] == 0 or is_first_overall
        if is_setup_class and r.f["q"] != 0:
            return Outcome("R4", "FAIL", [f"R4(a/b): setup-class bp issued with q!=0 ({r.cite()})"])
        if (r.f["fsm"] == 2 or r.f["arm"] == 1) and r.f["udpr"] != 1:
            return Outcome("R4", "FAIL", [f"R4(c): bp during fsm=2∨arm=1 context with udpr!=1 ({r.cite()})"])

        overlaps_realtime_window = r.f["fsm2"] == 2 or r.f["arm2"] == 1 or r.f["fsm3"] == 2 or r.f["arm3"] == 1
        if is_setup_class and overlaps_realtime_window:
            return Outcome(
                "R4",
                "FAIL",
                [f"R4: setup-class bp acquisition overlapped a real-time Balancing/arm window ({r.cite()})"],
            )
        if not is_setup_class and overlaps_realtime_window:
            warn_notes.append(f"R4 WARN: reused (udpr=1) bp send overlapped a real-time Balancing/arm window ({r.cite()})")

    status = "WARN" if warn_notes else "PASS"
    evidence = [f"R4: {len(bp_ops)} bp op row(s) checked, all setup-class gated on q=0"] + warn_notes
    return Outcome("R4", status, evidence)


def _evaluate_common_validity(sorted_records: Sequence[Record]) -> Outcome:
    """Section 5 "共通妥当性": log-quality INVALID conditions, always run."""
    drop_rows = [r for r in sorted_records if r.kind == "drop"]
    if drop_rows:
        return Outcome("common", "INVALID", [f"common: drop line present ({drop_rows[0].cite()})"])

    hb_rows = [r for r in sorted_records if r.kind == "hb"]
    for hb in hb_rows:
        if hb.f["evdrop"] != 0:
            return Outcome("common", "INVALID", [f"common: hb evdrop!=0 ({hb.cite()})"])
        if hb.f["evo"] != 0:
            return Outcome("common", "INVALID", [f"common: hb evo!=0 ({hb.cite()})"])
        if hb.f["flmax"] > FLMAX_BUDGET_US:
            return Outcome("common", "INVALID", [f"common: hb flmax={hb.f['flmax']}us > {FLMAX_BUDGET_US}us ({hb.cite()})"])
        if hb.f["stkmin"] < STKMIN_MIN_BYTES:
            return Outcome("common", "INVALID", [f"common: hb stkmin={hb.f['stkmin']} < {STKMIN_MIN_BYTES} ({hb.cite()})"])

    if not sorted_records or sorted_records[0].kind != "boot":
        return Outcome("common", "INVALID", ["common: log does not start with a boot line"])
    if len(hb_rows) < 2:
        return Outcome("common", "INVALID", [f"common: only {len(hb_rows)} hb row(s) present, need >= 2"])
    if sorted_records[-1].kind != "hb":
        return Outcome("common", "INVALID", [f"common: last [WG1] line is not hb ({sorted_records[-1].cite()})"])

    ev_op_rows = [r for r in sorted_records if r.kind in ("ev", "op")]
    if ev_op_rows:
        last_ev_op = ev_op_rows[-1]
        if not any(hb.t > last_ev_op.t for hb in hb_rows):
            return Outcome("common", "INVALID", [f"common: no hb after last ev/op ({last_ev_op.cite()})"])

    # Latch/radio-off classification (section 5): a hazard state already
    # present at the very first st/hb (before any hit episode could explain
    # it) is an evidence-integrity problem, not a rule violation.
    first_ctx = next((r for r in sorted_records if r.kind in ("st", "hb")), None)
    if first_ctx is not None and (first_ctx.f.get("latch") == 1 or first_ctx.f.get("roff") == 1):
        return Outcome(
            "common",
            "INVALID",
            [f"common: latch/roff already set at first st/hb, unrelated to any hit episode ({first_ctx.cite()})"],
        )

    return Outcome("common", "PASS", [])


@dataclass
class CheckResult:
    outcomes: List[Outcome]

    @property
    def ok(self) -> bool:
        return all(o.status in ("PASS", "WARN") for o in self.outcomes)

    def render(self) -> List[str]:
        lines: List[str] = []
        for o in self.outcomes:
            lines.append(f"[{o.status}] {o.name}")
            for e in o.evidence:
                lines.append(f"    {e}")
        lines.append("")
        lines.append("RESULT: " + ("PASS" if self.ok else "FAIL"))
        return lines


_RULE_EVALUATORS = {
    "R1": lambda sr, tk, tkt, ctx: _evaluate_r1(sr),
    "R2": lambda sr, tk, tkt, ctx: _evaluate_r2(sr, tk, tkt, ctx),
    "R3": lambda sr, tk, tkt, ctx: _evaluate_r3(sr, tk, tkt, ctx),
    "R3b": lambda sr, tk, tkt, ctx: _evaluate_r3b(sr, tk, tkt, ctx),
    "R5": lambda sr, tk, tkt, ctx: _evaluate_r5(sr, tk, tkt, ctx),
}


def check_log(text: str, scenario: str) -> CheckResult:
    """Run preprocessing + common validity + R4 (always) + the one rule
    selected by `scenario` (t1/t2/t3/t3b/t5). Pure function of `text`.
    """
    if scenario not in RULE_BY_SCENARIO:
        raise ValueError(f"unknown scenario {scenario!r}, expected one of {sorted(RULE_BY_SCENARIO)}")

    records, malformed = load_wg1_lines(text)

    boot_reason = check_boot_structure(records, malformed)
    if boot_reason is not None:
        return CheckResult([Outcome("preprocessing", "INVALID", [f"preprocessing: {boot_reason}"])])

    continuity_reason = check_s_continuity(records, malformed)
    if continuity_reason is not None:
        return CheckResult([Outcome("preprocessing", "INVALID", [f"preprocessing: {continuity_reason}"])])

    sorted_records = sorted(records, key=lambda r: r.t)
    context_timeline = build_context_timeline(sorted_records)
    ticks = build_ticks(sorted_records)
    tk_ts = [r.t for _pos, r in ticks]

    outcomes = [Outcome("preprocessing", "PASS", [])]
    outcomes.append(_evaluate_common_validity(sorted_records))
    outcomes.append(_evaluate_r4(sorted_records))

    rule_name = RULE_BY_SCENARIO[scenario]
    outcomes.append(_RULE_EVALUATORS[rule_name](sorted_records, ticks, tk_ts, context_timeline))

    return CheckResult(outcomes)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Judge a Wi-Fi guard [WG1] trace capture (grammar v1)")
    parser.add_argument("--scenario", required=True, choices=sorted(RULE_BY_SCENARIO), help="which scenario-specific rule to run (R4 + common validity always run)")
    parser.add_argument("log", type=Path, help="path to a captured trace log")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    text = args.log.read_text(encoding="utf-8", errors="replace")
    result = check_log(text, args.scenario)
    for line in result.render():
        print(line)
    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
