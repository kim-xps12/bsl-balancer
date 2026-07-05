"""Post-run analysis of a bsl-telemetry session (schema v1).

Pure, deterministic functions that reconstruct loss/timing/event
information purely from a session's raw.jsonl (plus metadata.json for
identity/rejected-packet bookkeeping that the receiver alone can observe).
Nothing here depends on wall-clock time or network state, so a session can
be re-analyzed at any time and will always produce the same events.jsonl /
summary.json for the same raw.jsonl + metadata.json inputs.

See docs/plans/2026-07-05-udp-telemetry-phase1.md section 3.2 ("analyzer").

CLI usage:
    python3 tools/telemetry/bsl_telemetry/analyzer.py logs/telemetry/<session_id>
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

DT_H_LEN = 8

# uint32 wrap-aware delta helpers (seq/loop/dt_h/ovr/stale/read_fail/trunc
# are all firmware uint32 counters, see section 3.1).
_UINT32_MOD = 1 << 32
_UINT32_REBOOT_THRESHOLD = 1 << 31

# Event types considered worth surfacing first, in priority order (used to
# rank summary.json's recommended_windows).
_WINDOW_PRIORITY = (
    "reboot",
    "reboot_lead_in",
    "control_task_stall",
    "fault_transition",
    "loss",
    "saturation",
    "i2t",
    "fsm_transition",
    "device_change",
    "firmware_change",
)


def _uint32_forward_delta(prev: int, curr: int) -> Optional[int]:
    """Wrap-aware forward delta for a monotonic uint32 counter.

    Returns the non-negative delta assuming forward progress (allowing for
    at most one wraparound), or None if the observed jump looks like a
    backward reset (device reboot) rather than legitimate wraparound.
    """
    delta = (curr - prev) % _UINT32_MOD
    if delta >= _UINT32_REBOOT_THRESHOLD:
        return None
    return delta


def _percentile(sorted_values: Sequence[float], pct: float) -> Optional[float]:
    """Nearest-rank percentile. Returns None for an empty sequence."""
    if not sorted_values:
        return None
    k = int(round((pct / 100.0) * (len(sorted_values) - 1)))
    k = max(0, min(len(sorted_values) - 1, k))
    return sorted_values[k]


def load_raw_records(raw_path: Path) -> List[Dict[str, Any]]:
    """Load raw.jsonl into a list of {host_receive_ns, source_ip, packet} dicts.

    Malformed lines are skipped (they should not occur since receiver.py only
    appends already-validated packets, but this keeps re-analysis robust
    against a hand-edited or truncated raw.jsonl).
    """
    records: List[Dict[str, Any]] = []
    if not raw_path.exists():
        return records
    with raw_path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return records


def load_metadata(session_dir: Path) -> Dict[str, Any]:
    metadata_path = session_dir / "metadata.json"
    if not metadata_path.exists():
        return {}
    with metadata_path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _region_tracker_update(
    state: Optional[Tuple[int, int]],
    active: bool,
    seq: int,
    epoch: int,
    event_type: str,
    events: List[Dict[str, Any]],
) -> Optional[Tuple[int, int]]:
    """Track a contiguous run of `active == True` over ordered full packets.

    `state` is (start_seq, last_true_seq) or None. Appends a closed-region
    event to `events` the moment the run ends. Returns the updated state.
    `epoch` tags the closed event with its reboot epoch (see
    assign_reboot_epochs) so events sort correctly across a reboot boundary
    even though `seq` restarts from 1 (指摘2/3).
    """
    if active:
        if state is None:
            return (seq, seq)
        return (state[0], seq)
    if state is not None:
        events.append(
            {
                "type": event_type,
                "severity": "warn",
                "epoch": epoch,
                "start_seq": state[0],
                "end_seq": state[1],
            }
        )
    return None


def _close_open_regions(
    events: List[Dict[str, Any]],
    epoch: int,
    sat_region: Optional[Tuple[int, int]],
    i2t_region: Optional[Tuple[int, int]],
    loop_stall_start: Optional[int],
    prev_full: Optional[Dict[str, Any]],
) -> None:
    """Close any still-open saturation/i2t/control-task-stall region as an event.

    Used both (a) when a reboot is detected mid-session -- so an episode
    that was still in progress right before the reboot is reported instead
    of silently discarded when the baselines are reset (指摘1: without
    this, only the reboot event would show up and the pre-reboot
    saturation/i2t/stall precursor would vanish) -- and (b) at the very end
    of the session, for a region that is still open when records run out.
    `epoch` should be the epoch the open region belongs to (the pre-reboot
    epoch in case (a), the final epoch in case (b)).
    """
    if sat_region is not None:
        events.append(
            {"type": "saturation", "severity": "warn", "epoch": epoch, "start_seq": sat_region[0], "end_seq": sat_region[1]}
        )
    if i2t_region is not None:
        events.append(
            {"type": "i2t", "severity": "warn", "epoch": epoch, "start_seq": i2t_region[0], "end_seq": i2t_region[1]}
        )
    if loop_stall_start is not None and prev_full is not None:
        events.append(
            {
                "type": "control_task_stall",
                "severity": "error",
                "epoch": epoch,
                "start_seq": loop_stall_start,
                "end_seq": prev_full["seq"],
            }
        )


def assign_reboot_epochs(records: Sequence[Dict[str, Any]]) -> List[int]:
    """Return the reboot epoch (0-based, incremented once per detected
    reboot) for each record in `records`, aligned by position.

    A "reboot" here is exactly the condition analyze_records() uses to
    re-anchor its baselines: a uint32-wrap-aware backward jump in `seq`, or
    a backward jump in `t_us`. Firmware `seq` restarts at 1 after a reboot
    while the receiver keeps appending to the same session, so `seq` alone
    is not a safe ordering/keying key once a reboot has occurred (指摘2:
    events.jsonl/report ordering, 指摘3: report.py raw excerpt keying).
    Exposed as a standalone helper (rather than inlined only in
    analyze_records) so report.py can independently derive the same epoch
    for raw.jsonl records without analyzer.py and report.py ever
    disagreeing on where a reboot boundary falls.
    """
    epochs: List[int] = []
    epoch = 0
    prev_seq: Optional[int] = None
    prev_t_us: Optional[int] = None
    for record in records:
        packet = record["packet"]
        seq = packet["seq"]
        t_us = packet["t_us"]
        is_reboot = False
        if prev_seq is not None and _uint32_forward_delta(prev_seq, seq) is None:
            is_reboot = True
        if prev_t_us is not None and t_us < prev_t_us:
            is_reboot = True
        if is_reboot:
            epoch += 1
        epochs.append(epoch)
        prev_seq = seq
        prev_t_us = t_us
    return epochs


def analyze_records(records: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """Compute events + summary fields from a list of raw.jsonl records.

    Pure function: no I/O, deterministic given `records`. Returns a dict
    with an "events" key (list, unsorted-by-caller-safe: already sorted by
    reboot epoch then seq/start_seq -- see assign_reboot_epochs -- so the
    chronological order survives a firmware reboot restarting `seq` at 1,
    指摘2) plus assorted summary fields that callers merge with session
    metadata (device_id, firmware, rejected_packets, ...).
    """
    events: List[Dict[str, Any]] = []
    loss_events: List[Dict[str, Any]] = []
    epochs = assign_reboot_epochs(records)
    epoch = 0

    variant_counts = {"full": 0, "diagnostic": 0}
    diagnostic_by_reason = {"read_fail": 0, "trunc": 0}
    tick_seq_gaps: List[int] = []
    dt_us_samples: List[int] = []

    prev_seq: Optional[int] = None
    prev_t_us: Optional[int] = None
    lost_estimate = 0
    reboot_count = 0

    # Full-packet-only interval trackers (dt_h/ovr/stale/loop). Reset to
    # None whenever a reboot is detected so deltas are never computed across
    # a reboot boundary (re-anchoring, section 3.2).
    prev_full: Optional[Dict[str, Any]] = None
    dt_h_totals = [0] * DT_H_LEN
    dt_h_intervals = 0
    ovr_total_delta = 0
    stale_total_delta = 0
    loop_stall_start: Optional[int] = None

    # read_fail/trunc are present on both variants, so their interval
    # tracker spans every record (not just full packets).
    prev_rf_tr: Optional[Tuple[int, int]] = None
    read_fail_total_delta = 0
    trunc_total_delta = 0

    prev_fsm: Optional[int] = None
    prev_fault: Optional[int] = None
    sat_region: Optional[Tuple[int, int]] = None
    i2t_region: Optional[Tuple[int, int]] = None

    # dev/fw are present on every packet (both variants); a mid-session
    # change indicates a different device or firmware build mixed into this
    # session (misconfiguration / multi-device mixup, section 3.2 threat
    # model), independent of the reboot re-anchoring above.
    prev_dev: Optional[str] = None
    prev_fw: Optional[str] = None

    for record_idx, record in enumerate(records):
        packet = record["packet"]
        seq = packet["seq"]
        tick = packet["tick"]
        t_us = packet["t_us"]
        dev = packet.get("dev")
        fw = packet.get("fw")
        variant = "full" if packet.get("snap_valid") else "diagnostic"
        variant_counts[variant] += 1
        tick_seq_gaps.append(tick - seq)
        epoch = epochs[record_idx]

        is_reboot = False
        if prev_seq is not None:
            delta = _uint32_forward_delta(prev_seq, seq)
            if delta is None:
                is_reboot = True
            elif delta > 1:
                lost = delta - 1
                lost_estimate += lost
                loss_events.append(
                    {
                        "type": "loss",
                        "severity": "warn",
                        "epoch": epoch,
                        "start_seq": prev_seq,
                        "end_seq": seq,
                        "lost_packets": lost,
                    }
                )
        if prev_t_us is not None and t_us < prev_t_us:
            is_reboot = True

        if is_reboot:
            reboot_count += 1
            # 指摘1: close any region that was still open right before the
            # reboot -- tagged with the *pre-reboot* epoch, since it belongs
            # to the segment that is about to be discarded below -- before
            # the baselines are reset. Otherwise a saturation/i2t/stall
            # episode in progress at the moment of the reboot silently
            # vanishes and only the reboot itself gets reported.
            _close_open_regions(events, epochs[record_idx - 1], sat_region, i2t_region, loop_stall_start, prev_full)
            events.append(
                {
                    "type": "reboot",
                    "severity": "error",
                    "epoch": epoch,
                    "seq": seq,
                    "prev_seq": prev_seq,
                    "t_us": t_us,
                    "prev_t_us": prev_t_us,
                }
            )
            # Re-anchor: forget every "previous" baseline so no delta is
            # computed across the reboot boundary.
            prev_full = None
            prev_rf_tr = None
            prev_fsm = None
            prev_fault = None
            sat_region = None
            i2t_region = None
            loop_stall_start = None

        prev_seq = seq
        prev_t_us = t_us

        # dev/fw change detection happens after the reboot handling above so
        # a change observed on the very first post-reboot packet is tagged
        # with the new epoch (its `seq` belongs to the post-reboot
        # numbering, 指摘2).
        if prev_dev is not None and dev != prev_dev:
            events.append(
                {"type": "device_change", "severity": "error", "epoch": epoch, "seq": seq, "from": prev_dev, "to": dev}
            )
        if prev_fw is not None and fw != prev_fw:
            events.append(
                {"type": "firmware_change", "severity": "warn", "epoch": epoch, "seq": seq, "from": prev_fw, "to": fw}
            )
        prev_dev = dev
        prev_fw = fw

        read_fail = packet["read_fail"]
        trunc = packet["trunc"]
        if prev_rf_tr is not None:
            rf_delta = _uint32_forward_delta(prev_rf_tr[0], read_fail)
            tr_delta = _uint32_forward_delta(prev_rf_tr[1], trunc)
            if rf_delta is not None:
                read_fail_total_delta += rf_delta
            if tr_delta is not None:
                trunc_total_delta += tr_delta
        prev_rf_tr = (read_fail, trunc)

        if variant == "diagnostic":
            reason = packet["reason"]
            diagnostic_by_reason[reason] = diagnostic_by_reason.get(reason, 0) + 1
            continue

        dt_us_samples.append(packet["dt_us"])

        fsm = packet["fsm"]
        fault = packet["fault"]
        if prev_fsm is not None and fsm != prev_fsm:
            events.append(
                {"type": "fsm_transition", "severity": "info", "epoch": epoch, "seq": seq, "from": prev_fsm, "to": fsm}
            )
        if prev_fault is not None and fault != prev_fault:
            events.append(
                {
                    "type": "fault_transition",
                    "severity": "warn" if fault else "info",
                    "epoch": epoch,
                    "seq": seq,
                    "from": prev_fault,
                    "to": fault,
                }
            )
        prev_fsm = fsm
        prev_fault = fault

        sat_region = _region_tracker_update(sat_region, packet["sat"], seq, epoch, "saturation", events)
        i2t_region = _region_tracker_update(i2t_region, packet["i2t"], seq, epoch, "i2t", events)

        loop = packet["loop"]
        dt_h = packet["dt_h"]
        ovr = packet["ovr"]
        stale = packet["stale"]

        if prev_full is not None:
            loop_delta = _uint32_forward_delta(prev_full["loop"], loop)
            if loop_delta is not None:
                dt_h_intervals += 1
                for bin_idx in range(DT_H_LEN):
                    bin_delta = _uint32_forward_delta(prev_full["dt_h"][bin_idx], dt_h[bin_idx])
                    if bin_delta is not None:
                        dt_h_totals[bin_idx] += bin_delta
                ovr_delta = _uint32_forward_delta(prev_full["ovr"], ovr)
                if ovr_delta is not None:
                    ovr_total_delta += ovr_delta
                stale_delta = _uint32_forward_delta(prev_full["stale"], stale)
                if stale_delta is not None:
                    stale_total_delta += stale_delta

                if loop_delta == 0:
                    if loop_stall_start is None:
                        loop_stall_start = prev_full["seq"]
                elif loop_stall_start is not None:
                    events.append(
                        {
                            "type": "control_task_stall",
                            "severity": "error",
                            "epoch": epoch,
                            "start_seq": loop_stall_start,
                            "end_seq": prev_full["seq"],
                        }
                    )
                    loop_stall_start = None

        prev_full = {"seq": seq, "loop": loop, "dt_h": list(dt_h), "ovr": ovr, "stale": stale}

    # Close out any regions still open at the end of the session.
    _close_open_regions(events, epoch, sat_region, i2t_region, loop_stall_start, prev_full)

    events.extend(loss_events)
    events.sort(key=lambda e: (e.get("epoch", 0), e.get("start_seq", e.get("seq", 0))))

    total_expected = variant_counts["full"] + variant_counts["diagnostic"] + lost_estimate
    loss_rate = (lost_estimate / total_expected) if total_expected > 0 else 0.0

    dt_us_sorted = sorted(dt_us_samples)
    dt_us_stats = {
        "count": len(dt_us_samples),
        "mean": (sum(dt_us_samples) / len(dt_us_samples)) if dt_us_samples else None,
        "min": dt_us_sorted[0] if dt_us_sorted else None,
        "max": dt_us_sorted[-1] if dt_us_sorted else None,
        "p50": _percentile(dt_us_sorted, 50),
        "p95": _percentile(dt_us_sorted, 95),
    }

    dt_h_sum = sum(dt_h_totals)
    dt_h_fractions = [(count / dt_h_sum) if dt_h_sum else 0.0 for count in dt_h_totals]

    tick_seq_gap_stats = {
        "max": max(tick_seq_gaps) if tick_seq_gaps else None,
        "mean": (sum(tick_seq_gaps) / len(tick_seq_gaps)) if tick_seq_gaps else None,
    }

    return {
        "events": events,
        "packet_count": len(records),
        "variant_counts": variant_counts,
        "loss": {
            "lost_estimate": lost_estimate,
            "loss_rate": loss_rate,
            "gap_count": len(loss_events),
        },
        "dt_us": dt_us_stats,
        "dt_histogram": {
            "totals": dt_h_totals,
            "fractions": dt_h_fractions,
            "intervals": dt_h_intervals,
        },
        "overrun_total_delta": ovr_total_delta,
        "imu_stale_total_delta": stale_total_delta,
        "read_fail_total_delta": read_fail_total_delta,
        "trunc_total_delta": trunc_total_delta,
        "diagnostic_by_reason": diagnostic_by_reason,
        "tick_seq_gap": tick_seq_gap_stats,
        "reboot_count": reboot_count,
    }


def build_recommended_windows(
    events: Sequence[Dict[str, Any]], margin: int = 5, limit: int = 8
) -> List[Dict[str, Any]]:
    """Pick the seq windows an agent should read raw.jsonl for first.

    Ranks by event-type priority (reboot/stall/fault first, then loss,
    then saturation/i2t/fsm), widening each event's seq span by `margin` on
    both sides so the raw excerpt includes some lead-in/lead-out context.
    Each window carries the source event's `epoch` (指摘3 originally: report.py
    needs it to excerpt raw.jsonl records from the correct reboot segment,
    since `seq` alone can collide across a reboot boundary).

    A `reboot` event is a special cross-epoch case (ゲート2レビュー(2回目)
    指摘3): its own `seq`/`epoch` describe only the *post*-reboot side, so a
    single window built the generic way never shows any pre-reboot context.
    For reboot events this emits *two* windows -- the usual post-reboot one
    plus a `reboot_lead_in` window anchored on `prev_seq` in the preceding
    epoch -- so an agent reading recommended_windows always sees the cycles
    immediately leading up to the reboot, not just the fresh restart at
    seq 1. The lead-in half is only emitted when the event carries a real
    (>=1) epoch; a hand-crafted reboot event with no "epoch" field has no
    reliable pre-reboot epoch to anchor on and yields just the one window
    (kept for backward compatibility with pre-existing callers/tests that
    build reboot events without an epoch).
    """
    windows: List[Dict[str, Any]] = []
    for event in events:
        if event["type"] == "fault_transition" and not event.get("to"):
            continue  # only flag transitions INTO a fault, not recovery to 0
        if event["type"] == "reboot":
            epoch = event.get("epoch", 0)
            seq = event["seq"]
            windows.append(
                {
                    "label": "reboot",
                    "epoch": epoch,
                    "start_seq": max(0, seq - margin),
                    "end_seq": seq + margin,
                    "severity": event.get("severity", "info"),
                }
            )
            if epoch >= 1:
                prev_seq = event["prev_seq"]
                windows.append(
                    {
                        "label": "reboot_lead_in",
                        "epoch": epoch - 1,
                        "start_seq": max(0, prev_seq - margin),
                        "end_seq": prev_seq + margin,
                        "severity": event.get("severity", "info"),
                    }
                )
            continue
        if "start_seq" in event:
            start, end = event["start_seq"], event["end_seq"]
        elif "seq" in event:
            start = end = event["seq"]
        else:
            continue  # pragma: no cover - defensive, all current event types have one of the above
        windows.append(
            {
                "label": event["type"],
                "epoch": event.get("epoch", 0),
                "start_seq": max(0, start - margin),
                "end_seq": end + margin,
                "severity": event.get("severity", "info"),
            }
        )

    def sort_key(window: Dict[str, Any]) -> Tuple[int, int]:
        try:
            priority = _WINDOW_PRIORITY.index(window["label"])
        except ValueError:
            priority = len(_WINDOW_PRIORITY)
        return (priority, window["start_seq"])

    windows.sort(key=sort_key)
    return windows[:limit]


def _default_rejected_packets() -> Dict[str, Any]:
    return {"total": 0, "by_source": {}, "by_reason": {}}


def analyze_session(session_dir: Path, write: bool = True) -> Dict[str, Any]:
    """Analyze one session directory and (optionally) write events/summary.

    Reads raw.jsonl (source of truth for events/timing) and metadata.json
    (source of truth for identity + rejected_packets, which are not
    reconstructible from raw.jsonl since rejected packets are never
    appended there). Returns {"events": [...], "summary": {...}}.
    """
    session_dir = Path(session_dir)
    records = load_raw_records(session_dir / "raw.jsonl")
    metadata = load_metadata(session_dir)

    result = analyze_records(records)
    events = result.pop("events")
    summary = result

    summary["session_id"] = metadata.get("session_id", session_dir.name)
    summary["device_id"] = metadata.get("device_id")
    summary["firmware"] = metadata.get("firmware")
    summary["started_at"] = metadata.get("started_at")
    summary["ended_at"] = metadata.get("ended_at")

    if records:
        first_ns = records[0]["host_receive_ns"]
        last_ns = records[-1]["host_receive_ns"]
        summary["duration_s"] = max(0.0, (last_ns - first_ns) / 1e9)
    else:
        summary["duration_s"] = 0.0

    summary["rejected_packets"] = metadata.get("rejected_packets", _default_rejected_packets())

    event_counts: Dict[str, int] = {}
    for event in events:
        event_counts[event["type"]] = event_counts.get(event["type"], 0) + 1
    summary["event_counts"] = event_counts
    summary["recommended_windows"] = build_recommended_windows(events)

    if write:
        _write_jsonl(session_dir / "events.jsonl", events)
        _write_json(session_dir / "summary.json", summary)

    return {"events": events, "summary": summary}


def _write_jsonl(path: Path, items: Sequence[Dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        for item in items:
            handle.write(json.dumps(item, sort_keys=True))
            handle.write("\n")


def _write_json(path: Path, obj: Dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(obj, handle, indent=2, sort_keys=True)
        handle.write("\n")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Analyze a bsl-telemetry session directory")
    parser.add_argument("session_dir", type=Path, help="logs/telemetry/<session_id> directory")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    result = analyze_session(args.session_dir, write=True)
    print(f"wrote {args.session_dir / 'events.jsonl'} ({len(result['events'])} events)")
    print(f"wrote {args.session_dir / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
