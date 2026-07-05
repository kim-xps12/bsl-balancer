"""Markdown digest of a bsl-telemetry session for humans/coding agents.

Reads metadata.json + summary.json + events.jsonl (as produced by
analyzer.py) plus bounded raw.jsonl excerpts for each recommended window,
and prints a single markdown document to stdout. This is the artifact a
coding agent should read first (section 3.2 "report"); it does not read
the full raw.jsonl.

See docs/plans/2026-07-05-udp-telemetry-phase1.md section 3.2 ("report").

CLI usage:
    python3 tools/telemetry/bsl_telemetry/report.py logs/telemetry/<session_id>
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Optional

DT_H_BIN_LABELS = ("<1.02x", "<1.05x", "<1.1x", "<1.2x", "<1.3x", "<1.5x", "<2.0x", ">=2.0x")


def load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def load_jsonl(path: Path) -> List[Dict[str, Any]]:
    items: List[Dict[str, Any]] = []
    if not path.exists():
        return items
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                items.append(json.loads(line))
    return items


def load_raw_index_by_seq(raw_path: Path) -> Dict[int, Dict[str, Any]]:
    index: Dict[int, Dict[str, Any]] = {}
    for record in load_jsonl(raw_path):
        seq = record.get("packet", {}).get("seq")
        if seq is not None:
            index[seq] = record
    return index


def _format_event_span(event: Dict[str, Any]) -> str:
    if "start_seq" in event:
        return f"seq {event['start_seq']}-{event['end_seq']}"
    if "seq" in event:
        return f"seq {event['seq']}"
    return ""


def _format_event_extra(event: Dict[str, Any]) -> str:
    extra = {k: v for k, v in event.items() if k not in ("type", "severity", "start_seq", "end_seq", "seq")}
    return json.dumps(extra, sort_keys=True) if extra else ""


def _raw_excerpt(
    raw_index: Dict[int, Dict[str, Any]], start_seq: int, end_seq: int, limit: int
) -> List[Dict[str, Any]]:
    seqs = sorted(seq for seq in raw_index if start_seq <= seq <= end_seq)[:limit]
    return [raw_index[seq] for seq in seqs]


def render_report(session_dir: Path, raw_excerpt_limit: int = 20) -> str:
    session_dir = Path(session_dir)
    metadata_path = session_dir / "metadata.json"
    summary_path = session_dir / "summary.json"
    events_path = session_dir / "events.jsonl"

    metadata = load_json(metadata_path) if metadata_path.exists() else {}

    if not summary_path.exists():
        return (
            f"# {session_dir.name}\n\n"
            "summary.json not found. Generate it first:\n\n"
            f"    python3 tools/telemetry/bsl_telemetry/analyzer.py {session_dir}\n"
        )

    summary = load_json(summary_path)
    events = load_jsonl(events_path)

    lines: List[str] = []
    lines.append(f"# Telemetry session report: {summary.get('session_id', session_dir.name)}")
    lines.append("")
    lines.append(f"- device: `{summary.get('device_id')}`  firmware: `{summary.get('firmware')}`")
    lines.append(f"- started_at: {summary.get('started_at')}  ended_at: {summary.get('ended_at')}")
    duration_s = summary.get("duration_s")
    lines.append(f"- duration: {duration_s:.1f} s" if duration_s is not None else "- duration: n/a")
    variant_counts = summary.get("variant_counts", {})
    lines.append(
        f"- packets: {summary.get('packet_count')} "
        f"(full={variant_counts.get('full')}, diagnostic={variant_counts.get('diagnostic')})"
    )
    lines.append(
        f"- authenticated: {metadata.get('authenticated', 'unknown')} "
        "(Phase 1 threat model: source-IP pinning only, not an authentication boundary)"
    )
    lines.append("")

    lines.append("## Loss / timing")
    loss = summary.get("loss", {})
    loss_rate = loss.get("loss_rate") or 0.0
    lines.append(
        f"- seq loss: {loss.get('lost_estimate')} packets estimated lost "
        f"({loss_rate * 100:.2f}%), {loss.get('gap_count')} gap(s)"
    )
    dt_us = summary.get("dt_us", {})
    lines.append(
        f"- dt_us (direct sample): mean={dt_us.get('mean')}, p50={dt_us.get('p50')}, "
        f"p95={dt_us.get('p95')}, max={dt_us.get('max')}"
    )
    histogram = summary.get("dt_histogram", {})
    totals = histogram.get("totals") or []
    fractions = histogram.get("fractions") or []
    if totals:
        hist_line = ", ".join(
            f"{label}={count}({frac * 100:.2f}%)"
            for label, count, frac in zip(DT_H_BIN_LABELS, totals, fractions)
        )
        lines.append(f"- dt histogram (reconstructed over {histogram.get('intervals')} intervals): {hist_line}")
    lines.append(f"- overrun_total delta: {summary.get('overrun_total_delta')}")
    lines.append(f"- imu_stale_total delta: {summary.get('imu_stale_total_delta')}")
    lines.append(
        f"- read_fail_total delta: {summary.get('read_fail_total_delta')}, "
        f"trunc_total delta: {summary.get('trunc_total_delta')}"
    )
    lines.append(f"- diagnostic packets by reason: {summary.get('diagnostic_by_reason')}")
    tick_seq_gap = summary.get("tick_seq_gap", {})
    lines.append(f"- tick-seq gap: max={tick_seq_gap.get('max')}, mean={tick_seq_gap.get('mean')}")
    lines.append(f"- reboot_count: {summary.get('reboot_count')}")
    lines.append("")

    rejected = summary.get("rejected_packets", {})
    lines.append("## Rejected / invalid packets")
    lines.append(f"- total rejected: {rejected.get('total', 0)}")
    if rejected.get("by_source"):
        lines.append(f"- by source: {rejected.get('by_source')}")
    if rejected.get("by_reason"):
        lines.append(f"- by reason: {rejected.get('by_reason')}")
    lines.append("")

    lines.append(f"## Events ({len(events)})")
    if not events:
        lines.append("(none)")
    else:
        for event in events:
            extra = _format_event_extra(event)
            suffix = f" {extra}" if extra else ""
            lines.append(f"- `{event.get('type')}` [{event.get('severity')}] {_format_event_span(event)}{suffix}")
    lines.append("")

    windows = summary.get("recommended_windows", [])
    lines.append(f"## Recommended windows ({len(windows)})")
    if not windows:
        lines.append("(none)")
    else:
        raw_index = load_raw_index_by_seq(session_dir / "raw.jsonl")
        for window in windows:
            lines.append(
                f"### {window.get('label')} "
                f"[seq {window.get('start_seq')}-{window.get('end_seq')}] "
                f"({window.get('severity')})"
            )
            excerpt = _raw_excerpt(raw_index, window["start_seq"], window["end_seq"], raw_excerpt_limit)
            if excerpt:
                lines.append("```jsonl")
                lines.extend(json.dumps(record["packet"], sort_keys=True) for record in excerpt)
                lines.append("```")
            else:
                lines.append("(no matching raw packets in this seq range)")
            lines.append("")

    return "\n".join(lines) + "\n"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Print a markdown digest of a bsl-telemetry session")
    parser.add_argument("session_dir", type=Path, help="logs/telemetry/<session_id> directory")
    parser.add_argument(
        "--raw-excerpt-limit",
        type=int,
        default=20,
        help="max raw packets shown per recommended window (default: 20)",
    )
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    print(render_report(args.session_dir, raw_excerpt_limit=args.raw_excerpt_limit), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
