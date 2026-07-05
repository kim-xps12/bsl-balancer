"""UDP telemetry receiver -> session directory logger (schema v1).

Binds 0.0.0.0:<port> and appends every accepted packet to
logs/telemetry/<session_id>/raw.jsonl. The per-packet hot path is
intentionally minimal: decode -> attach host_receive_ns -> schema validate
-> append. All heavier analysis (loss stats, event detection) is deferred to
analyzer.py, invoked once when a session closes.

See docs/plans/2026-07-05-udp-telemetry-phase1.md section 3.2 ("receiver").

CLI usage:
    python3 tools/telemetry/bsl_telemetry/receiver.py --port 45678 --log-root logs/telemetry
"""

from __future__ import annotations

import argparse
import json
import re
import signal
import socket
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

# Support both `python3 tools/telemetry/bsl_telemetry/receiver.py` (plain
# script execution) and `python3 -m bsl_telemetry.receiver` (package
# execution, run from tools/telemetry/) without relying on a specific cwd.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from bsl_telemetry import analyzer, schema
else:
    from . import analyzer, schema

_SESSION_ID_DEV_RE = re.compile(r"[^A-Za-z0-9_-]+")


def _sanitize_dev(dev: str) -> str:
    """Make a device id filesystem/regex-safe for use in a session_id."""
    cleaned = _SESSION_ID_DEV_RE.sub("_", str(dev)).strip("_")
    return cleaned or "unknown"


def _empty_rejected_tally() -> Dict[str, Any]:
    return {"total": 0, "by_source": {}, "by_reason": {}}


def _tally_rejected(tally: Dict[str, Any], source_ip: str, reason: str) -> None:
    tally["total"] += 1
    tally["by_source"][source_ip] = tally["by_source"].get(source_ip, 0) + 1
    tally["by_reason"][reason] = tally["by_reason"].get(reason, 0) + 1


class _Session:
    """Mutable state for one active telemetry session."""

    def __init__(
        self,
        session_dir: Path,
        session_id: str,
        dev: str,
        fw: str,
        pinned_ip: str,
        port: int,
        device_ip_arg: Optional[str],
        rejected: Dict[str, Any],
    ):
        self.session_dir = session_dir
        self.session_id = session_id
        self.dev = dev
        self.fw = fw
        self.pinned_ip = pinned_ip
        self.port = port
        self.device_ip_arg = device_ip_arg
        self.rejected = rejected
        self.packet_count = 0
        self.variant_counts = {"full": 0, "diagnostic": 0}
        self.started_at = datetime.now().astimezone().isoformat(timespec="seconds")
        self.last_activity_monotonic = time.monotonic()

        session_dir.mkdir(parents=True, exist_ok=False)
        self.raw_path = session_dir / "raw.jsonl"
        self._raw_file = self.raw_path.open("a", encoding="utf-8")
        self._write_metadata(ended_at=None)

    def _write_metadata(self, ended_at: Optional[str]) -> None:
        metadata = {
            "schema": "bsl-telemetry-session-v1",
            "session_id": self.session_id,
            "started_at": self.started_at,
            "ended_at": ended_at,
            "device_id": self.dev,
            "firmware": self.fw,
            "udp_port": self.port,
            "pinned_source_ip": self.pinned_ip,
            "device_ip_arg": self.device_ip_arg,
            # Phase 1 threat model: pinning guards against misconfiguration /
            # accidental multi-device mixing, not an active LAN attacker.
            "authenticated": False,
            "packet_count": self.packet_count,
            "variant_counts": self.variant_counts,
            "rejected_packets": self.rejected,
        }
        tmp_path = self.session_dir / "metadata.json.tmp"
        with tmp_path.open("w", encoding="utf-8") as handle:
            json.dump(metadata, handle, indent=2, sort_keys=True)
            handle.write("\n")
        tmp_path.replace(self.session_dir / "metadata.json")

    def append_raw(self, host_receive_ns: int, source_ip: str, packet: Dict[str, Any], variant: str) -> None:
        record = {"host_receive_ns": host_receive_ns, "source_ip": source_ip, "packet": packet}
        self._raw_file.write(json.dumps(record, sort_keys=True))
        self._raw_file.write("\n")
        self._raw_file.flush()
        self.packet_count += 1
        self.variant_counts[variant] += 1
        self.last_activity_monotonic = time.monotonic()

    def record_rejected(self, source_ip: str, reason: str) -> None:
        _tally_rejected(self.rejected, source_ip, reason)
        self.last_activity_monotonic = time.monotonic()

    def close(self) -> Dict[str, Any]:
        ended_at = datetime.now().astimezone().isoformat(timespec="seconds")
        self._write_metadata(ended_at=ended_at)
        self._raw_file.close()
        # Analysis is deferred entirely to analyzer.py, which re-derives
        # events.jsonl/summary.json purely from raw.jsonl + metadata.json
        # (so a session can always be re-analyzed later, section 3.2).
        return analyzer.analyze_session(self.session_dir, write=True)


class TelemetryReceiver:
    """UDP telemetry receiver. One socket, at most one active session.

    Usable both as the CLI entry point and directly from tests: instantiate
    with port=0 to bind an ephemeral port (see `bound_port`), run
    `serve_forever()` on a background thread, and call `request_stop()` to
    end it (SIGINT does the same via KeyboardInterrupt in `main()`).
    """

    def __init__(
        self,
        port: int,
        log_root: Path,
        device_ip: Optional[str] = None,
        idle_timeout_s: float = 10.0,
        poll_interval_s: float = 0.2,
    ):
        self.log_root = Path(log_root)
        self.log_root.mkdir(parents=True, exist_ok=True)
        self.explicit_device_ip = device_ip
        self.idle_timeout_s = idle_timeout_s

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", port))
        self._sock.settimeout(poll_interval_s)
        self.bound_port: int = self._sock.getsockname()[1]

        self._stop = threading.Event()
        self.session: Optional[_Session] = None
        self.pinned_ip: Optional[str] = device_ip
        self._pre_session_rejected = _empty_rejected_tally()
        # Populated as each session closes; tests/CLI inspect this list.
        self.closed_sessions: List[Path] = []
        self.last_close_result: Optional[Dict[str, Any]] = None

    def request_stop(self) -> None:
        self._stop.set()

    def serve_forever(self) -> None:
        try:
            while not self._stop.is_set():
                try:
                    data, addr = self._sock.recvfrom(65535)
                except socket.timeout:
                    self._check_idle()
                    continue
                self._handle_datagram(data, addr[0])
                self._check_idle()
        except KeyboardInterrupt:
            pass
        finally:
            if self.session is not None:
                self._close_session()
            self._sock.close()

    def _check_idle(self) -> None:
        if self.session is None:
            return
        if time.monotonic() - self.session.last_activity_monotonic >= self.idle_timeout_s:
            self._close_session()

    def _handle_datagram(self, data: bytes, source_ip: str) -> None:
        host_receive_ns = time.time_ns()

        try:
            text = data.decode("utf-8")
        except UnicodeDecodeError:
            self._reject(source_ip, "decode_error")
            return

        try:
            obj = json.loads(text)
        except json.JSONDecodeError:
            self._reject(source_ip, "json_error")
            return

        try:
            variant = schema.validate_packet(obj)
        except schema.PacketValidationError as exc:
            self._reject(source_ip, f"schema:{exc.reason}")
            return

        if self.pinned_ip is not None and source_ip != self.pinned_ip:
            self._reject(source_ip, "ip_pinning_mismatch")
            return

        if self.session is None:
            self.pinned_ip = source_ip
            self._start_session(source_ip, obj)

        self.session.append_raw(host_receive_ns, source_ip, obj, variant)  # type: ignore[union-attr]

    def _reject(self, source_ip: str, reason: str) -> None:
        if self.session is not None:
            self.session.record_rejected(source_ip, reason)
        else:
            _tally_rejected(self._pre_session_rejected, source_ip, reason)

    def _next_run_number(self, dev: str) -> int:
        pattern = re.compile(rf"^\d{{8}}-\d{{6}}_{re.escape(dev)}_run-(\d+)$")
        max_run = 0
        for entry in self.log_root.iterdir():
            if not entry.is_dir():
                continue
            match = pattern.match(entry.name)
            if match:
                max_run = max(max_run, int(match.group(1)))
        return max_run + 1

    def _start_session(self, source_ip: str, first_packet: Dict[str, Any]) -> None:
        dev = _sanitize_dev(first_packet.get("dev", "unknown"))
        fw = str(first_packet.get("fw", "unknown"))
        run_no = self._next_run_number(dev)
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        session_id = f"{timestamp}_{dev}_run-{run_no:03d}"
        session_dir = self.log_root / session_id

        rejected = self._pre_session_rejected
        self._pre_session_rejected = _empty_rejected_tally()

        self.session = _Session(
            session_dir=session_dir,
            session_id=session_id,
            dev=dev,
            fw=fw,
            pinned_ip=source_ip,
            port=self.bound_port,
            device_ip_arg=self.explicit_device_ip,
            rejected=rejected,
        )

    def _close_session(self) -> None:
        session = self.session
        self.session = None
        if self.explicit_device_ip is None:
            self.pinned_ip = None
        assert session is not None
        self.last_close_result = session.close()
        self.closed_sessions.append(session.session_dir)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="BSL telemetry UDP receiver (Phase 1)")
    parser.add_argument("--port", type=int, required=True, help="UDP port to bind on 0.0.0.0")
    parser.add_argument("--log-root", type=Path, default=Path("logs/telemetry"), help="session directory root")
    parser.add_argument(
        "--device-ip",
        default=None,
        help="explicit allowlisted source IP; overrides first-packet pinning",
    )
    parser.add_argument(
        "--idle-timeout",
        type=float,
        default=10.0,
        help="seconds of silence before auto-closing the active session (default: 10)",
    )
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    receiver = TelemetryReceiver(
        port=args.port,
        log_root=args.log_root,
        device_ip=args.device_ip,
        idle_timeout_s=args.idle_timeout,
    )
    print(
        f"bsl-telemetry receiver listening on 0.0.0.0:{receiver.bound_port} "
        f"(log_root={args.log_root}, idle_timeout={args.idle_timeout}s)"
    )
    if args.device_ip:
        print(f"device-ip pinning: {args.device_ip}")
    print("Ctrl-C to stop (writes summary.json for the active session before exiting)")

    # Install an explicit handler rather than relying on Python's default
    # SIGINT->KeyboardInterrupt behavior: some launch contexts (e.g. a shell
    # background job via `&`) have SIGINT pre-set to SIG_IGN, which Python
    # inherits verbatim and would otherwise leave Ctrl-C/kill -INT with no
    # effect. Setting our own handler makes shutdown reliable regardless of
    # how the process was started, and reuses the same stop path as the
    # idle-timeout auto-close (`request_stop()` -> serve_forever's poll loop).
    def _handle_sigint(signum, frame):  # noqa: ARG001 - required signal handler signature
        receiver.request_stop()

    signal.signal(signal.SIGINT, _handle_sigint)

    try:
        receiver.serve_forever()
    finally:
        for session_dir in receiver.closed_sessions:
            print(f"session closed: {session_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
