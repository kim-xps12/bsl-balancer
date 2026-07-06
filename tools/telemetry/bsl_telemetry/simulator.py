"""Synthetic packet sender for exercising the receiver without hardware.

Generates schema-v1-valid packet sequences for a handful of named scenarios
and (optionally) sends them over real UDP sockets, so receiver.py,
analyzer.py and report.py can all be exercised end-to-end on localhost.

See docs/plans/2026-07-05-udp-telemetry-phase1.md section 3.2 ("simulator").

CLI usage:
    python3 tools/telemetry/bsl_telemetry/simulator.py --port 45678 --scenario fall
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from bsl_telemetry import schema
else:
    from . import schema

DEFAULT_DEV = "core2-a4cf"
DEFAULT_FW = "b0d8535"
DEFAULT_PERIOD_US = 5000  # nominal 200 Hz ControlTask period
TELEMETRY_PACKET_PERIOD_US = 50000  # 20 Hz telemetry task period

SCENARIOS = ("normal", "fall", "loss", "saturation", "reboot", "diagnostic-first")


class PacketSequenceBuilder:
    """Stateful generator of schema-v1 packets with monotonic counters.

    Mirrors the firmware's telemetry_task semantics closely enough for
    testing: `seq`/`tick` advance once per emitted datagram, cumulative
    counters (`loop`, `dt_h`, `ovr`, `stale`, `read_fail`, `trunc`) only
    ever increase (until `reboot()` resets them), matching section 3.1's
    "additive monotonic telemetry counter" design.
    """

    def __init__(
        self,
        dev: str = DEFAULT_DEV,
        fw: str = DEFAULT_FW,
        start_seq: int = 0,
        start_tick: int = 0,
        start_loop: int = 0,
        start_t_us: int = 0,
        period_us: int = DEFAULT_PERIOD_US,
        samples_per_packet: int = 10,
    ):
        self.dev = dev
        self.fw = fw
        self.seq = start_seq
        self.tick = start_tick
        self.loop = start_loop
        self.t_us = start_t_us
        self.period_us = period_us
        self.samples_per_packet = samples_per_packet
        self.dt_h = [0] * 8
        self.ovr = 0
        self.stale = 0
        self.read_fail = 0
        self.trunc = 0
        # Counts reboot() calls (2026-07-06 review round 9): folded into the
        # post-reboot `t_us` baseline below so two different boot
        # generations of this builder never emit a byte-for-byte identical
        # packet at the same relative seq/tick offset. Real firmware's
        # `t_us` is microseconds since THAT boot; two independent boots
        # reaching the exact same "time since boot" for their Nth telemetry
        # packet is unrealistic (WiFi reconnect timing always jitters by
        # more than this). Without this, a short, field-static test session
        # can have its post-reboot packet(s) collide byte-for-byte with an
        # earlier pre-reboot packet at the same seq, which is exactly the
        # ambiguity analyzer._classify_records()'s duplicate-vs-reboot
        # `observed`-content check cannot resolve from content alone.
        self._reboot_count = 0

    def _advance_common(self) -> None:
        self.seq += 1
        self.tick += 1
        self.t_us += TELEMETRY_PACKET_PERIOD_US

    def full_packet(self, **overrides: Any) -> Dict[str, Any]:
        self._advance_common()
        self.loop += self.samples_per_packet
        self.dt_h[0] += self.samples_per_packet
        packet: Dict[str, Any] = {
            "v": 1,
            "seq": self.seq,
            "tick": self.tick,
            "snap_valid": True,
            "t_us": self.t_us,
            "dev": self.dev,
            "fw": self.fw,
            "fsm": 2,
            "fault": 0,
            "theta": 0.01,
            "theta_dot": 0.0,
            "theta_ref": 0.0,
            "v_body": 0.0,
            "om_l": 0.0,
            "om_r": 0.0,
            "i_cmd_l": 0.1,
            "i_cmd_r": 0.1,
            "i_mea_l": 0.1,
            "i_mea_r": 0.1,
            "dt_us": self.period_us,
            "dt_max_us": self.period_us,
            "loop": self.loop,
            "dt_h": list(self.dt_h),
            "ovr": self.ovr,
            "stale": self.stale,
            "read_fail": self.read_fail,
            "trunc": self.trunc,
            "volt": 7.4,
            "temp": 34.0,
            "sat": False,
            "i2t": False,
            "kp": 1.5,
            "ki": 0.0,
            "kd": 0.08,
            "eq": 0.0,
            "rssi": -55,
        }
        packet.update(overrides)
        return packet

    def diagnostic_packet(self, reason: str = "read_fail", **overrides: Any) -> Dict[str, Any]:
        self._advance_common()
        if reason == "read_fail":
            self.read_fail += 1
        elif reason == "trunc":
            self.trunc += 1
        packet: Dict[str, Any] = {
            "v": 1,
            "seq": self.seq,
            "tick": self.tick,
            "t_us": self.t_us,
            "dev": self.dev,
            "fw": self.fw,
            "snap_valid": False,
            "reason": reason,
            "read_fail": self.read_fail,
            "trunc": self.trunc,
        }
        packet.update(overrides)
        return packet

    def reboot(self) -> None:
        """Simulate a device restart: all counters reset near zero."""
        self._reboot_count += 1
        self.seq = 0
        self.tick = 0
        self.loop = 0
        # Not a plain 0: see the `_reboot_count` comment in __init__. Still
        # "near zero" (microsecond-scale) for every existing reboot-signal
        # assertion (t_us regression / "reset near zero" checks) -- this
        # only guarantees the post-reboot trajectory never numerically
        # replays an earlier boot's.
        self.t_us = self._reboot_count
        self.dt_h = [0] * 8
        self.ovr = 0
        self.stale = 0
        self.read_fail = 0
        self.trunc = 0


def generate_scenario(name: str, count: int = 40, **builder_kwargs: Any) -> List[Dict[str, Any]]:
    """Return a list of schema-v1 packet dicts for the named scenario.

    Scenarios (section 3.2): normal, fall, loss, saturation, reboot,
    diagnostic-first.
    """
    builder = PacketSequenceBuilder(**builder_kwargs)
    packets: List[Dict[str, Any]] = []

    if name == "normal":
        for _ in range(count):
            packets.append(builder.full_packet())

    elif name == "diagnostic-first":
        packets.append(builder.diagnostic_packet(reason="read_fail"))
        packets.append(builder.diagnostic_packet(reason="read_fail"))
        for _ in range(count):
            packets.append(builder.full_packet())

    elif name == "loss":
        drop_indices = {10, 11, 20}
        for i in range(count):
            packet = builder.full_packet()
            if i in drop_indices:
                continue  # simulated UDP loss: counters advanced, packet not sent
            packets.append(packet)

    elif name == "saturation":
        for i in range(count):
            sat = 15 <= i < 25
            packets.append(builder.full_packet(sat=sat, i2t=sat))

    elif name == "fall":
        half = count // 2
        for i in range(count):
            if i < half:
                packets.append(builder.full_packet())
            else:
                packets.append(builder.full_packet(fsm=4, fault=1))

    elif name == "reboot":
        half = count // 2
        for _ in range(half):
            packets.append(builder.full_packet())
        builder.reboot()
        for _ in range(count - half):
            packets.append(builder.full_packet())

    else:
        raise ValueError(f"unknown scenario: {name!r} (expected one of {SCENARIOS})")

    return packets


def send_packets(
    dest_host: str, dest_port: int, packets: List[Dict[str, Any]], interval_s: float = 0.0
) -> None:
    """Send pre-built packets over UDP, validating each against schema v1 first."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        for packet in packets:
            schema.validate_packet(packet)  # never emit a malformed packet
            sock.sendto(json.dumps(packet).encode("utf-8"), (dest_host, dest_port))
            if interval_s:
                time.sleep(interval_s)
    finally:
        sock.close()


def send_scenario(
    dest_host: str,
    dest_port: int,
    name: str,
    count: int = 40,
    interval_s: float = 0.0,
    **builder_kwargs: Any,
) -> List[Dict[str, Any]]:
    """Generate and send one scenario's packets. Returns the sent packets."""
    packets = generate_scenario(name, count=count, **builder_kwargs)
    send_packets(dest_host, dest_port, packets, interval_s=interval_s)
    return packets


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Send synthetic bsl-telemetry packets (Phase 1 simulator)")
    parser.add_argument("--host", default="127.0.0.1", help="destination host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, required=True, help="destination UDP port")
    parser.add_argument("--scenario", required=True, choices=sorted(SCENARIOS))
    parser.add_argument("--count", type=int, default=40, help="number of samples generated (default: 40)")
    parser.add_argument(
        "--interval", type=float, default=0.05, help="seconds between packets (default: 0.05 = 20 Hz)"
    )
    parser.add_argument("--dev", default=DEFAULT_DEV)
    parser.add_argument("--fw", default=DEFAULT_FW)
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    packets = send_scenario(
        args.host,
        args.port,
        args.scenario,
        count=args.count,
        interval_s=args.interval,
        dev=args.dev,
        fw=args.fw,
    )
    print(f"sent {len(packets)} packets to {args.host}:{args.port} (scenario={args.scenario})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
