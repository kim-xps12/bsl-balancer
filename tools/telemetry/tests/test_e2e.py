"""End-to-end tests: simulator -> receiver -> analyzer -> report over a real
loopback UDP socket (ephemeral port, no fixed port collisions).

Covers the specific behaviors called out in
docs/plans/2026-07-05-udp-telemetry-phase1.md section 3.2/5:
  - session dir / metadata.json / raw.jsonl / events.jsonl / summary.json creation
  - a session can start from a diagnostic-first packet
  - source-IP pinning rejection is tallied (not silently dropped)
  - device reboot re-anchoring is visible end-to-end in summary.json

Note on IP pinning coverage: this sandboxed test environment cannot bind a
UDP socket to a second loopback alias (e.g. 127.0.0.2) without a one-time
`ifconfig lo0 alias` that requires privileges we don't assume are available,
so the "packet arrives from an unexpected source after a session already
started" branch is exercised via a direct call to the receiver's internal
`_handle_datagram(data, source_ip)` with a synthetic source_ip string
(bypassing the OS socket layer only for that one call). The "explicit
--device-ip never matches the sender" branch is exercised fully over the
real socket, since that only requires a single (real) sender address.
"""

import json
import shutil
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bsl_telemetry import receiver as receiver_mod
from bsl_telemetry import report, simulator

# Fast polling so idle-timeout-based session close doesn't make the suite slow.
POLL_INTERVAL_S = 0.02
IDLE_TIMEOUT_S = 0.3
CLOSE_WAIT_S = IDLE_TIMEOUT_S + 1.0


class ReceiverTestHarness:
    """Runs a TelemetryReceiver on a background thread against a temp log root."""

    def __init__(self, device_ip=None, idle_timeout_s=IDLE_TIMEOUT_S):
        self.tmp_dir = tempfile.mkdtemp(prefix="bsl_telemetry_e2e_")
        self.log_root = Path(self.tmp_dir) / "telemetry"
        self.receiver = receiver_mod.TelemetryReceiver(
            port=0,
            log_root=self.log_root,
            device_ip=device_ip,
            idle_timeout_s=idle_timeout_s,
            poll_interval_s=POLL_INTERVAL_S,
        )
        self.thread = threading.Thread(target=self.receiver.serve_forever, daemon=True)
        self.thread.start()

    @property
    def port(self):
        return self.receiver.bound_port

    def stop_and_join(self, timeout=2.0):
        self.receiver.request_stop()
        self.thread.join(timeout=timeout)

    def cleanup(self):
        shutil.rmtree(self.tmp_dir, ignore_errors=True)


class SessionLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.harness = ReceiverTestHarness()
        self.addCleanup(self.harness.cleanup)

    def test_normal_scenario_produces_full_session_artifacts(self):
        sent = simulator.send_scenario("127.0.0.1", self.harness.port, "normal", count=20, interval_s=0.0)
        time.sleep(CLOSE_WAIT_S)
        self.harness.stop_and_join()

        self.assertEqual(len(self.harness.receiver.closed_sessions), 1)
        session_dir = self.harness.receiver.closed_sessions[0]

        for name in ("metadata.json", "raw.jsonl", "events.jsonl", "summary.json"):
            self.assertTrue((session_dir / name).exists(), f"missing {name}")

        metadata = json.loads((session_dir / "metadata.json").read_text(encoding="utf-8"))
        self.assertEqual(metadata["device_id"], "core2-a4cf")
        self.assertEqual(metadata["firmware"], "b0d8535")
        self.assertFalse(metadata["authenticated"])
        self.assertEqual(metadata["packet_count"], len(sent))

        summary = json.loads((session_dir / "summary.json").read_text(encoding="utf-8"))
        self.assertEqual(summary["packet_count"], len(sent))
        self.assertEqual(summary["loss"]["lost_estimate"], 0)

        raw_lines = (session_dir / "raw.jsonl").read_text(encoding="utf-8").strip().splitlines()
        self.assertEqual(len(raw_lines), len(sent))
        first_record = json.loads(raw_lines[0])
        self.assertIn("host_receive_ns", first_record)
        self.assertEqual(first_record["source_ip"], "127.0.0.1")
        self.assertEqual(first_record["packet"]["seq"], 1)

        # report.py must run against the produced session without raising.
        markdown = report.render_report(session_dir)
        self.assertIn("Telemetry session report", markdown)
        self.assertIn("core2-a4cf", markdown)

    def test_first_packet_diagnostic_starts_session(self):
        sent = simulator.send_scenario(
            "127.0.0.1", self.harness.port, "diagnostic-first", count=10, interval_s=0.0
        )
        time.sleep(CLOSE_WAIT_S)
        self.harness.stop_and_join()

        self.assertEqual(len(self.harness.receiver.closed_sessions), 1)
        session_dir = self.harness.receiver.closed_sessions[0]

        metadata = json.loads((session_dir / "metadata.json").read_text(encoding="utf-8"))
        self.assertEqual(metadata["packet_count"], len(sent))

        raw_lines = (session_dir / "raw.jsonl").read_text(encoding="utf-8").strip().splitlines()
        first_packet = json.loads(raw_lines[0])["packet"]
        self.assertFalse(first_packet["snap_valid"])
        self.assertEqual(first_packet["reason"], "read_fail")

        summary = json.loads((session_dir / "summary.json").read_text(encoding="utf-8"))
        self.assertEqual(summary["variant_counts"]["diagnostic"], 2)
        self.assertEqual(summary["variant_counts"]["full"], 10)

    def test_reboot_scenario_is_reanchored_end_to_end(self):
        simulator.send_scenario("127.0.0.1", self.harness.port, "reboot", count=20, interval_s=0.0)
        time.sleep(CLOSE_WAIT_S)
        self.harness.stop_and_join()

        session_dir = self.harness.receiver.closed_sessions[0]
        summary = json.loads((session_dir / "summary.json").read_text(encoding="utf-8"))
        events = [json.loads(line) for line in (session_dir / "events.jsonl").read_text(encoding="utf-8").splitlines()]

        self.assertEqual(summary["reboot_count"], 1)
        reboot_events = [e for e in events if e["type"] == "reboot"]
        self.assertEqual(len(reboot_events), 1)
        self.assertGreater(reboot_events[0]["prev_seq"], reboot_events[0]["seq"])
        # The reboot boundary must not be miscounted as UDP loss.
        self.assertEqual(summary["loss"]["gap_count"], 0)


class IpPinningRejectionTests(unittest.TestCase):
    def test_explicit_device_ip_mismatch_rejects_and_tallies_without_starting_session(self):
        harness = ReceiverTestHarness(device_ip="203.0.113.5")
        self.addCleanup(harness.cleanup)

        simulator.send_scenario("127.0.0.1", harness.port, "normal", count=5, interval_s=0.0)
        time.sleep(CLOSE_WAIT_S)
        harness.stop_and_join()

        self.assertEqual(harness.receiver.closed_sessions, [])
        rejected = harness.receiver._pre_session_rejected  # no session ever started
        self.assertEqual(rejected["total"], 5)
        self.assertEqual(rejected["by_source"].get("127.0.0.1"), 5)
        self.assertEqual(rejected["by_reason"].get("ip_pinning_mismatch"), 5)

    def test_mismatched_source_after_session_start_is_rejected_and_tallied(self):
        harness = ReceiverTestHarness()
        self.addCleanup(harness.cleanup)

        # Start a session normally from the real loopback sender.
        simulator.send_scenario("127.0.0.1", harness.port, "normal", count=3, interval_s=0.01)
        time.sleep(0.2)
        self.assertIsNotNone(harness.receiver.session)
        packet_count_before = harness.receiver.session.packet_count

        # Simulate a stray packet from a different source IP while the
        # session is active (see module docstring for why this bypasses the
        # socket layer). The receiver must reject it without disturbing the
        # active session's raw.jsonl.
        spoofed_packet = simulator.PacketSequenceBuilder().full_packet()
        harness.receiver._handle_datagram(json.dumps(spoofed_packet).encode("utf-8"), "203.0.113.9")

        self.assertEqual(harness.receiver.session.packet_count, packet_count_before)
        self.assertEqual(harness.receiver.session.rejected["total"], 1)
        self.assertEqual(harness.receiver.session.rejected["by_reason"].get("ip_pinning_mismatch"), 1)
        self.assertEqual(harness.receiver.session.rejected["by_source"].get("203.0.113.9"), 1)

        time.sleep(CLOSE_WAIT_S)
        harness.stop_and_join()

        session_dir = harness.receiver.closed_sessions[0]
        summary = json.loads((session_dir / "summary.json").read_text(encoding="utf-8"))
        self.assertEqual(summary["rejected_packets"]["total"], 1)
        self.assertEqual(summary["rejected_packets"]["by_source"].get("203.0.113.9"), 1)


class InvalidPacketRejectionTests(unittest.TestCase):
    def test_malformed_json_is_rejected_and_does_not_start_session(self):
        harness = ReceiverTestHarness()
        self.addCleanup(harness.cleanup)

        import socket

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto(b"{not valid json", ("127.0.0.1", harness.port))
        finally:
            sock.close()

        time.sleep(CLOSE_WAIT_S)
        harness.stop_and_join()

        self.assertEqual(harness.receiver.closed_sessions, [])
        rejected = harness.receiver._pre_session_rejected
        self.assertEqual(rejected["total"], 1)
        self.assertIn("json_error", rejected["by_reason"])


if __name__ == "__main__":
    unittest.main()
