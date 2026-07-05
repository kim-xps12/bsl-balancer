"""Unit tests for bsl_telemetry.analyzer (pure raw.jsonl -> events/summary)."""

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bsl_telemetry import analyzer, simulator


def wrap_records(packets, source_ip="127.0.0.1", start_ns=0, step_ns=50_000_000):
    records = []
    ns = start_ns
    for packet in packets:
        records.append({"host_receive_ns": ns, "source_ip": source_ip, "packet": packet})
        ns += step_ns
    return records


class SeqLossTests(unittest.TestCase):
    def test_no_gap_no_loss(self):
        packets = simulator.generate_scenario("normal", count=10)
        result = analyzer.analyze_records(wrap_records(packets))
        self.assertEqual(result["loss"]["lost_estimate"], 0)
        self.assertEqual(result["loss"]["gap_count"], 0)

    def test_seq_gap_detected_as_loss(self):
        packets = simulator.generate_scenario("normal", count=10)
        # Drop packets at index 3 and 4 (two consecutive datagrams lost).
        del packets[3:5]
        result = analyzer.analyze_records(wrap_records(packets))
        self.assertEqual(result["loss"]["lost_estimate"], 2)
        self.assertEqual(result["loss"]["gap_count"], 1)
        loss_events = [e for e in result["events"] if e["type"] == "loss"]
        self.assertEqual(len(loss_events), 1)
        self.assertEqual(loss_events[0]["lost_packets"], 2)

    def test_generated_loss_scenario_reports_three_gaps(self):
        packets = simulator.generate_scenario("loss", count=30)
        result = analyzer.analyze_records(wrap_records(packets))
        # simulator drops indices {10, 11, 20}: that is two separate gaps
        # (10,11 are contiguous -> one 2-packet gap; 20 is a separate 1-packet gap)
        self.assertEqual(result["loss"]["gap_count"], 2)
        self.assertEqual(result["loss"]["lost_estimate"], 3)


class RebootReanchorTests(unittest.TestCase):
    def test_reboot_detected_and_counters_reanchored(self):
        packets = simulator.generate_scenario("reboot", count=10)
        result = analyzer.analyze_records(wrap_records(packets))
        self.assertEqual(result["reboot_count"], 1)
        reboot_events = [e for e in result["events"] if e["type"] == "reboot"]
        self.assertEqual(len(reboot_events), 1)
        # prev_seq was large (pre-reboot), new seq is small (post-reboot).
        self.assertGreater(reboot_events[0]["prev_seq"], reboot_events[0]["seq"])

        # The reboot must not be miscounted as ordinary seq loss, and must
        # not poison the dt_h/loop delta accounting with a huge bogus jump.
        self.assertEqual(result["loss"]["gap_count"], 0)
        self.assertEqual(result["loss"]["lost_estimate"], 0)
        # 10 packets, 1 reboot boundary -> 8 valid consecutive-full-packet
        # intervals contribute to the histogram (4 pre-reboot + 4 post-reboot).
        self.assertEqual(result["dt_histogram"]["intervals"], 8)

    def test_backward_seq_without_wraparound_is_reboot_not_loss(self):
        builder = simulator.PacketSequenceBuilder()
        # A single seq transition from a large value straight down to a
        # small one (no legitimate uint32 wraparound context) must be
        # classified as a reboot, not as a (huge, bogus) loss gap.
        large_seq_packet = builder.full_packet()
        large_seq_packet["seq"] = 50000
        small_seq_packet = builder.full_packet()
        small_seq_packet["seq"] = 3
        result = analyzer.analyze_records(wrap_records([large_seq_packet, small_seq_packet]))
        self.assertEqual(result["reboot_count"], 1)
        self.assertEqual(result["loss"]["gap_count"], 0)
        self.assertEqual(result["loss"]["lost_estimate"], 0)


class RebootOpenRegionTests(unittest.TestCase):
    """指摘1: a saturation/i2t/control_task_stall region still open at the
    moment of a reboot must be closed and reported (using the pre-reboot
    baseline) instead of silently discarded by the baseline reset."""

    def test_saturation_region_open_at_reboot_is_closed_not_dropped(self):
        builder = simulator.PacketSequenceBuilder()
        packets = [builder.full_packet(sat=True) for _ in range(3)]  # seq 1,2,3, still open
        pre_reboot_start_seq = packets[0]["seq"]
        pre_reboot_end_seq = packets[-1]["seq"]

        builder.reboot()
        # Post-reboot seq restarts at 1, colliding with the pre-reboot seq
        # values above; sat is back to the default False.
        packets.extend(builder.full_packet() for _ in range(2))

        result = analyzer.analyze_records(wrap_records(packets))
        self.assertEqual(result["reboot_count"], 1)
        sat_events = [e for e in result["events"] if e["type"] == "saturation"]
        # The pre-reboot saturation region must be reported, not merged into
        # a bogus cross-reboot span and not silently dropped.
        self.assertEqual(len(sat_events), 1)
        self.assertEqual(sat_events[0]["start_seq"], pre_reboot_start_seq)
        self.assertEqual(sat_events[0]["end_seq"], pre_reboot_end_seq)

    def test_control_task_stall_open_at_reboot_is_closed_not_dropped(self):
        builder = simulator.PacketSequenceBuilder(samples_per_packet=10)
        packets = [builder.full_packet() for _ in range(2)]
        pre_stall_seq = packets[-1]["seq"]  # last packet before loop progress stops
        stalled_loop = packets[-1]["loop"]

        stalled = builder.full_packet()
        stalled["loop"] = stalled_loop  # ControlTask stall begins, still open at reboot
        packets.append(stalled)
        stalled_seq = stalled["seq"]

        builder.reboot()
        packets.extend(builder.full_packet() for _ in range(2))  # post-reboot resumes normally

        result = analyzer.analyze_records(wrap_records(packets))
        self.assertEqual(result["reboot_count"], 1)
        stalls = [e for e in result["events"] if e["type"] == "control_task_stall"]
        self.assertEqual(len(stalls), 1)
        self.assertEqual(stalls[0]["start_seq"], pre_stall_seq)
        self.assertEqual(stalls[0]["end_seq"], stalled_seq)


class RebootEventOrderingTests(unittest.TestCase):
    """指摘2: firmware `seq` restarts at 1 after a reboot while the receiver
    keeps appending to the same session, so events must sort by reboot
    epoch first (falling back to seq only within an epoch) -- not by raw
    `seq` alone, which would put a post-reboot event with a small seq ahead
    of a pre-reboot event with a larger seq."""

    def test_events_ordered_by_reboot_epoch_not_raw_seq(self):
        builder = simulator.PacketSequenceBuilder(fw="aaaaaaa")
        packets = [builder.full_packet() for _ in range(3)]  # seq 1,2,3
        packets.append(builder.full_packet(sat=True))  # seq 4, sat opens (still open at reboot)
        pre_reboot_open_seq = packets[-1]["seq"]

        builder.reboot()
        # Post-reboot firmware differs -> firmware_change fires on the very
        # first post-reboot packet, whose seq (1) is numerically smaller
        # than the pre-reboot saturation region's seq (4) above.
        packets.append(builder.full_packet(fw="bbbbbbb"))
        post_reboot_small_seq = packets[-1]["seq"]
        self.assertLess(post_reboot_small_seq, pre_reboot_open_seq)

        result = analyzer.analyze_records(wrap_records(packets))
        events = result["events"]
        sat_idx = next(i for i, e in enumerate(events) if e["type"] == "saturation")
        reboot_idx = next(i for i, e in enumerate(events) if e["type"] == "reboot")
        fw_change_idx = next(i for i, e in enumerate(events) if e["type"] == "firmware_change")

        # A naive seq-only sort would put firmware_change (seq=1) first.
        # The actual chronological order is: pre-reboot saturation, then the
        # reboot itself, then the post-reboot firmware_change.
        self.assertLess(sat_idx, reboot_idx)
        self.assertLess(reboot_idx, fw_change_idx)
        self.assertEqual(events[sat_idx]["epoch"], 0)
        self.assertEqual(events[reboot_idx]["epoch"], 1)
        self.assertEqual(events[fw_change_idx]["epoch"], 1)

    def test_assign_reboot_epochs_matches_analyze_records_reboot_count(self):
        packets = simulator.generate_scenario("reboot", count=10)
        records = wrap_records(packets)
        epochs = analyzer.assign_reboot_epochs(records)
        result = analyzer.analyze_records(records)
        self.assertEqual(epochs[-1], result["reboot_count"])
        self.assertEqual(epochs[0], 0)


class DtHistogramOvrStaleTests(unittest.TestCase):
    def test_dt_h_ovr_stale_diffs_reconstructed_across_interval(self):
        builder = simulator.PacketSequenceBuilder(samples_per_packet=10)
        packets = []
        for _ in range(3):
            packets.append(builder.full_packet())
        # Manually bump ovr/stale/dt_h bin 3 between packet 3 and packet 4,
        # simulating jitter observed only in cumulative counters.
        builder.ovr += 2
        builder.stale += 1
        builder.dt_h[3] += 5
        packets.append(builder.full_packet())

        result = analyzer.analyze_records(wrap_records(packets))
        self.assertEqual(result["overrun_total_delta"], 2)
        self.assertEqual(result["imu_stale_total_delta"], 1)
        self.assertEqual(result["dt_histogram"]["totals"][3], 5)
        # 4 packets -> 3 intervals, each contributing samples_per_packet to bin 0.
        self.assertEqual(result["dt_histogram"]["totals"][0], 10 * 3)
        self.assertEqual(result["dt_histogram"]["intervals"], 3)

    def test_read_fail_trunc_diff_spans_diagnostic_and_full_packets(self):
        builder = simulator.PacketSequenceBuilder()
        packets = [
            builder.full_packet(),
            builder.diagnostic_packet(reason="read_fail"),
            builder.diagnostic_packet(reason="read_fail"),
            builder.full_packet(),
        ]
        result = analyzer.analyze_records(wrap_records(packets))
        self.assertEqual(result["read_fail_total_delta"], 2)
        self.assertEqual(result["diagnostic_by_reason"]["read_fail"], 2)


class LoopStallTests(unittest.TestCase):
    def test_zero_loop_delta_run_reported_as_stall(self):
        builder = simulator.PacketSequenceBuilder(samples_per_packet=10)
        packets = [builder.full_packet() for _ in range(2)]
        stalled_loop = packets[-1]["loop"]
        for _ in range(3):
            stalled = builder.full_packet()
            stalled["loop"] = stalled_loop  # ControlTask stalled: no progress
            packets.append(stalled)
        packets.append(builder.full_packet())  # resumes

        result = analyzer.analyze_records(wrap_records(packets))
        stalls = [e for e in result["events"] if e["type"] == "control_task_stall"]
        self.assertEqual(len(stalls), 1)
        self.assertEqual(stalls[0]["start_seq"], packets[1]["seq"])
        self.assertEqual(stalls[0]["end_seq"], packets[4]["seq"])


class TransitionAndRegionTests(unittest.TestCase):
    def test_fsm_and_fault_transition_events(self):
        packets = simulator.generate_scenario("fall", count=10)
        result = analyzer.analyze_records(wrap_records(packets))
        fsm_events = [e for e in result["events"] if e["type"] == "fsm_transition"]
        fault_events = [e for e in result["events"] if e["type"] == "fault_transition"]
        self.assertEqual(len(fsm_events), 1)
        self.assertEqual(fsm_events[0]["from"], 2)
        self.assertEqual(fsm_events[0]["to"], 4)
        self.assertEqual(len(fault_events), 1)
        self.assertEqual(fault_events[0]["to"], 1)

    def test_saturation_region_detected(self):
        packets = simulator.generate_scenario("saturation", count=30)
        result = analyzer.analyze_records(wrap_records(packets))
        sat_events = [e for e in result["events"] if e["type"] == "saturation"]
        self.assertEqual(len(sat_events), 1)
        self.assertEqual(sat_events[0]["start_seq"], packets[15]["seq"])
        self.assertEqual(sat_events[0]["end_seq"], packets[24]["seq"])

    def test_region_still_open_at_end_is_closed(self):
        builder = simulator.PacketSequenceBuilder()
        packets = [builder.full_packet(sat=True) for _ in range(3)]
        result = analyzer.analyze_records(wrap_records(packets))
        sat_events = [e for e in result["events"] if e["type"] == "saturation"]
        self.assertEqual(len(sat_events), 1)
        self.assertEqual(sat_events[0]["end_seq"], packets[-1]["seq"])


class DeviceFirmwareChangeTests(unittest.TestCase):
    def test_device_change_mid_session_is_reported(self):
        builder = simulator.PacketSequenceBuilder(dev="core2-a4cf")
        packets = [builder.full_packet() for _ in range(2)]
        intruder = simulator.PacketSequenceBuilder(dev="core2-ffff", start_seq=2, start_tick=2, start_t_us=100000)
        packets.append(intruder.full_packet())

        result = analyzer.analyze_records(wrap_records(packets))
        changes = [e for e in result["events"] if e["type"] == "device_change"]
        self.assertEqual(len(changes), 1)
        self.assertEqual(changes[0]["from"], "core2-a4cf")
        self.assertEqual(changes[0]["to"], "core2-ffff")

    def test_firmware_change_mid_session_is_reported(self):
        builder = simulator.PacketSequenceBuilder(fw="aaaaaaa")
        packets = [builder.full_packet() for _ in range(2)]
        packets.append(builder.full_packet(fw="bbbbbbb"))

        result = analyzer.analyze_records(wrap_records(packets))
        changes = [e for e in result["events"] if e["type"] == "firmware_change"]
        self.assertEqual(len(changes), 1)
        self.assertEqual(changes[0]["from"], "aaaaaaa")
        self.assertEqual(changes[0]["to"], "bbbbbbb")

    def test_no_change_no_event(self):
        packets = simulator.generate_scenario("normal", count=5)
        result = analyzer.analyze_records(wrap_records(packets))
        changes = [e for e in result["events"] if e["type"] in ("device_change", "firmware_change")]
        self.assertEqual(changes, [])


class DiagnosticFirstTests(unittest.TestCase):
    def test_first_packet_diagnostic_does_not_crash_and_is_counted(self):
        packets = simulator.generate_scenario("diagnostic-first", count=5)
        result = analyzer.analyze_records(wrap_records(packets))
        self.assertEqual(result["variant_counts"]["diagnostic"], 2)
        self.assertEqual(result["variant_counts"]["full"], 5)
        self.assertEqual(result["packet_count"], 7)


class RecommendedWindowsTests(unittest.TestCase):
    def test_reboot_and_stall_outrank_loss_and_saturation(self):
        events = [
            {"type": "loss", "severity": "warn", "start_seq": 100, "end_seq": 103, "lost_packets": 3},
            {"type": "saturation", "severity": "warn", "start_seq": 10, "end_seq": 20},
            {"type": "reboot", "severity": "error", "seq": 50, "prev_seq": 49, "t_us": 0, "prev_t_us": 100},
            {"type": "control_task_stall", "severity": "error", "start_seq": 60, "end_seq": 65},
        ]
        windows = analyzer.build_recommended_windows(events, margin=2, limit=8)
        labels = [w["label"] for w in windows]
        self.assertEqual(labels[0], "reboot")
        self.assertEqual(labels[1], "control_task_stall")
        self.assertIn("loss", labels)
        self.assertIn("saturation", labels)

    def test_margin_applied_and_clamped_at_zero(self):
        events = [{"type": "loss", "severity": "warn", "start_seq": 1, "end_seq": 3, "lost_packets": 2}]
        windows = analyzer.build_recommended_windows(events, margin=5, limit=8)
        self.assertEqual(windows[0]["start_seq"], 0)  # clamped, not negative
        self.assertEqual(windows[0]["end_seq"], 8)

    def test_fault_recovery_transition_excluded(self):
        events = [
            {"type": "fault_transition", "severity": "info", "seq": 10, "from": 1, "to": 0},
            {"type": "fault_transition", "severity": "warn", "seq": 20, "from": 0, "to": 2},
        ]
        windows = analyzer.build_recommended_windows(events)
        self.assertEqual(len(windows), 1)
        self.assertEqual(windows[0]["start_seq"], 15)


class AnalyzeSessionTests(unittest.TestCase):
    def test_analyze_session_writes_events_and_summary_using_metadata(self):
        with tempfile.TemporaryDirectory() as tmp:
            session_dir = Path(tmp) / "20260705-120000_core2-a4cf_run-001"
            session_dir.mkdir()

            packets = simulator.generate_scenario("normal", count=5)
            with (session_dir / "raw.jsonl").open("w", encoding="utf-8") as handle:
                for record in wrap_records(packets):
                    handle.write(json.dumps(record) + "\n")

            metadata = {
                "session_id": session_dir.name,
                "device_id": "core2-a4cf",
                "firmware": "b0d8535",
                "started_at": "2026-07-05T12:00:00+09:00",
                "ended_at": "2026-07-05T12:00:10+09:00",
                "rejected_packets": {"total": 2, "by_source": {"10.0.0.9": 2}, "by_reason": {"ip_pinning_mismatch": 2}},
            }
            with (session_dir / "metadata.json").open("w", encoding="utf-8") as handle:
                json.dump(metadata, handle)

            result = analyzer.analyze_session(session_dir, write=True)

            self.assertTrue((session_dir / "events.jsonl").exists())
            self.assertTrue((session_dir / "summary.json").exists())

            summary = json.loads((session_dir / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual(summary["session_id"], session_dir.name)
            self.assertEqual(summary["device_id"], "core2-a4cf")
            self.assertEqual(summary["packet_count"], 5)
            self.assertEqual(summary["rejected_packets"]["total"], 2)
            self.assertIn("recommended_windows", summary)
            self.assertEqual(result["summary"]["packet_count"], 5)

    def test_analyze_session_defaults_rejected_packets_when_metadata_missing(self):
        with tempfile.TemporaryDirectory() as tmp:
            session_dir = Path(tmp) / "session-no-metadata"
            session_dir.mkdir()
            packets = simulator.generate_scenario("normal", count=2)
            with (session_dir / "raw.jsonl").open("w", encoding="utf-8") as handle:
                for record in wrap_records(packets):
                    handle.write(json.dumps(record) + "\n")

            result = analyzer.analyze_session(session_dir, write=False)
            self.assertEqual(result["summary"]["rejected_packets"], {"total": 0, "by_source": {}, "by_reason": {}})


if __name__ == "__main__":
    unittest.main()
