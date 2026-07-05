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

    def test_same_seq_differing_content_after_early_reboot_is_reboot(self):
        """2026-07-05 review 指摘2: two back-to-back very-short-lived boots
        can land their first/only successfully-sent datagrams on the exact
        same seq value (e.g. seq=1 both times) without `t_us` ever going
        backward -- the second boot can take longer to reconnect, so its
        later-in-time `t_us` legitimately exceeds the first boot's very
        early `t_us` -- while `tick` (which also restarts at boot) is lower
        than the first boot's. The old predicate (seq-forward-delta-is-None
        OR t_us decreased) missed this and would have silently analyzed the
        reset post-reboot counters (loop, dt_h, ovr, stale) against the
        stale pre-reboot baseline.
        """
        builder = simulator.PacketSequenceBuilder()
        pre_reboot = builder.full_packet()  # seq=1, tick=1, t_us=50000, loop=10
        post_reboot = dict(pre_reboot)
        post_reboot["t_us"] = pre_reboot["t_us"] + 1  # higher: NOT a t_us regression
        post_reboot["tick"] = 0  # lower: tick counter restarted at boot
        post_reboot["loop"] = 0  # loop counter also restarted at boot
        self.assertEqual(post_reboot["seq"], pre_reboot["seq"])  # same seq, by construction
        self.assertNotEqual(post_reboot, pre_reboot)  # but NOT an exact duplicate

        records = wrap_records([pre_reboot, post_reboot])
        result = analyzer.analyze_records(records)
        self.assertEqual(result["reboot_count"], 1)
        reboot_events = [e for e in result["events"] if e["type"] == "reboot"]
        self.assertEqual(len(reboot_events), 1)
        self.assertEqual(reboot_events[0]["seq"], post_reboot["seq"])
        self.assertEqual(reboot_events[0]["epoch"], 1)

        # Must not be miscounted as ordinary seq loss (seq delta is 0).
        self.assertEqual(result["loss"]["gap_count"], 0)
        self.assertEqual(result["loss"]["lost_estimate"], 0)

        epochs = analyzer.assign_reboot_epochs(records)
        self.assertEqual(epochs, [0, 1])

    def test_exact_duplicate_packet_is_not_reboot(self):
        """2026-07-05 review 指摘2: a byte-for-byte UDP-level retransmission
        (same seq, same everything) must not be misclassified as a reboot
        just because its seq-forward-delta is 0, and must not be
        double-counted into variant/loss stats either."""
        builder = simulator.PacketSequenceBuilder()
        packets = [builder.full_packet() for _ in range(3)]
        duplicate = dict(packets[-1])  # exact retransmission of the last packet
        packets_with_dup = packets + [duplicate, builder.full_packet()]

        records = wrap_records(packets_with_dup)
        result = analyzer.analyze_records(records)
        self.assertEqual(result["reboot_count"], 0)
        self.assertEqual([e for e in result["events"] if e["type"] == "reboot"], [])
        self.assertEqual(result["loss"]["gap_count"], 0)
        self.assertEqual(result["loss"]["lost_estimate"], 0)
        # 5 raw records, but the duplicate must not be double-counted.
        self.assertEqual(result["packet_count"], 5)
        self.assertEqual(result["variant_counts"]["full"], 4)

        epochs = analyzer.assign_reboot_epochs(records)
        self.assertEqual(epochs, [0, 0, 0, 0, 0])

    def test_loop_rollback_with_forward_seq_tick_tus_is_reboot(self):
        """2026-07-05 review round 6: a reboot where the first post-reboot
        datagram(s) were lost can present seq/tick as *forward* (e.g. the
        short pre-reboot telemetry run only reached seq=1, and the first
        observed post-reboot packet is seq=2), and reconnect-time jitter can
        make the post-reboot t_us exceed the short pre-reboot run's t_us.
        The only remaining reboot signal is the `loop` (ControlTask cycle
        counter) rollback between the two full packets: the control loop had
        been running long before telemetry connected pre-reboot, so its
        counter was large, and restarts small after the reboot."""
        builder = simulator.PacketSequenceBuilder()
        pre_reboot = builder.full_packet()  # seq=1, tick=1
        pre_reboot["loop"] = 500_000  # control loop ran long before telemetry
        post_reboot = builder.full_packet()  # seq=2, tick=2 (forward!)
        post_reboot["t_us"] = pre_reboot["t_us"] + 10_000  # forward: reconnect jitter
        post_reboot["loop"] = 400  # rolled back: control task restarted

        records = wrap_records([pre_reboot, post_reboot])
        result = analyzer.analyze_records(records)
        self.assertEqual(result["reboot_count"], 1)
        reboot_events = [e for e in result["events"] if e["type"] == "reboot"]
        self.assertEqual(len(reboot_events), 1)
        # Must not be miscounted as seq loss.
        self.assertEqual(result["loss"]["gap_count"], 0)
        self.assertEqual(result["loss"]["lost_estimate"], 0)

        epochs = analyzer.assign_reboot_epochs(records)
        self.assertEqual(epochs, [0, 1])


class OutOfOrderRecoveryTests(unittest.TestCase):
    """2026-07-06 review 指摘1: raw.jsonl is in *receive* order, so ordinary
    UDP reordering (a delayed datagram arriving after a later one) must be
    recovered as out-of-order rather than misclassified as a reboot, and
    must not permanently inflate the loss estimate once the delayed
    datagram actually arrives."""

    def test_delayed_packet_recovered_not_reboot_and_not_counted_as_loss(self):
        # seq 1, 3, 2 (2 delayed): classic UDP reordering, nothing lost.
        builder = simulator.PacketSequenceBuilder()
        p1 = builder.full_packet()
        p2 = builder.full_packet()
        p3 = builder.full_packet()
        records = wrap_records([p1, p3, p2])

        result = analyzer.analyze_records(records)
        self.assertEqual(result["reboot_count"], 0)
        self.assertEqual([e for e in result["events"] if e["type"] == "reboot"], [])
        self.assertEqual(result["loss"]["lost_estimate"], 0)
        self.assertEqual(result["out_of_order_count"], 1)
        # The gap detected when seq 3 arrived (before seq 2 caught up) is
        # still reported as an informational window -- it really was
        # reordered, even though it turned out not to be lost -- but it
        # must not leave the final aggregate loss count inflated.
        self.assertEqual(result["loss"]["gap_count"], 1)
        self.assertEqual(result["variant_counts"]["full"], 3)

        epochs = analyzer.assign_reboot_epochs(records)
        self.assertEqual(epochs, [0, 0, 0])

    def test_permanent_gap_survives_alongside_unrelated_recovery(self):
        builder = simulator.PacketSequenceBuilder()
        p1 = builder.full_packet()  # seq 1
        builder.full_packet()  # seq 2: never arrives (permanent loss)
        p3 = builder.full_packet()  # seq 3
        p4 = builder.full_packet()  # seq 4: arrives late (recovered)
        p5 = builder.full_packet()  # seq 5

        # Arrival order: 1, 3, 5, 4 (4 delayed and recovered; 2 never shows).
        records = wrap_records([p1, p3, p5, p4])
        result = analyzer.analyze_records(records)

        self.assertEqual(result["reboot_count"], 0)
        self.assertEqual(result["out_of_order_count"], 1)
        # Only seq 2 remains unexplained: the delayed seq 4 must not mask it.
        self.assertEqual(result["loss"]["lost_estimate"], 1)
        loss_events = [e for e in result["events"] if e["type"] == "loss"]
        self.assertEqual(len(loss_events), 2)  # one gap noticed at seq 3, one at seq 5

    def test_out_of_order_recovery_does_not_pollute_interval_trackers(self):
        """The recovered (delayed) packet must not be used to compute
        dt_h/ovr/stale/loop interval deltas or become the new baseline --
        otherwise the *next* legitimately-ordered packet's delta would be
        computed against the wrong (smaller-seq) anchor."""
        builder = simulator.PacketSequenceBuilder(samples_per_packet=10)
        p1 = builder.full_packet()
        p2 = builder.full_packet()
        p3 = builder.full_packet()
        p4 = builder.full_packet()

        records = wrap_records([p1, p3, p2, p4])
        result = analyzer.analyze_records(records)

        self.assertEqual(result["reboot_count"], 0)
        self.assertEqual(result["out_of_order_count"], 1)
        # Intervals: 1->3 (delta 2, valid forward jump) and 3->4 (p2 is
        # skipped for interval purposes, and does not become the anchor).
        self.assertEqual(result["dt_histogram"]["intervals"], 2)

    def test_missing_set_cleared_on_reboot_prevents_false_recovery(self):
        """A gap left unexplained right before a reboot must not be
        "recovered" by a coincidentally-matching post-reboot seq -- the
        missing-seq set is per-epoch and must not survive a reboot
        boundary."""
        builder = simulator.PacketSequenceBuilder()
        p1 = builder.full_packet()  # seq 1
        builder.full_packet()  # seq 2: dropped, gap never explained pre-reboot
        p3 = builder.full_packet()  # seq 3

        builder.reboot()
        post1 = builder.full_packet()  # seq 1 again (post-reboot)
        post2 = builder.full_packet()  # seq 2 again (post-reboot, normal forward progress)

        records = wrap_records([p1, p3, post1, post2])
        result = analyzer.analyze_records(records)

        self.assertEqual(result["reboot_count"], 1)
        self.assertEqual(result["out_of_order_count"], 0)
        # The pre-reboot gap (seq 2) is never explained and stays lost.
        self.assertEqual(result["loss"]["lost_estimate"], 1)

    def test_existing_reboot_scenarios_unaffected_by_out_of_order_handling(self):
        """指摘1 regression guard: the reboot scenarios covered elsewhere in
        this file must keep reporting zero out-of-order packets."""
        packets = simulator.generate_scenario("reboot", count=10)
        result = analyzer.analyze_records(wrap_records(packets))
        self.assertEqual(result["reboot_count"], 1)
        self.assertEqual(result["out_of_order_count"], 0)


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


class RebootLeadInWindowTests(unittest.TestCase):
    """ゲート2レビュー(2回目)指摘3: a reboot event's own seq/epoch describe
    only the *post*-reboot side. Without a companion window anchored on
    prev_seq in the pre-reboot epoch, the (epoch, seq)-keyed raw excerpt
    (report.py, 指摘3 original) never shows the lead-in context right before
    the reboot."""

    def test_reboot_event_yields_pre_reboot_lead_in_window(self):
        events = [
            {
                "type": "reboot",
                "severity": "error",
                "epoch": 1,
                "seq": 1,
                "prev_seq": 120,
                "t_us": 500,
                "prev_t_us": 999,
            }
        ]
        windows = analyzer.build_recommended_windows(events, margin=5, limit=8)
        self.assertEqual(len(windows), 2)

        post = next(w for w in windows if w["label"] == "reboot")
        lead_in = next(w for w in windows if w["label"] == "reboot_lead_in")

        # Post-reboot window: anchored on the new epoch's seq (1).
        self.assertEqual(post["epoch"], 1)
        self.assertEqual(post["start_seq"], 0)  # max(0, 1-5), clamped
        self.assertEqual(post["end_seq"], 6)

        # Lead-in window: anchored on prev_seq (120) in the PREVIOUS epoch.
        self.assertEqual(lead_in["epoch"], 0)
        self.assertEqual(lead_in["start_seq"], 115)
        self.assertEqual(lead_in["end_seq"], 125)

        # Both halves of a reboot should be looked at first (equal priority
        # to the plain "reboot" label), ahead of lower-priority event types.
        self.assertLess(windows.index(post), 2)
        self.assertLess(windows.index(lead_in), 2)

    def test_synthetic_reboot_without_epoch_field_yields_single_window(self):
        # Regression guard: a hand-crafted reboot event lacking an "epoch"
        # key (as used elsewhere in this module, e.g.
        # test_reboot_and_stall_outrank_loss_and_saturation) has no reliable
        # pre-reboot epoch to anchor a lead-in window on, so it must keep
        # yielding exactly the one (post-reboot-shaped) window as before.
        events = [{"type": "reboot", "severity": "error", "seq": 50, "prev_seq": 49, "t_us": 0, "prev_t_us": 100}]
        windows = analyzer.build_recommended_windows(events, margin=2, limit=8)
        self.assertEqual(len(windows), 1)
        self.assertEqual(windows[0]["label"], "reboot")


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
