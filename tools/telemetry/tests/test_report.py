"""Unit tests for bsl_telemetry.report, focused on the reboot-epoch-aware
raw.jsonl excerpt keying (指摘3).

See docs/plans/2026-07-05-udp-telemetry-phase1.md section 3.2 ("report").
"""

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bsl_telemetry import analyzer, report, simulator


def write_session(session_dir, packets, metadata_overrides=None):
    session_dir.mkdir(parents=True, exist_ok=True)
    with (session_dir / "raw.jsonl").open("w", encoding="utf-8") as handle:
        ns = 0
        for packet in packets:
            handle.write(json.dumps({"host_receive_ns": ns, "source_ip": "127.0.0.1", "packet": packet}) + "\n")
            ns += 50_000_000
    metadata = {
        "session_id": session_dir.name,
        "device_id": packets[0]["dev"],
        "firmware": packets[0]["fw"],
    }
    if metadata_overrides:
        metadata.update(metadata_overrides)
    with (session_dir / "metadata.json").open("w", encoding="utf-8") as handle:
        json.dump(metadata, handle)


def _excerpt_block(markdown: str, heading: str) -> str:
    """Return the ```jsonl ... ``` block immediately following `heading`."""
    start = markdown.index(heading)
    block_start = markdown.index("```jsonl", start) + len("```jsonl")
    block_end = markdown.index("```", block_start)
    return markdown[block_start:block_end]


class RebootEpochRawIndexTests(unittest.TestCase):
    def test_load_raw_index_by_epoch_seq_keeps_both_colliding_records(self):
        builder = simulator.PacketSequenceBuilder()
        packets = [builder.full_packet(sat=True) for _ in range(3)]  # seq 1,2,3
        builder.reboot()
        packets.extend(builder.full_packet(theta=99.0) for _ in range(3))  # seq 1,2,3 again

        with tempfile.TemporaryDirectory() as tmp:
            session_dir = Path(tmp) / "session"
            write_session(session_dir, packets)

            raw_index = report.load_raw_index_by_epoch_seq(session_dir / "raw.jsonl")
            # Both the pre-reboot (epoch 0) and post-reboot (epoch 1) record
            # for seq=1 must be present -- a flat seq-only dict would have
            # let the second overwrite the first.
            self.assertIn((0, 1), raw_index)
            self.assertIn((1, 1), raw_index)
            self.assertTrue(raw_index[(0, 1)]["packet"]["sat"])
            self.assertEqual(raw_index[(1, 1)]["packet"]["theta"], 99.0)

    def test_raw_excerpt_only_returns_matching_epoch(self):
        builder = simulator.PacketSequenceBuilder()
        packets = [builder.full_packet(sat=True) for _ in range(3)]
        builder.reboot()
        packets.extend(builder.full_packet(theta=99.0) for _ in range(3))

        with tempfile.TemporaryDirectory() as tmp:
            session_dir = Path(tmp) / "session"
            write_session(session_dir, packets)
            raw_index = report.load_raw_index_by_epoch_seq(session_dir / "raw.jsonl")

            pre_reboot_excerpt = report._raw_excerpt(raw_index, epoch=0, start_seq=1, end_seq=3, limit=20)
            self.assertEqual(len(pre_reboot_excerpt), 3)
            self.assertTrue(all(r["packet"]["sat"] for r in pre_reboot_excerpt))

            post_reboot_excerpt = report._raw_excerpt(raw_index, epoch=1, start_seq=1, end_seq=3, limit=20)
            self.assertEqual(len(post_reboot_excerpt), 3)
            self.assertTrue(all(r["packet"]["theta"] == 99.0 for r in post_reboot_excerpt))


class RenderReportRebootExcerptTests(unittest.TestCase):
    def test_pre_reboot_window_excerpt_not_overwritten_by_colliding_post_reboot_seq(self):
        builder = simulator.PacketSequenceBuilder()
        packets = [builder.full_packet(sat=True) for _ in range(3)]  # seq 1,2,3, all sat=True
        builder.reboot()
        # Post-reboot packets deliberately collide on seq with the
        # pre-reboot ones (1,2,3): firmware always restarts seq at 1 after a
        # reboot. theta=99.0 is a marker absent from the pre-reboot packets.
        packets.extend(builder.full_packet(theta=99.0) for _ in range(3))

        with tempfile.TemporaryDirectory() as tmp:
            session_dir = Path(tmp) / "session"
            write_session(session_dir, packets)
            analyzer.analyze_session(session_dir, write=True)
            markdown = report.render_report(session_dir)

        excerpt = _excerpt_block(markdown, "### saturation")
        # Every packet shown for the saturation window must be a PRE-reboot
        # one (sat=True), never a same-seq post-reboot packet (theta=99.0)
        # that happens to collide on seq (指摘3).
        self.assertIn('"sat": true', excerpt)
        self.assertNotIn("99.0", excerpt)

    def test_report_runs_without_reboot_and_omits_epoch_label(self):
        packets = simulator.generate_scenario("saturation", count=30)
        with tempfile.TemporaryDirectory() as tmp:
            session_dir = Path(tmp) / "session"
            write_session(session_dir, packets)
            analyzer.analyze_session(session_dir, write=True)
            markdown = report.render_report(session_dir)

        # Single-epoch (no reboot) sessions should not mention "epoch" at
        # all -- keeps the common-case report uncluttered.
        self.assertNotIn("epoch", markdown)


if __name__ == "__main__":
    unittest.main()
