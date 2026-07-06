"""Unit tests for bsl_telemetry.schema (packet schema v1 validation)."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bsl_telemetry import schema


def make_full_packet(**overrides):
    packet = {
        "v": 1,
        "seq": 1234,
        "tick": 1236,
        "snap_valid": True,
        "t_us": 987654321,
        "dev": "core2-a4cf",
        "fw": "b0d8535",
        "fsm": 2,
        "fault": 0,
        "theta": 0.012,
        "theta_dot": -0.34,
        "theta_ref": 0.002,
        "v_body": 0.05,
        "om_l": 3.1,
        "om_r": 3.0,
        "i_cmd_l": 0.12,
        "i_cmd_r": 0.11,
        "i_mea_l": 0.10,
        "i_mea_r": 0.09,
        "dt_us": 5012,
        "dt_max_us": 5480,
        "loop": 123456,
        "dt_h": [123000, 300, 80, 15, 4, 1, 0, 0],
        "ovr": 0,
        "stale": 0,
        "read_fail": 0,
        "trunc": 0,
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


def make_diagnostic_packet(**overrides):
    packet = {
        "v": 1,
        "seq": 42,
        "tick": 44,
        "t_us": 1000,
        "dev": "core2-a4cf",
        "fw": "b0d8535",
        "snap_valid": False,
        "reason": "read_fail",
        "read_fail": 3,
        "trunc": 0,
    }
    packet.update(overrides)
    return packet


class ValidateFullPacketTests(unittest.TestCase):
    def test_valid_full_packet_returns_full(self):
        self.assertEqual(schema.validate_packet(make_full_packet()), "full")

    def test_missing_required_field_raises(self):
        packet = make_full_packet()
        del packet["theta"]
        with self.assertRaises(schema.PacketValidationError) as ctx:
            schema.validate_packet(packet)
        self.assertEqual(ctx.exception.reason, "missing_field:theta")

    def test_wrong_type_raises_bad_type(self):
        packet = make_full_packet(seq="not-an-int")
        with self.assertRaises(schema.PacketValidationError) as ctx:
            schema.validate_packet(packet)
        self.assertEqual(ctx.exception.reason, "bad_type:seq")

    def test_bool_rejected_for_int_field(self):
        # bool is a subclass of int in Python; schema must reject it anyway.
        packet = make_full_packet(seq=True)
        with self.assertRaises(schema.PacketValidationError) as ctx:
            schema.validate_packet(packet)
        self.assertEqual(ctx.exception.reason, "bad_type:seq")

    def test_int_accepted_for_number_field(self):
        # theta is a "number" field; a bare JSON integer (e.g. 0) must be
        # accepted even though the example shows a float.
        packet = make_full_packet(theta=0, eq=1)
        self.assertEqual(schema.validate_packet(packet), "full")

    def test_dt_h_wrong_length_raises(self):
        packet = make_full_packet(dt_h=[1, 2, 3])
        with self.assertRaises(schema.PacketValidationError) as ctx:
            schema.validate_packet(packet)
        self.assertEqual(ctx.exception.reason, "bad_shape:dt_h")

    def test_dt_h_not_a_list_raises(self):
        packet = make_full_packet(dt_h="nope")
        with self.assertRaises(schema.PacketValidationError) as ctx:
            schema.validate_packet(packet)
        self.assertEqual(ctx.exception.reason, "bad_shape:dt_h")

    def test_dt_h_non_int_element_raises(self):
        packet = make_full_packet(dt_h=[1, 2, 3, 4, 5, 6, 7, 1.5])
        with self.assertRaises(schema.PacketValidationError) as ctx:
            schema.validate_packet(packet)
        self.assertEqual(ctx.exception.reason, "bad_type:dt_h[]")

    def test_unknown_extra_field_is_ignored(self):
        packet = make_full_packet(extra_future_field=123)
        self.assertEqual(schema.validate_packet(packet), "full")

    def test_bad_version_raises(self):
        packet = make_full_packet(v=2)
        with self.assertRaises(schema.PacketValidationError) as ctx:
            schema.validate_packet(packet)
        self.assertEqual(ctx.exception.reason, "bad_version:2")

    def test_not_a_dict_raises(self):
        with self.assertRaises(schema.PacketValidationError) as ctx:
            schema.validate_packet(["not", "a", "dict"])
        self.assertEqual(ctx.exception.reason, "not_object")


class ValidateDiagnosticPacketTests(unittest.TestCase):
    def test_valid_diagnostic_packet_returns_diagnostic(self):
        self.assertEqual(schema.validate_packet(make_diagnostic_packet()), "diagnostic")

    def test_valid_trunc_reason(self):
        packet = make_diagnostic_packet(reason="trunc", trunc=1)
        self.assertEqual(schema.validate_packet(packet), "diagnostic")

    def test_invalid_reason_value_raises(self):
        packet = make_diagnostic_packet(reason="something_else")
        with self.assertRaises(schema.PacketValidationError) as ctx:
            schema.validate_packet(packet)
        self.assertEqual(ctx.exception.reason, "bad_value:reason='something_else'")

    def test_missing_reason_raises(self):
        packet = make_diagnostic_packet()
        del packet["reason"]
        with self.assertRaises(schema.PacketValidationError) as ctx:
            schema.validate_packet(packet)
        self.assertEqual(ctx.exception.reason, "missing_field:reason")

    def test_diagnostic_does_not_require_full_only_fields(self):
        packet = make_diagnostic_packet()
        self.assertNotIn("dt_h", packet)
        self.assertEqual(schema.validate_packet(packet), "diagnostic")

    def test_missing_snap_valid_raises(self):
        packet = make_diagnostic_packet()
        del packet["snap_valid"]
        with self.assertRaises(schema.PacketValidationError) as ctx:
            schema.validate_packet(packet)
        self.assertEqual(ctx.exception.reason, "missing_field:snap_valid")

    def test_snap_valid_wrong_type_raises(self):
        packet = make_diagnostic_packet(snap_valid="false")
        with self.assertRaises(schema.PacketValidationError) as ctx:
            schema.validate_packet(packet)
        self.assertEqual(ctx.exception.reason, "bad_type:snap_valid")


class ClassifyPacketTests(unittest.TestCase):
    def test_classify_full(self):
        self.assertEqual(schema.classify_packet(make_full_packet()), "full")

    def test_classify_diagnostic(self):
        self.assertEqual(schema.classify_packet(make_diagnostic_packet()), "diagnostic")

    def test_classify_not_object_raises(self):
        with self.assertRaises(schema.PacketValidationError):
            schema.classify_packet(None)


class CommonFieldsTests(unittest.TestCase):
    def test_read_fail_and_trunc_are_common_to_both_variants(self):
        self.assertIn("read_fail", schema.COMMON_FIELDS)
        self.assertIn("trunc", schema.COMMON_FIELDS)
        self.assertIn("seq", schema.COMMON_FIELDS)
        self.assertIn("tick", schema.COMMON_FIELDS)
        self.assertIn("dev", schema.COMMON_FIELDS)
        self.assertIn("fw", schema.COMMON_FIELDS)


if __name__ == "__main__":
    unittest.main()
