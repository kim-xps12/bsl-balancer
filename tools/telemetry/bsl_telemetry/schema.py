"""BSL telemetry packet schema v1 (definition + validation).

Standard-library only. See docs/plans/2026-07-05-udp-telemetry-phase1.md
section 3.1 for the authoritative field list. Schema v1 defines exactly two
wire variants, discriminated by the ``snap_valid`` field:

- ``full``       (``snap_valid: true``): a complete Snapshot-derived sample.
- ``diagnostic`` (``snap_valid: false``): emitted when the telemetry task
  failed to obtain a fresh seqlock read (``reason: "read_fail"``) or when the
  formatted JSON line would have exceeded the send buffer
  (``reason: "trunc"``).

This module only validates *shape* (required keys + JSON types). It does not
interpret field semantics (that is analyzer.py's job) and it deliberately
allows unknown extra keys so that additive, backward-compatible firmware
changes do not break older PC-side tooling.
"""

from __future__ import annotations

from typing import Any, Dict, Literal

SCHEMA_VERSION = 1

DT_H_LEN = 8

# reason codes carried by diagnostic packets (packet schema v1, section 3.1).
DIAGNOSTIC_REASONS = ("read_fail", "trunc")

PacketVariant = Literal["full", "diagnostic"]

# Field name -> JSON "kind" required. Kinds:
#   "int"    : JSON integer (bool excluded even though bool is an int subclass)
#   "number" : JSON integer or float (bool excluded)
#   "bool"   : JSON true/false
#   "str"    : JSON string
FULL_SCALAR_FIELDS: Dict[str, str] = {
    "v": "int",
    "seq": "int",
    "tick": "int",
    "snap_valid": "bool",
    "t_us": "int",
    "dev": "str",
    "fw": "str",
    "fsm": "int",
    "fault": "int",
    "theta": "number",
    "theta_dot": "number",
    "theta_ref": "number",
    "v_body": "number",
    "om_l": "number",
    "om_r": "number",
    "i_cmd_l": "number",
    "i_cmd_r": "number",
    "i_mea_l": "number",
    "i_mea_r": "number",
    "dt_us": "int",
    "dt_max_us": "int",
    "loop": "int",
    "ovr": "int",
    "stale": "int",
    "read_fail": "int",
    "trunc": "int",
    "volt": "number",
    "temp": "number",
    "sat": "bool",
    "i2t": "bool",
    "kp": "number",
    "ki": "number",
    "kd": "number",
    "eq": "number",
    "rssi": "int",
}
# "dt_h" is validated separately (fixed-length int array), see _validate_dt_h.
FULL_ARRAY_FIELDS = ("dt_h",)

DIAGNOSTIC_SCALAR_FIELDS: Dict[str, str] = {
    "v": "int",
    "seq": "int",
    "tick": "int",
    "t_us": "int",
    "dev": "str",
    "fw": "str",
    "snap_valid": "bool",
    "reason": "str",
    "read_fail": "int",
    "trunc": "int",
}

# Fields present verbatim in both variants (used by analyzer.py for
# cross-variant reconstruction, e.g. read_fail/trunc interval diffs).
COMMON_FIELDS = tuple(
    name
    for name in FULL_SCALAR_FIELDS
    if name in DIAGNOSTIC_SCALAR_FIELDS
)


class PacketValidationError(ValueError):
    """Raised by validate_packet() when a decoded packet violates schema v1.

    ``reason`` is a short machine-readable code (e.g. "missing_field:dev",
    "bad_type:seq", "bad_version:2") suitable for tallying rejected packets
    by cause in receiver.py.
    """

    def __init__(self, reason: str):
        super().__init__(reason)
        self.reason = reason


def _matches_kind(value: Any, kind: str) -> bool:
    if kind == "int":
        return isinstance(value, int) and not isinstance(value, bool)
    if kind == "number":
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    if kind == "bool":
        return isinstance(value, bool)
    if kind == "str":
        return isinstance(value, str)
    raise AssertionError(f"unknown kind {kind!r}")  # pragma: no cover


def _validate_scalar_fields(obj: Dict[str, Any], spec: Dict[str, str]) -> None:
    for name, kind in spec.items():
        if name not in obj:
            raise PacketValidationError(f"missing_field:{name}")
        if not _matches_kind(obj[name], kind):
            raise PacketValidationError(f"bad_type:{name}")


def _validate_dt_h(obj: Dict[str, Any]) -> None:
    if "dt_h" not in obj:
        raise PacketValidationError("missing_field:dt_h")
    dt_h = obj["dt_h"]
    if not isinstance(dt_h, list) or len(dt_h) != DT_H_LEN:
        raise PacketValidationError("bad_shape:dt_h")
    for item in dt_h:
        if not isinstance(item, int) or isinstance(item, bool):
            raise PacketValidationError("bad_type:dt_h[]")


def classify_packet(obj: Dict[str, Any]) -> PacketVariant:
    """Return "full" or "diagnostic" from a decoded packet's snap_valid.

    Raises PacketValidationError if obj is not a dict or lacks a boolean
    snap_valid field. Does not perform full schema validation; use
    validate_packet() for that.
    """
    if not isinstance(obj, dict):
        raise PacketValidationError("not_object")
    if "snap_valid" not in obj:
        raise PacketValidationError("missing_field:snap_valid")
    snap_valid = obj["snap_valid"]
    if not isinstance(snap_valid, bool):
        raise PacketValidationError("bad_type:snap_valid")
    return "full" if snap_valid else "diagnostic"


def validate_packet(obj: Any) -> PacketVariant:
    """Validate a decoded JSON packet against schema v1.

    Returns the packet variant ("full" or "diagnostic") on success.
    Raises PacketValidationError (with a machine-readable .reason) on any
    schema violation. Unknown extra keys are ignored (forward compatible).
    """
    if not isinstance(obj, dict):
        raise PacketValidationError("not_object")

    if "v" not in obj:
        raise PacketValidationError("missing_field:v")
    if obj["v"] != SCHEMA_VERSION:
        raise PacketValidationError(f"bad_version:{obj['v']!r}")

    variant = classify_packet(obj)

    if variant == "full":
        _validate_scalar_fields(obj, FULL_SCALAR_FIELDS)
        _validate_dt_h(obj)
        return "full"

    _validate_scalar_fields(obj, DIAGNOSTIC_SCALAR_FIELDS)
    if obj["reason"] not in DIAGNOSTIC_REASONS:
        raise PacketValidationError(f"bad_value:reason={obj['reason']!r}")
    return "diagnostic"
