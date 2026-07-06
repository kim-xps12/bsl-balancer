"""bsl_telemetry: PC-side UDP telemetry tooling for bsl-balancer (Phase 1).

Standard-library only (Python 3.11+). See
docs/plans/2026-07-05-udp-telemetry-phase1.md section 3.2 for the design.

Modules:
  schema     - packet schema v1 definition and validation
  receiver   - UDP receiver -> session directory -> raw.jsonl
  analyzer   - post-run analysis: raw.jsonl -> events.jsonl, summary.json
  report     - markdown digest of a session for humans/coding agents
  simulator  - synthetic packet sender for exercising the receiver
"""

__all__ = ["schema", "receiver", "analyzer", "report", "simulator"]
