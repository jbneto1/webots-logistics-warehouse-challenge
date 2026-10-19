# Validation checks

- All 15 nominal trials delivered BOX_0 with the expected score, no rejected drops, a detached box inside the outgoing region, and controller completion after clearance.
- The invalid-drop control recorded one rejection, score zero, and no delivery. It is excluded from nominal statistics.
- Retained controller, world, and measurement-source hashes match `manifest.json`. Snapshot exclusions are listed separately.
- Per-run events, trajectories, and state logs agree with the measurement records. Aggregate means, sample standard deviations, and real-time factors were checked against the recorded intervals.
- `tests/navigation/check_planner.py` passed the 24-wall consistency, arc geometry, payload-clearance, straight-docking, and fallback checks. No contact metric is included in this batch.
