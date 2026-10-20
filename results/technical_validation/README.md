# Validation results

| Batch | Contents |
| --- | --- |
| [2026-10-20_camera_ready_fixed](2026-10-20_camera_ready_fixed/REPORT.md) | Completed corrected-harness batch: all 15 nominal trials and the invalid-drop control passed. [Manifest](2026-10-20_camera_ready_fixed/manifest.json); [verification](2026-10-20_camera_ready_fixed/VERIFICATION.md). |
| [2026-10-19_baseline](2026-10-19_baseline/REPORT.md) | Historical pre-fix batch: five trials per initial type and one invalid-drop control. See terminal-metadata caveat below. |
| [2026-10-19_pilot](2026-10-19_pilot/manifest.json) | Historical pre-fix batch: one trial per initial type and one invalid-drop control. |

The batches are separate; controls are excluded from nominal statistics.

The 20 October corrected batch uses the 19 October baseline's retained controller/world sources with the corrected current harness. All five seeds per initial type delivered with the expected scores and no rejected releases; the separate invalid-drop control recorded one rejection, zero score, and no delivery. Terminal-evidence checks passed and the completed manifest records `source_unchanged_after_run: true`. Delivery times reproduce the historical baseline; wall throughput is specific to this new batch and is not substituted into the historical records.

The recorded comparison command was:

```bash
python tests/technical_validation/run.py --source results/technical_validation/2026-10-19_baseline/source_snapshot --output results/technical_validation/2026-10-20_camera_ready_fixed --seeds 101 102 103 104 105 --negative-control
```

Choose a fresh output directory when rerunning; the named batch already exists. The [validation instructions](../../tests/technical_validation/README.md) describe the harness and metric definitions. This is a separate corrected batch, not an in-place repair of the historical archive.

`runs.csv` and `runs.json` contain individual measurements; `summary.json` contains aggregates. `manifest.json` records the environment, seeds, and source hashes. `source_snapshot/` retains test inputs, with any exclusions listed in the manifest. `trials/` contains commands, patches, events, trajectories, and controller logs. `working_tree.diff` records the controller changes relative to the base commit.

Success uses supervisor acceptance, expected score, zero rejected releases, logical detachment, and the part centre inside the outgoing acceptance region when the controller has finished after reversing clear. It does not measure full-footprint containment or independently verify physical connector linkage. Reports aggregate successful nominal delivery times and per-trial RTF ratios using their arithmetic means and sample standard deviations; rejection counts are totals.

The pre-fix harness could enter finalization again after requesting simulator shutdown, overwriting terminal JSON one step later and accessing already closed log streams. In the 19 October baseline, three JSON end times differ by 32 ms from the final event/trajectory. Independent event reconstruction and fresh execution reproduced its delivery-time statistics, scores, and rejection counts; those findings do not remove the terminal-metadata defect. Historical directories remain unchanged.

The corrected harness exits the supervisor loop after finalization and rejects mismatches between terminal JSON, the final trajectory, and the single terminal event, including their timestamps (tolerance 1e-7 s) and final-state fields. Its manifest records `terminal_consistency_checked`; only a completed passing batch establishes that its records satisfy the checks.

See the [validation instructions](../../tests/technical_validation/README.md) for execution and metric definitions.
