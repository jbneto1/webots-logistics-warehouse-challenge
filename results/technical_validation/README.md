# Validation results

| Batch | Contents |
| --- | --- |
| [2026-10-19_baseline](2026-10-19_baseline/REPORT.md) | Five trials per initial type and one invalid-drop control. |
| [2026-10-19_pilot](2026-10-19_pilot/manifest.json) | One trial per initial type and one invalid-drop control. |

The batches are separate; controls are excluded from nominal statistics.

`runs.csv` and `runs.json` contain individual measurements; `summary.json` contains aggregates. `manifest.json` records the environment, seeds, and source hashes. `source_snapshot/` retains test inputs, with any exclusions listed in the manifest. `trials/` contains commands, patches, events, trajectories, and controller logs. `working_tree.diff` records the controller changes relative to the base commit.

See the [validation instructions](../../tests/technical_validation/README.md) for execution and metric definitions.
