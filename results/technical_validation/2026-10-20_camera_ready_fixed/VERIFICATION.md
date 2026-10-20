# Corrected camera-ready batch verification

Verified 20 October 2026. All 15 nominal trials and the separate invalid-drop control passed. The corrected harness, source snapshots and raw evidence are retained in this directory. The 19 October baseline was not modified.

| Type | Nominal successes | Score in each trial | Delivery time, mean ± sample SD (s) | Rejected releases, total | RTF, mean ± sample SD |
|---|---:|---:|---:|---:|---:|
| Blue | 5/5 | 1 | 20.832 ± 0.000 | 0 | 16.15 ± 0.77 |
| Green | 5/5 | 2 | 49.184 ± 1.593 | 0 | 16.09 ± 0.82 |
| Red | 5/5 | 3 | 78.765 ± 2.583 | 0 | 16.44 ± 0.75 |

The negative control (blue, seed 101) completed with no delivery, score zero and one rejected release. Its Webots exit code 1 is the expected negative outcome and the control check passed. Controls are excluded from nominal statistics. The paper table reports mean RTF only: 16.15, 16.09 and 16.44.

## Execution and source identity

The exact recorded command, from the repository root, was:

```bash
python tests/technical_validation/run.py --source results/technical_validation/2026-10-19_baseline/source_snapshot --output results/technical_validation/2026-10-20_camera_ready_fixed --seeds 101 102 103 104 105 --negative-control
```

Execution used Python 3.13.12, Webots R2025a, g++ 14.2 with `-O2`, Windows 11, serial fast-mode runs and disabled 3D rendering. The CPU is an AMD Ryzen 9 7945HX (16 cores, 32 logical processors). The author confirmed 32 GB installed RAM; `HOST_ENVIRONMENT.json` separately records the OS-visible memory and CIM observations.

`manifest.json` records base Git HEAD `92093f083c07619ad1e3cbd48f7c4549bb89d2f4` plus working-tree changes and all 14 input SHA-256 hashes. That base HEAD does not itself contain the new fix. The hashes, complete source snapshot and per-trial instrumentation identify the actual tested inputs; a later publication commit/tag contains this evidence. No snapshot files were excluded. All 14 archived inputs matched the manifest and current files at sign-off, and the ten world/controller files match the original baseline inputs. The named command in the general validation README uses current inputs by default; those ten inputs are byte-identical here, but the explicit `--source` command above records the actual run.

## Independent checks

The raw checker reconstructs measurements from events, trajectories and measurement JSON without using `REPORT.md` or `summary.json` as its numerical source. The supplemental checker independently verifies the case matrix, source provenance, exports and comparison with the old baseline.

- Exactly 16 unique cases: three initial types at seeds 101--105, plus the blue seed-101 control.
- Every nominal case has the expected score and no rejected release; all declared harness checks passed.
- All 16 final trajectory timestamps, JSON end timestamps and single terminal-event timestamps agree; every offset is zero.
- Final score, delivery and attachment flags, robot XY and part XYZ fields agree between trajectory and JSON; outgoing acceptance uses the part centre.
- All 24,494 retained trajectory samples have consistent event-derived score, logical attachment and delivery state. Fifteen processing lifecycles satisfy the declared delay/timestep semantics.
- All per-seed delivery times, scores, rejection counts, assigned processing delays and pre-delivery trajectory fields reproduce the original baseline. New RTF observations are session-specific and are kept as a separate batch.
- CSV exports and summary aggregates agree with raw reconstruction, including sample SD and the arithmetic mean of per-trial RTF ratios.
- Fourteen regression tests cover consistent delivery, control and timeout records, plus mismatched/duplicate/missing terminal evidence, inconsistent final state/positions, and nonfinite positions.

The strict sign-off outputs in `verification/raw_recomputed.json` and `verification/extra_checks.json` have no errors or hash mismatches; all 372 supplemental assertions passed. The runtime manifest also records completed checks and unchanged source inputs during execution.

## Recheck without running Webots

From a checkout containing both archived batches:

```bash
python tests/technical_validation/verify_records.py --base results/technical_validation/2026-10-20_camera_ready_fixed --result tmp/corrected_raw_check.json
python tests/technical_validation/check_corrected_batch_provenance.py --base results/technical_validation/2026-10-20_camera_ready_fixed --original results/technical_validation/2026-10-19_baseline --recomputed tmp/corrected_raw_check.json --result tmp/corrected_extra_check.json
```

Both tools enforce archived source hashes and evidence consistency and exit nonzero on required failures. Current working-tree hashes are informational by default: different checkout line endings or later edits do not alter the archived evidence. Add `--require-current-match` to both commands to require all 14 current inputs to match byte for byte, as at local sign-off. That strict mode produced 372 supplemental assertions; archive-only mode omits the 14 current-tree assertions and passed 358. The scoped `.gitattributes` rule preserves all corrected evidence bytes across clones with automatic line-ending conversion enabled.

## Interpretation limits

This verifies the stated nominal single-part routes, bay 0 and supervisor-provided pose conditions. It does not establish four-part scheduling performance, broad reliability, collision-free execution, physical connector-link sensing, full-footprint containment, physical transfer or classroom learning outcomes. The historical baseline's shutdown defect remains documented with its unchanged records; it is absent from this corrected batch.
