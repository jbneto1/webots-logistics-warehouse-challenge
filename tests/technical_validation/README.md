# BOX_0 validation

Runs the reference C++ controller in isolated Webots projects for blue (`BRRG`), green (`GRRB`), and red (`RRGB`) initial orders. Expected scores are 1, 2, and 3. Tests use BOX_0, machine bay 0, a 32 ms timestep, and the existing uniform 15-25 s processing delays.

Requires Python 3, Webots R2025a, and its C++ toolchain. From the repository root:

```bash
python tests/technical_validation/run.py --output results/technical_validation/my_baseline --seeds 101 102 103 104 105 --negative-control
```

The output directory must be new. Webots runs serially with the 3D view disabled and quits after each trial. Windows uses the bundled compiler; other platforms require `g++` and have not been tested here.

| Option | Purpose |
| --- | --- |
| `--webots-home PATH` | Webots installation directory. |
| `--source PATH` | Source tree to test; defaults to the current repository. |
| `--cases blue green red` | Routes to run. |
| `--seeds 101 102 103 104 105` | Supervisor and world seeds. |
| `--negative-control` | Also release a blue box outside a valid destination. |
| `--prepare-only` | Build projects without starting Webots. |
| `--sim-timeout N` / `--wall-timeout N` | Per-trial limits in seconds; both default to 240. |

The negative control must record one rejected drop, score zero, no delivery, and controller completion. Its expected simulator failure exit counts as a passed control check. Controls are excluded from nominal aggregates. Unsuccessful nominal trials return a nonzero harness status and retain their records.

## Files

| Path | Contents |
| --- | --- |
| `runs.csv`, `runs.json` | Individual measurements. |
| `summary.json`, `REPORT.md` | Aggregate results and metric definitions. |
| `manifest.json` | Versions, configuration, source hashes, and snapshot exclusions. |
| `source_snapshot/` | Test input sources. |
| `trials/` | Build commands, source patches, events, trajectories, state logs, and measurement JSON. |

Instrumentation records the existing supervisor acceptance/rejection branches and marks the controller's final state. Navigation, scoring, processing, and communication behavior are retained. The negative control changes only the destination decision. A raw value of -1 for delivery time or RTF means no delivery measurement.

Summarize saved observations with:

```bash
python tests/technical_validation/report.py results/technical_validation/my_baseline
```

To retest the recorded controller and world sources with the current harness:

```bash
python tests/technical_validation/run.py --source results/technical_validation/2026-10-19_baseline/source_snapshot --output tmp/validation_retest --seeds 101 102 103 104 105 --negative-control
```

Snapshots are read-only test inputs. Archived harness scripts record the original implementation and may depend on files listed in `snapshot_excluded_files`; use the current `run.py --source` command above for reruns. Excluded files retain their original hashes in the manifest. Each new run records the current harness and measurement header alongside its input sources.

Scope: single-box, bay-0 operation with supervisor-provided pose. These tests do not collect contact measurements or cover four-box scheduling. See `tests/navigation/` for geometry and contact regressions.
