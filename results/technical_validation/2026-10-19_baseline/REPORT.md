# BOX_0 validation

Webots R2025a; base commit `986d0e58cd6e2b13e1bff9746ac940a8b753290c`. The snapshot identifies the tested controller and world sources, including local changes.

| Initial type | Order | Deliveries / trials | Score observed (expected) | Delivery time, mean ± sample SD (s) | Range (s) | Invalid drops | RTF, mean ± sample SD |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Blue | `BRRG` | 5/5 | 1 (1) | 20.832 ± 0.000 | 20.832-20.832 | 0 | 16.30 ± 0.85 |
| Green | `GRRB` | 5/5 | 2 (2) | 49.184 ± 1.593 | 47.104-51.392 | 0 | 16.68 ± 0.67 |
| Red | `RRGB` | 5/5 | 3 (3) | 78.765 ± 2.583 | 76.320-82.784 | 0 | 16.90 ± 0.72 |

## Metrics

- Delivery time: simulation start to supervisor acceptance of blue BOX_0 in the outgoing region, including messaging, motion, handling waits, and processing. Time statistics include successful trials only.
- Success: accepted delivery, expected score, no rejected drops, a detached box still inside the outgoing region, and controller FINISHED after reverse clearance.
- Invalid drops: associated-box releases rejected by `evaluateDrop`. Empty magnet-off commands and physical slips without a reported release are outside this count.
- RTF: delivery simulation interval divided by its matching monotonic wall interval. Compilation and simulator startup are excluded; physics, logging, overlay work, and scheduling are included.
- `end_simulation_s`: termination after controller completion or timeout. Observing FINISHED can lag the controller transition by one 32 ms tick.

## Configuration

Seeds: 101, 102, 103, 104, 105. BOX_0; machine bay 0; 32 ms timestep. Uniform 15-25 s processing delays. Supervisor and world seeds are fixed per trial; assigned delays are recorded in `events.csv`. Routes consume random draws in different orders. Trials run serially in fast mode with 3D rendering disabled and the normal overlay and debug level retained.

Host: `Windows-11-10.0.26300-SP0`; `AMD64 Family 25 Model 97 Stepping 2, AuthenticAMD`; 32 logical processors. Compiler: `g++.exe (Rev2, Built by MSYS2 project) 14.2.0`.

## Records

`runs.csv`/`runs.json` contain individual observations; `summary.json` contains aggregates. `manifest.json` records versions, configuration, source hashes, and snapshot exclusions. `source_snapshot/` contains the retained test inputs. Each trial includes commands, patches, events, trajectories, state logs, and measurement JSON.

## Invalid-drop control

Blue, seed 101: score 0; rejected drops 1; delivered=False; control passed=True. Excluded from nominal statistics.

## Scope

Single-box, bay-0 operation with supervisor-provided pose. No contact metric was collected. The blue route has no random processing delay; machine-route timing varies with assigned delays. The trials do not cover four-box scheduling or establish reliability beyond the tested conditions. RTF is specific to this host. Pilot and baseline batches are separate.
