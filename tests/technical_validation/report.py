"""Summarize saved BOX_0 measurements."""
import argparse
import json
from pathlib import Path
import statistics

TYPES = {"blue": ("Blue", "BRRG", 1), "green": ("Green", "GRRB", 2), "red": ("Red", "RRGB", 3)}


def describe(values, digits):
    if not values:
        return "unavailable"
    mean = statistics.mean(values)
    if len(values) == 1:
        return f"{mean:.{digits}f} (n=1)"
    return f"{mean:.{digits}f} ± {statistics.stdev(values):.{digits}f}"


def write_report(output):
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    rows = json.loads((output / "runs.json").read_text(encoding="utf-8"))
    nominal = [r for r in rows if not r["negative_control"]]
    controls = [r for r in rows if r["negative_control"]]
    lines = ["# BOX_0 validation", "",
             f"Webots {manifest['webots_version']}; base commit `{manifest['git_head']}`. "
             "The snapshot identifies the tested controller and world sources, including local changes.", "",
             "| Initial type | Order | Deliveries / trials | Score observed (expected) | Delivery time, mean ± sample SD (s) | Range (s) | Invalid drops | RTF, mean ± sample SD |",
             "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for case, (label, order, expected) in TYPES.items():
        group = [r for r in nominal if r["case"] == case]
        if not group:
            continue
        successful = [r for r in group if r.get("success")]
        times = [r["delivery_simulation_s"] for r in successful]
        rtfs = [r["delivery_real_time_factor"] for r in successful]
        scores = ", ".join(str(s) for s in sorted({r.get("supervisor_score", "unavailable") for r in group}, key=str))
        invalid = str(sum(r["invalid_drops"] for r in group)) if all("invalid_drops" in r for r in group) else "unavailable"
        time_range = f"{min(times):.3f}-{max(times):.3f}" if times else "unavailable"
        lines.append(f"| {label} | `{order}` | {len(successful)}/{len(group)} | {scores} ({expected}) | "
                     f"{describe(times, 3)} | {time_range} | {invalid} | {describe(rtfs, 2)} |")
    lines += ["", "## Metrics", "",
              "- Delivery time: simulation start to supervisor acceptance of blue BOX_0 in the outgoing region, including messaging, motion, handling waits, and processing. Time statistics include successful trials only.",
              "- Success: accepted delivery, expected score, no rejected drops, a detached box still inside the outgoing region, and controller FINISHED after reverse clearance.",
              "- Invalid drops: associated-box releases rejected by `evaluateDrop`. Empty magnet-off commands and physical slips without a reported release are outside this count.",
              "- RTF: delivery simulation interval divided by its matching monotonic wall interval. Compilation and simulator startup are excluded; physics, logging, overlay work, and scheduling are included.",
              "- `end_simulation_s`: termination after controller completion or timeout. Observing FINISHED can lag the controller transition by one 32 ms tick.", "",
              "## Configuration", "",
              f"Seeds: {', '.join(map(str, manifest['seeds']))}. BOX_0; machine bay 0; 32 ms timestep. "
              "Uniform 15-25 s processing delays. Supervisor and world seeds are fixed per trial; assigned delays are recorded in `events.csv`. "
              "Routes consume random draws in different orders. Trials run serially in fast mode with 3D rendering disabled and the normal overlay and debug level retained.", "",
              f"Host: `{manifest['platform']}`; `{manifest['processor']}`; "
              f"{manifest.get('logical_processors', 'unavailable')} logical processors. "
              f"Compiler: `{manifest['compiler_version'].splitlines()[0]}`.", "",
              "## Records", "",
              "`runs.csv`/`runs.json` contain individual observations; `summary.json` contains aggregates. "
              "`manifest.json` records versions, configuration, source hashes, and snapshot exclusions. "
              "`source_snapshot/` contains the retained test inputs. Each trial includes commands, patches, events, trajectories, state logs, and measurement JSON.", ""]
    if controls:
        lines += ["## Invalid-drop control", ""]
        for row in controls:
            lines += [f"Blue, seed {row['seed']}: score {row.get('supervisor_score', 'unavailable')}; "
                      f"rejected drops {row.get('invalid_drops', 'unavailable')}; delivered={row.get('box_0_delivered', 'unavailable')}; "
                      f"control passed={row['validation_check_passed']}. Excluded from nominal statistics.", ""]
    lines += ["## Scope", "",
              "Single-box, bay-0 operation with supervisor-provided pose. No contact metric was collected. "
              "The blue route has no random processing delay; machine-route timing varies with assigned delays. "
              "The trials do not cover four-box scheduling or establish reliability beyond the tested conditions. RTF is specific to this host. "
              "Pilot and baseline batches are separate.", ""]
    (output / "REPORT.md").write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    write_report(parser.parse_args().output.resolve())
