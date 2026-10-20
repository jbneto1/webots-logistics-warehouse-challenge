"""Independent raw-record audit; does not launch Webots or change original inputs.

Portable at output/audit/ or tests/technical_validation/ (two levels below repo root).
Use explicit --base and --result for published verification; only the result JSON is written.
"""
from pathlib import Path
import argparse
import ast
import csv
import hashlib
import json
import math
import statistics

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--base", type=Path, default=ROOT / "results/technical_validation/2026-10-19_baseline",
                    help="Complete evidence archive to audit; defaults to the original baseline")
parser.add_argument("--result", type=Path, default=OUT / "validation_records_recomputed.json",
                    help="Recomputation JSON destination; set explicitly when checking a fresh archive")
parser.add_argument("--require-current-match", action="store_true",
                    help="Also require current working-tree input bytes to match the archive; default checks archive hashes only")
args = parser.parse_args()
BASE = args.base.resolve()
RESULT = args.result.resolve()

def load(path):
    return json.loads(path.read_text(encoding="utf-8"))

def read_csv(path):
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def close(a, b, tol=1.1e-8):
    return abs(float(a)-float(b)) <= tol

manifest = load(BASE / "manifest.json")
snapshot_hashes = []
for rel, digest in manifest["source_sha256"].items():
    snapshot = BASE / "source_snapshot" / rel
    current = ROOT / rel
    snapshot_hashes.append({"path": rel,
        "snapshot_present": snapshot.exists(),
        "current_present": current.exists(),
        "snapshot_matches_manifest": sha(snapshot) == digest if snapshot.exists() else None,
        "current_matches_manifest": sha(current) == digest if current.exists() else None,
        "snapshot_hash": sha(snapshot) if snapshot.exists() else None,
        "current_hash": sha(current) if current.exists() else None})

# Reconstruct all instrumented trial sources from the archived instrumentation,
# without importing its missing report module or executing its main routine.
source = (BASE / "source_snapshot/tests/technical_validation/run.py").read_text(encoding="utf-8")
tree = ast.parse(source)
chosen = [node for node in tree.body if isinstance(node, ast.FunctionDef)
          and node.name in {"replace_once", "instrument"}]
namespace = {}
exec(compile(ast.Module(body=chosen, type_ignores=[]), "archived_instrument", "exec"), namespace)

all_runs = load(BASE / "runs.json")
reports = []
all_errors = []
for run in all_runs:
    trial = BASE / run["trial_directory"].replace("\\", "/")
    evidence = trial / "controllers/logistics_supervisor_cpp"
    measured = load(evidence / "measurement.json")
    events = read_csv(evidence / "events.csv")
    trajectory = read_csv(evidence / "trajectory.csv")
    end = trajectory[-1]
    errors = []
    def require(condition, message):
        if not condition:
            errors.append(message)

    for key, value in measured.items():
        require(run.get(key) == value, f"runs.json mismatch: {key}")
    deliveries = [e for e in events if e["event"] == "delivery"]
    accepted = [e for e in events if e["event"] == "machine_accepted"]
    rejected = [e for e in events if e["event"] == "invalid_drop"]
    pickups = [e for e in events if e["event"] == "pickup"]
    starts = [e for e in events if e["event"] == "processing_start"]
    ready = [e for e in events if e["event"] == "processing_ready"]
    require(len(deliveries) == (0 if run["negative_control"] else 1), "delivery event count")
    require(len(rejected) == measured["invalid_drops"], "rejection count")
    require(len(accepted) + len(deliveries) + len(rejected) == measured["evaluated_drops"], "drop count")
    require(bool(deliveries) == measured["box_0_delivered"], "delivered flag")
    require(events[-1]["event"] == "controller_finished", "no finish event")
    require(close(events[-1]["simulation_s"], end["simulation_s"]), "end event/trace time mismatch")
    require(close(end["simulation_s"], measured["end_simulation_s"]), "trace/measurement end time")
    log = (trial / "controllers/student_controller_cpp/controller.log").read_text(encoding="utf-8")
    require("State: FINISHED " in log, "controller state log lacks FINISHED")
    require(int(end["score"]) == measured["supervisor_score"], "final trace score")
    require(int(end["attached_index"]) == measured["attached_index_at_end"] == -1, "logical attachment")
    require(int(end["box_0_delivered"]) == measured["box_0_delivered"], "final delivered trace flag")
    final = [float(end[k]) for k in ("box_x_m", "box_y_m", "box_z_m")]
    require(all(close(a, b) for a,b in zip(final, measured["box_final_xyz_m"])), "box final position")
    require(all(close(end[k], v) for k,v in zip(("robot_x_m", "robot_y_m"), measured["robot_final_xy_m"])), "robot final position")
    inside = .160 <= final[0] <= .785 and -.590 <= final[1] <= -.420
    require(inside == measured["box_0_in_outgoing_at_end"], "outgoing test")
    require(all(close(float(e["simulation_s"])/.032, round(float(e["simulation_s"])/.032), 1e-5) for e in events), "event times off timestep grid")
    require(all(close(float(t["simulation_s"]), (i+1)*.032, 1e-7) for i,t in enumerate(trajectory)), "trajectory time grid/gaps")
    require(all(float(a["simulation_s"]) <= float(b["simulation_s"]) and float(a["wall_s"]) <= float(b["wall_s"]) for a,b in zip(events, events[1:])), "event monotonicity")
    require(len(starts) == len(ready) == len(accepted), "processing event counts")
    processing = []
    for start, finish, accept in zip(starts, ready, accepted):
        assigned = float(start["assigned_delay_s"])
        elapsed = float(finish["simulation_s"]) - float(start["simulation_s"])
        require(15 <= assigned <= 25, "delay out of bounds")
        require(close(elapsed, math.ceil((assigned-1e-10)/.032)*.032), "ready not first timestep after delay")
        require(start["machine"] == finish["machine"] == accept["machine"], "machine lifecycle mismatch")
        require(start["bay"] == finish["bay"] == accept["bay"] == "0", "bay mismatch")
        require(close(start["simulation_s"], accept["simulation_s"]), "processing acceptance/start time")
        processing.append({"machine": start["machine"], "assigned_delay_s": assigned,
                           "observed_delay_s": elapsed, "quantization_overrun_s": elapsed-assigned})
    credit = 0
    for event in events:
        if event["event"] in {"delivery", "machine_accepted"}:
            credit += 1
        require(int(event["score"]) == credit, "event score increment inconsistency")
    require(credit == measured["supervisor_score"], "raw-event final score mismatch")
    trace_state_errors = 0
    event_i = 0
    trace_credit, trace_attached, trace_delivered = 0, -1, 0
    for row in trajectory:
        while event_i < len(events) and float(events[event_i]["simulation_s"]) <= float(row["simulation_s"])+1e-8:
            event = events[event_i]
            if event["event"] in {"delivery", "machine_accepted"}:
                trace_credit += 1
            if event["event"] == "pickup":
                trace_attached = 0
            elif event["event"] in {"delivery", "machine_accepted", "invalid_drop"}:
                trace_attached = -1
            if event["event"] == "delivery":
                trace_delivered = 1
            event_i += 1
        trace_state_errors += (int(row["score"]) != trace_credit or int(row["attached_index"]) != trace_attached or int(row["box_0_delivered"]) != trace_delivered)
    require(trace_state_errors == 0, f"{trace_state_errors} trajectory state rows disagree with raw events")
    require(len(pickups) == len(accepted)+1, "pickup event count")
    derived_sim = float(deliveries[0]["simulation_s"]) if deliveries else None
    derived_rtf = derived_sim / measured["delivery_wall_s"] if deliveries else None
    if deliveries:
        require(close(derived_sim, measured["delivery_simulation_s"]), "delivery timestamp mismatch")
        require(close(derived_rtf, measured["delivery_real_time_factor"], 2e-8), "RTF mismatch")
        # Event logging calls wall() just before deliveryWall_ is captured.
        # Logging can block or be preempted, so no fixed wall-gap upper bound is an invariant.
        require(measured["delivery_wall_s"] - float(deliveries[0]["wall_s"]) >= -1e-9, "delivery wall timer precedes its event")
        expected = {"blue":1, "green":2, "red":3}[run["case"]]
        require(credit == expected and inside and not rejected and measured["success"], "nominal success criteria")
        require(len(accepted) == expected-1, "machine acceptance count by type")
        require([e["machine"] for e in accepted] == {"blue":[], "green":["B"], "red":["A","B"]}[run["case"]], "machine route order")
        first_delivered = next(t for t in trajectory if t["box_0_delivered"] == "1")
        require(close(first_delivered["simulation_s"], derived_sim), "delivery flag trace onset")
    else:
        require(credit == 0 and len(rejected) == 1 and not measured["success"] and not inside, "negative-control outcome")

    # Rebuild expected instrumented contents from archived source text only.
    trial_source_mismatches = []
    for controller in ("student_controller_cpp", "logistics_supervisor_cpp"):
        folder = BASE / "source_snapshot/controllers" / controller
        for archived in folder.iterdir():
            original = archived.read_text(encoding="utf-8")
            expected_text = namespace["instrument"](archived.name, original, run["order"], run["seed"], measured["expected_score"], manifest["sim_timeout_s"], run["negative_control"])
            target = trial / "controllers" / controller / archived.name
            if target.read_text(encoding="utf-8") != expected_text:
                trial_source_mismatches.append(target.relative_to(BASE).as_posix())
    require(not trial_source_mismatches, "trial source differs from archived instrumentation")
    original_world = (BASE / "source_snapshot/worlds/logistics_pbl_enu.wbt").read_text(encoding="utf-8")
    expected_world = original_world.replace("WorldInfo {", f'WorldInfo {{\n  randomSeed {run["seed"]}\n  basicTimeStep 32', 1)
    require((trial / "worlds/validation.wbt").read_text(encoding="utf-8") == expected_world, "trial world source mismatch")
    require(sha(evidence / "measurement.hpp") == manifest["source_sha256"]["tests/technical_validation/measurement.hpp"], "measurement source hash mismatch")
    commands = load(trial / "commands.json")
    require("--no-rendering" in commands["run"] and "--mode=fast" in commands["run"], "execution mode")
    require(all("-O2" in build for build in commands["build"]), "optimization flags")
    separation = math.dist(measured["robot_final_xy_m"], final[:2])
    reports.append({"trial": trial.name, "case": run["case"], "seed": run["seed"],
        "negative_control": run["negative_control"], "raw_event_delivery_simulation_s": derived_sim,
        "delivery_wall_s": measured["delivery_wall_s"] if deliveries else None,
        "recomputed_rtf": derived_rtf, "score_from_events": credit,
        "delivery_count":len(deliveries), "rejection_count":len(rejected), "trajectory_rows":len(trajectory),
        "trajectory_state_row_mismatches":trace_state_errors,
        "finish_event_simulation_s":float(events[-1]["simulation_s"]),
        "last_trace_simulation_s":float(end["simulation_s"]),
        "measurement_end_simulation_s":measured["end_simulation_s"],
        "last_trace_to_measurement_offset_s":measured["end_simulation_s"]-float(end["simulation_s"]),
        "delivery_wall_event_offset_s":measured["delivery_wall_s"]-float(deliveries[0]["wall_s"]) if deliveries else None,
        "final_box_xyz_m": final, "final_robot_box_separation_m":separation,
        "processing": processing, "errors":errors})
    all_errors.extend(f"{trial.name}: {err}" for err in errors)

aggregates = []
for case in ("blue", "green", "red"):
    selected = [r for r in reports if r["case"] == case and not r["negative_control"]]
    times = [r["raw_event_delivery_simulation_s"] for r in selected]
    rtfs = [r["recomputed_rtf"] for r in selected]
    aggregates.append({"case":case, "n":len(selected), "mean_simulation_s":statistics.mean(times),
        "sample_sd_simulation_s":statistics.stdev(times) if len(times) > 1 else None, "mean_rtf":statistics.mean(rtfs),
        "sample_sd_rtf":statistics.stdev(rtfs) if len(rtfs) > 1 else None, "raw_times_s":times,
        "mean_wall_s":statistics.mean(r["delivery_wall_s"] for r in selected),
        "ratio_of_means_for_comparison":statistics.mean(times)/statistics.mean(r["delivery_wall_s"] for r in selected)})

archive_hash_mismatches = [x for x in snapshot_hashes if x["snapshot_matches_manifest"] is False
                          or (not x["snapshot_present"] and x["path"] not in manifest.get("snapshot_excluded_files", []))]
current_hash_differences = [x for x in snapshot_hashes if x["current_matches_manifest"] is not True]
hash_mismatches = archive_hash_mismatches + (current_hash_differences if args.require_current_match else [])
output = {"source":"Raw events.csv, trajectory.csv, measurement.json, runs.json; no REPORT.md or summary.json used in recomputation",
          "archive_base": str(BASE), "hash_mismatches": hash_mismatches,
          "archive_hashes_required": True, "current_match_required": args.require_current_match,
          "archive_hash_mismatches": archive_hash_mismatches, "current_hash_differences": current_hash_differences,
          "aggregates":aggregates, "trials":reports, "hashes":snapshot_hashes, "errors":all_errors}
RESULT.parent.mkdir(parents=True, exist_ok=True)
RESULT.write_text(json.dumps(output, indent=2)+"\n", encoding="utf-8")
print(json.dumps({"current_match_required":args.require_current_match, "aggregates":aggregates,
                  "errors":all_errors, "hash_mismatches":hash_mismatches,
                  "current_hash_differences":current_hash_differences}, indent=2))
raise SystemExit(bool(all_errors or hash_mismatches))
