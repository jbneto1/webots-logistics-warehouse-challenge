"""Supplement the independent raw check: matrix, provenance, exports, and old/new comparison.

Read-only with respect to source and measurement archives. Writes audit JSON only.
Run the independent raw verifier (verify_records.py in a published checkout) first.
Portable at output/audit/ or tests/technical_validation/ (two levels below repo root).
Published use should provide --base, --original, --recomputed, and --result explicitly.
"""
from pathlib import Path
import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import statistics

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--base", type=Path, default=ROOT / "results/technical_validation/2026-10-20_camera_ready_fixed")
parser.add_argument("--original", type=Path, default=ROOT / "results/technical_validation/2026-10-19_baseline")
parser.add_argument("--recomputed", type=Path, default=ROOT / "output/audit/corrected_batch_recomputed.json")
parser.add_argument("--result", type=Path, default=ROOT / "output/audit/corrected_batch_extra_checks.json")
parser.add_argument("--require-current-match", action="store_true",
                    help="Also require current working-tree input bytes to match the archive; default reports those hashes informationally")
args = parser.parse_args()
base, original = args.base.resolve(), args.original.resolve()

def load(path):
    return json.loads(path.read_text(encoding="utf-8"))

def table(path):
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def close(a, b, tolerance=1e-8):
    return math.isfinite(float(a)) and math.isfinite(float(b)) and abs(float(a)-float(b)) <= tolerance

def identity(row):
    return row["case"], int(row["seed"]), bool(row["negative_control"])

checks, errors, checksums = [], [], {}
def check(ok, name, detail=None):
    item = {"name": name, "pass": bool(ok)}
    if detail is not None:
        item["detail"] = detail
    checks.append(item)
    if not ok:
        errors.append(name)

manifest = load(base / "manifest.json")
runs = load(base / "runs.json")
old_runs = load(original / "runs.json")
recomputed = load(args.recomputed)
expected = {(case, seed, False) for case in ("blue", "green", "red") for seed in range(101,106)} | {("blue",101,True)}
matrix = [identity(row) for row in runs]
check(len(runs) == 16 and set(matrix) == expected and all(n == 1 for n in Counter(matrix).values()),
      "exact_unique_15_nominal_plus_seed101_blue_negative_control_matrix")
check(len([row for row in runs if not row["negative_control"]]) == 15, "nominal_sample_count_15")
check(manifest["seeds"] == list(range(101,106)) and manifest["cases"] == ["blue","green","red"], "manifest_seeds_cases")
check(manifest["machine_bays"] == {"A":0,"B":0}, "manifest_machine_bays")
check(manifest["prepared_only"] is False and manifest["validation_checks_passed"] is True and
      manifest["terminal_consistency_checked"] is True and manifest["source_unchanged_after_run"] is True and
      bool(manifest["completed_utc"]), "manifest_completed_validated_and_frozen_flags")
check(datetime.fromisoformat(manifest["completed_utc"]) >= datetime.fromisoformat(manifest["created_utc"]), "manifest_completion_after_start")
check(manifest["snapshot_excluded_files"] == [], "no_snapshot_exclusions")
check(not recomputed["errors"], "independent_raw_recomputation_no_errors", recomputed["errors"])

declared = manifest["source_sha256"]
present = {p.relative_to(base / "source_snapshot").as_posix() for p in (base / "source_snapshot").rglob("*") if p.is_file()}
check(len(declared) == 14 and set(declared) == present, "all_14_declared_snapshot_inputs_present_no_undeclared_files")
source_details = []
for relative, hashed in declared.items():
    snapshot, current = base / "source_snapshot" / relative, ROOT / relative
    snap_hash, current_hash = digest(snapshot), digest(current) if current.is_file() else None
    check(snap_hash == hashed, f"snapshot_hash:{relative}")
    if args.require_current_match:
        check(current_hash == hashed, f"current_repo_hash:{relative}")
    checksums[snapshot.relative_to(base).as_posix()] = snap_hash
    old_path = original / "source_snapshot" / relative
    source_details.append({"path": relative, "canonical_sha256":snap_hash,
                           "current_repo_sha256":current_hash,"current_repo_matches_archive":current_hash == hashed,
                           "original_sha256":digest(old_path) if old_path.exists() else None,
                           "same_as_original":digest(old_path) == snap_hash if old_path.exists() else None})
gameplay = [row for row in source_details if row["path"].startswith(("worlds/","controllers/"))]
check(len(gameplay) == 10 and all(row["same_as_original"] for row in gameplay), "all_10_gameplay_world_controller_sources_unchanged_from_original")
measurement = (base / "source_snapshot/tests/technical_validation/measurement.hpp").read_text(encoding="utf-8")
check("bool step(" in measurement and "if (finalized_) return true;" in measurement and
      "finalized_ = true;" in measurement and "events_ = nullptr;" in measurement and "trace_ = nullptr;" in measurement,
      "archived_finalization_guard_and_closed_pointer_clearance_present")
check((base / "source_snapshot/tests/technical_validation/report.py").is_file(), "archived_run_report_import_dependency_present")

old_by_id = {identity(row): row for row in old_runs}
comparisons, trial_results = [], []
nominal_box_xyz, separations = [], []
processing_total, trajectory_total = 0, 0
all_delivery_event_wall_offsets = []
numeric_trace_columns = ("simulation_s","robot_x_m","robot_y_m","robot_yaw_rad","box_x_m","box_y_m","box_z_m","score","attached_index","box_0_delivered")
for run in runs:
    case, seed, negative = identity(run)
    name = f"{case}_seed_{seed}" + ("_invalid_drop" if negative else "")
    trial = base / run["trial_directory"].replace("\\","/")
    supervisor = trial / "controllers/logistics_supervisor_cpp"
    events = table(supervisor / "events.csv")
    trace = table(supervisor / "trajectory.csv")
    measured = load(supervisor / "measurement.json")
    commands = load(trial / "commands.json")
    terminal = [e for e in events if e["event"] in {"controller_finished","simulation_timeout"}]
    delivery = [e for e in events if e["event"] == "delivery"]
    rejection = [e for e in events if e["event"] == "invalid_drop"]
    processing = [e for e in events if e["event"] == "processing_start"]
    last = trace[-1]
    expected_order, expected_score = {"blue":("BRRG",1),"green":("GRRB",2),"red":("RRGB",3)}[case]
    check(trial.name == name and trial.is_relative_to(base / "trials"), f"trial_identity_and_containment:{name}")
    check(run["order"] == expected_order and measured["expected_score"] == expected_score, f"manual_order_expected_score:{name}")
    check(run["validation_check_passed"] is True and run["controller_finished"] is True, f"validation_finished_flags:{name}")
    check(run["webots_exit_code"] == (1 if negative else 0), f"exit_code:{name}")
    check(run["success"] is (not negative), f"success_flag:{name}")
    check(len(terminal) == 1 and terminal[0] == events[-1] and terminal[0]["event"] == "controller_finished" and
          close(terminal[0]["simulation_s"], last["simulation_s"]) and close(last["simulation_s"], measured["end_simulation_s"]),
          f"exactly_one_terminal_event_trace_json_timestamp_agreement:{name}")
    check(all(e["box"] == "0" for e in events if e["event"] != "start"), f"only_box0_events:{name}")
    check(all(math.isfinite(float(row[column])) for row in trace for column in numeric_trace_columns), f"all_trace_numeric_fields_finite:{name}")
    check(0 < measured["end_simulation_s"] < manifest["sim_timeout_s"] and
          0 < run["process_wall_s"] < manifest["wall_timeout_s"] and
          measured["end_wall_s"] >= float(terminal[0]["wall_s"]), f"simulation_wall_timeout_and_terminal_wall_intervals:{name}")
    check(len(commands["build"]) == 2 and all(all(flag in command for flag in ["-std=gnu++17","-Wall","-Wextra","-O2"])
          for command in commands["build"]), f"recorded_compiler_configuration:{name}")
    check(all(flag in commands["run"] for flag in ["--batch","--mode=fast","--no-rendering"]), f"recorded_execution_mode:{name}")
    supervisor_text = (supervisor / "logistics_supervisor_cpp.cpp").read_text(encoding="utf-8")
    check("if (validation_.step(robot_, mobile_, boxes_[0].node, score_, boxes_[0].delivered, attachedIndex_)) break;" in supervisor_text,
          f"compiled_source_breaks_loop_after_terminal_measurement:{name}")
    check(all((trial / f"build_{controller}.log").stat().st_size == 0 for controller in ["student_controller_cpp","logistics_supervisor_cpp"]),
          f"empty_successful_build_diagnostic_logs:{name}")
    if not negative:
        check(len(delivery) == 1 and not rejection and measured["score_at_delivery"] == expected_score, f"nominal_delivery_score:{name}")
        check(0 < measured["delivery_wall_s"] <= measured["end_wall_s"] < run["process_wall_s"], f"nominal_wall_interval_order:{name}")
        check(close(measured["end_simulation_s"]-measured["delivery_simulation_s"], 2.08), f"reverse_clear_completion_interval_2p08s:{name}")
        all_delivery_event_wall_offsets.append(measured["delivery_wall_s"]-float(delivery[0]["wall_s"]))
        nominal_box_xyz.append(measured["box_final_xyz_m"])
        separations.append(math.dist(measured["robot_final_xy_m"], measured["box_final_xyz_m"][:2]))
    else:
        check(not delivery and len(rejection) == 1 and measured["supervisor_score"] == 0 and measured["score_at_delivery"] == -1 and
              measured["delivery_simulation_s"] == measured["delivery_wall_s"] == measured["delivery_real_time_factor"] == -1,
              f"negative_control_rejected_without_delivery_sentinels:{name}")

    old = old_by_id[identity(run)]
    old_trial = original / old["trial_directory"].replace("\\","/")
    old_supervisor = old_trial / "controllers/logistics_supervisor_cpp"
    old_events = table(old_supervisor / "events.csv")
    old_trace = table(old_supervisor / "trajectory.csv")
    comparable_keys = ["delivery_simulation_s","supervisor_score","invalid_drops","evaluated_drops","box_0_delivered",
                       "success","box_0_in_outgoing_at_end","attached_index_at_end"]
    agreeing_keys = {key: old[key] == run[key] for key in comparable_keys}
    check(all(agreeing_keys.values()), f"original_outcome_delivery_time_exact_agreement:{name}")
    nonterminal_columns = ("simulation_s","event","box","machine","bay","score","assigned_delay_s")
    new_nonterminal = [{key:e[key] for key in nonterminal_columns} for e in events if e["event"] not in {"controller_finished","simulation_timeout"}]
    old_nonterminal = [{key:e[key] for key in nonterminal_columns} for e in old_events if e["event"] not in {"controller_finished","simulation_timeout"}]
    check(new_nonterminal == old_nonterminal, f"all_nonterminal_simulation_events_and_delays_exactly_match_original:{name}")
    cutoff = run["delivery_simulation_s"] if not negative else float(rejection[0]["simulation_s"])
    new_prefix = [row for row in trace if float(row["simulation_s"]) <= cutoff+1e-8]
    old_prefix = [row for row in old_trace if float(row["simulation_s"]) <= cutoff+1e-8]
    check(new_prefix == old_prefix, f"full_trajectory_through_delivery_or_rejection_exactly_matches_original:{name}")
    final_box_max_delta = max(abs(a-b) for a,b in zip(run["box_final_xyz_m"],old["box_final_xyz_m"]))
    final_robot_max_delta = max(abs(a-b) for a,b in zip(run["robot_final_xy_m"],old["robot_final_xy_m"]))
    comparisons.append({"trial":name,"outcome_fields_exactly_match":agreeing_keys,
        "all_nonterminal_events_match":new_nonterminal == old_nonterminal,"trace_through_delivery_or_rejection_exactly_matches":new_prefix == old_prefix,
        "delivery_simulation_s":run["delivery_simulation_s"], "old_end_json_s":old["end_simulation_s"],
        "old_last_trace_s":float(old_trace[-1]["simulation_s"]), "new_end_json_and_last_trace_s":run["end_simulation_s"],
        "old_final_trace_json_offset_s":old["end_simulation_s"]-float(old_trace[-1]["simulation_s"]),
        "max_box_xyz_difference_from_old_final_m":final_box_max_delta,
        "max_robot_xy_difference_from_old_final_m":final_robot_max_delta})
    trial_results.append({"trial":name,"case":case,"seed":seed,"negative_control":negative,
        "event_count":len(events),"trajectory_rows":len(trace), "terminal_simulation_s":measured["end_simulation_s"],
        "delivery_simulation_s":measured["delivery_simulation_s"],"delivery_wall_s":measured["delivery_wall_s"],
        "recomputed_rtf": measured["delivery_simulation_s"]/measured["delivery_wall_s"] if not negative else None,
        "source_reconstruction_errors":[e for row in recomputed["trials"] if row["trial"] == name for e in row["errors"]]})
    processing_total += len(processing)
    trajectory_total += len(trace)
    for relative in ["commands.json","instrumentation.diff","worlds/validation.wbt","controllers/student_controller_cpp/controller.log",
                     "controllers/logistics_supervisor_cpp/events.csv","controllers/logistics_supervisor_cpp/trajectory.csv",
                     "controllers/logistics_supervisor_cpp/measurement.json"]:
        path = trial / relative
        checksums[path.relative_to(base).as_posix()] = digest(path)

# CSV export is checked by typed values rather than formatting of booleans/floats.
csv_runs = table(base / "runs.csv")
check(len(csv_runs) == len(runs), "runs_csv_count")
for json_row, csv_row in zip(runs,csv_runs):
    same = True
    for key, value in csv_row.items():
        reference = json_row[key]
        if isinstance(reference, bool):
            same &= value == str(reference)
        elif isinstance(reference, (int,float)):
            same &= close(value, reference, 1e-9)
        else:
            same &= value == reference
    check(same, f"runs_csv_all_columns_match_json:{json_row['case']}_{json_row['seed']}_{json_row['negative_control']}")

summary = load(base / "summary.json")
check(len(summary) == 3 and {row["case"] for row in summary} == {"blue","green","red"}, "summary_case_matrix")
summary_results = []
for row in summary:
    selected = [r for r in trial_results if r["case"] == row["case"] and not r["negative_control"]]
    check(row["trials"] == row["successful"] == len(selected) == 5 and row["invalid_drops"] == 0, f"summary_sample_success_counts:{row['case']}")
    derived = {}
    for field, raw_field in [("delivery_simulation_s","delivery_simulation_s"),("end_simulation_s","terminal_simulation_s"),
                             ("delivery_real_time_factor","recomputed_rtf")]:
        values = [run[raw_field] for run in selected]
        wanted = {"mean":statistics.mean(values),"sample_sd":statistics.stdev(values),"min":min(values),"max":max(values)}
        check(all(close(row[field][key], value) for key,value in wanted.items()), f"summary_raw_recomputation:{row['case']}:{field}")
        derived[field] = wanted
    summary_results.append({"case":row["case"],"n":len(selected),"independent_statistics":derived})

for relative in ["manifest.json","runs.json","runs.csv","summary.json"]:
    checksums[relative] = digest(base / relative)
output = {"audit_utc":datetime.now(timezone.utc).isoformat(),"canonical_batch":str(base),"original_batch":str(original),
          "archive_hashes_required":True,"current_match_required":args.require_current_match,
          "read_only_archive_audit":True,"check_count":len(checks),"errors":errors,"checks":checks,
          "sample_counts":{"all_trials":len(runs),"nominal_trials":15,"negative_controls":1,
                           "all_trajectory_samples":trajectory_total,"processing_lifecycles":processing_total},
          "source_provenance":source_details,"trial_terminal_results":trial_results,"original_comparison":comparisons,
          "summary_recomputed":summary_results,
          "final_nominal_box_xyz_ranges_m":[[min(row[i] for row in nominal_box_xyz),max(row[i] for row in nominal_box_xyz)] for i in range(3)],
          "final_nominal_robot_box_separation_range_m":[min(separations),max(separations)],
          "delivery_wall_event_offset_range_s":[min(all_delivery_event_wall_offsets),max(all_delivery_event_wall_offsets)],
          "core_evidence_sha256":checksums}
args.result.parent.mkdir(parents=True, exist_ok=True)
args.result.write_text(json.dumps(output,indent=2)+"\n",encoding="utf-8")
print(json.dumps({"check_count":len(checks),"errors":errors,"sample_counts":output["sample_counts"],
                  "all_terminal_offsets_s":[r["last_trace_to_measurement_offset_s"] for r in recomputed["trials"]],
                  "summary_recomputed":summary_results},indent=2))
raise SystemExit(bool(errors))
