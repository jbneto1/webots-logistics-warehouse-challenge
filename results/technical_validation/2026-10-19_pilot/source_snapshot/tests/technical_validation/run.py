"""Reproducible, isolated Webots BOX_0 measurements; see README.md beside this file."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import difflib
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
CASES = {"blue": ("BRRG", 1), "green": ("GRRB", 2), "red": ("RRGB", 3)}


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError(f"Instrumentation anchor must occur once: {old!r}")
    return text.replace(old, new, 1)


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def instrument(name, text, order, seed, expected, timeout, negative):
    if name == "student_controller.cpp":
        text = replace_once(text, '#include <cstdio>', '#include <cstdio>\n#include <webots/robot.h>')
        text = replace_once(text, '  state = nextState;',
                            '  state = nextState;\n  if (state == FINISHED)\n'
                            '    wb_robot_set_custom_data("technical_validation_finished");')
        # Redirect without changing the existing debug level or motion logic.
        text = replace_once(text, '  nav->init();',
                            '  if (!std::freopen("controller.log", "w", stdout)) return 2;\n  nav->init();')
        if negative:
            text = replace_once(text, '      case DECIDE_BOX_0_DESTINATION:',
                                '      case DECIDE_BOX_0_DESTINATION:\n'
                                '        nav->magnetDrop();\n'
                                '        if (nav->wait(0.5)) changeState(FINISHED);\n'
                                '        break;\n')
    elif name == "logistics_supervisor_cpp.cpp":
        text = replace_once(text, '#include "warehouse_overlay.hpp"',
                            '#include "warehouse_overlay.hpp"\n'
                            f'#define VALIDATION_EXPECTED_SCORE {expected}\n'
                            f'#define VALIDATION_TIMEOUT_S {timeout!r}\n'
                            '#include "measurement.hpp"')
        text = replace_once(text, 'constexpr int kTaskOrderMode = kOrderModeRandom;',
                            'constexpr int kTaskOrderMode = kOrderModeManual;')
        text = replace_once(text, 'constexpr const char *kManualOrder = "RRGB";',
                            f'constexpr const char *kManualOrder = "{order}";')
        text = replace_once(text, 'rng_(static_cast<unsigned int>(std::time(nullptr)))', f'rng_({seed}u)')
        text = replace_once(text, '    initializeOrderAndBoxes();',
                            '    initializeOrderAndBoxes();\n    validation_.begin(robot_.getTime());')
        text = replace_once(text, '      updateOverlay();',
                            '      updateOverlay();\n'
                            '      validation_.step(robot_, mobile_, boxes_[0].node, score_, boxes_[0].delivered, attachedIndex_);')
        text = replace_once(text, '    const double delay = randomProcessingTime();',
                            '    const double delay = randomProcessingTime();\n'
                            '    validation_.event(now, "processing_start", bay.inputBox, bay.machine, bay.bayIndex, score_, delay);')
        text = replace_once(text, '    bay.inputBox = index;',
                            '    validation_.event(robot_.getTime(), "machine_accepted", index, machine, bay.bayIndex, score_);\n'
                            '    bay.inputBox = index;')
        text = replace_once(text, '      attachedIndex_ = best;',
                            '      attachedIndex_ = best;\n'
                            '      validation_.event(robot_.getTime(), "pickup", best, \'?\', -1, score_);')
        text = replace_once(text, '  void evaluateDrop(int index) {',
                            '  void evaluateDrop(int index) {\n    validation_.dropAttempt();')
        text = replace_once(text, '      box.delivered = true;\n      ++score_;',
                            '      box.delivered = true;\n      ++score_;\n'
                            '      validation_.delivery(robot_.getTime(), index, score_);')
        text = replace_once(text, '    recordOverlayEvent(box.boxDef + " drop rejected: " + rejection, true);',
                            '    validation_.reject(robot_.getTime(), index, score_);\n'
                            '    recordOverlayEvent(box.boxDef + " drop rejected: " + rejection, true);')
        text = replace_once(text, '      box.state = box.targetState;',
                            '      validation_.event(now, "processing_ready", i, bay->machine, bay->bayIndex, score_);\n'
                            '      box.state = box.targetState;')
        text = replace_once(text, '  webots::Supervisor robot_;',
                            '  ValidationMeasurement validation_;\n  webots::Supervisor robot_;')
    return text


def build_project(project, snapshot, webots, compiler, env, order, seed, expected, timeout, negative):
    patches = []
    commands = []
    for controller, sources in [("student_controller_cpp", ["student_controller.cpp", "robot_navigation.cpp"]),
                                ("logistics_supervisor_cpp", ["logistics_supervisor_cpp.cpp"])]:
        folder = project / "controllers" / controller
        folder.mkdir(parents=True)
        for source in sorted((snapshot / "controllers" / controller).iterdir()):
            original = source.read_text(encoding="utf-8")
            changed = instrument(source.name, original, order, seed, expected, timeout, negative)
            (folder / source.name).write_text(changed, encoding="utf-8")
            patches.extend(difflib.unified_diff(original.splitlines(True), changed.splitlines(True),
                                               fromfile=f"original/{controller}/{source.name}",
                                               tofile=f"instrumented/{controller}/{source.name}"))
        if controller == "logistics_supervisor_cpp":
            shutil.copy2(snapshot / "tests/technical_validation/measurement.hpp", folder)
        command = [str(compiler), "-std=gnu++17", "-Wall", "-Wextra", "-O2", "-D_GLIBCXX_USE_CXX11_ABI=1",
                   "-I" + str(webots / "include/controller/cpp"), "-I" + str(webots / "include/controller/c"),
                   *sources, "-L" + str(webots / "lib/controller"), "-lCppController", "-lController",
                   "-o", str(folder / (controller + (".exe" if os.name == "nt" else "")))]
        commands.append(command)
        result = subprocess.run(command, cwd=folder, env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        (project / f"build_{controller}.log").write_text(result.stdout, encoding="utf-8")
        result.check_returncode()
    (project / "instrumentation.diff").write_text("".join(patches), encoding="utf-8")
    (project / "worlds").mkdir()
    world = (snapshot / "worlds/logistics_pbl_enu.wbt").read_text(encoding="utf-8")
    world = replace_once(world, "WorldInfo {", f"WorldInfo {{\n  randomSeed {seed}\n  basicTimeStep 32")
    (project / "worlds/validation.wbt").write_text(world, encoding="utf-8")
    return commands


def evidence_checks(folder, result):
    """Cross-check raw events, trajectory, state log and final supervisor state."""
    controller = folder / "controllers/logistics_supervisor_cpp"
    events = list(csv.DictReader((controller / "events.csv").open(encoding="utf-8")))
    deliveries = [e for e in events if e["event"] == "delivery" and e["box"] == "0"]
    rejections = [e for e in events if e["event"] == "invalid_drop"]
    if result["invalid_drops"] != len(rejections):
        raise ValueError("Invalid-drop count differs from events")
    if result["box_0_delivered"] != bool(deliveries):
        raise ValueError("Delivery flag differs from events")
    if deliveries and (len(deliveries) != 1 or abs(float(deliveries[0]["simulation_s"]) - result["delivery_simulation_s"]) > 1e-7):
        raise ValueError("Delivery timestamp differs from events")
    trace = list(csv.DictReader((controller / "trajectory.csv").open(encoding="utf-8")))
    if not trace or int(trace[-1]["score"]) != result["supervisor_score"]:
        raise ValueError("Final trajectory score differs from summary")
    state_log = (folder / "controllers/student_controller_cpp/controller.log").read_text(encoding="utf-8")
    if result["controller_finished"] and "State: FINISHED " not in state_log:
        raise ValueError("Missing controller FINISHED state")
    if result["success"]:
        expected = result["expected_score"]
        accepted = [e for e in events if e["event"] == "machine_accepted"]
        if len(accepted) != expected - 1 or result["evaluated_drops"] != expected:
            raise ValueError("Unexpected scored lifecycle or drop count")
        if not result["box_0_in_outgoing_at_end"] or result["attached_index_at_end"] != -1:
            raise ValueError("Final delivery geometry/attachment inconsistent")


def summarize(output, rows):
    write_json(output / "runs.json", rows)
    columns = ["case", "order", "seed", "negative_control", "success", "box_0_delivered", "supervisor_score",
               "expected_score", "delivery_simulation_s", "end_simulation_s", "invalid_drops", "evaluated_drops",
               "delivery_wall_s", "delivery_real_time_factor", "process_wall_s", "webots_exit_code"]
    with (output / "runs.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    aggregate = []
    for case in CASES:
        group = [r for r in rows if r["case"] == case and not r["negative_control"]]
        successful = [r for r in group if r.get("success")]
        entry = {"case": case, "trials": len(group), "successful": len(successful),
                 "invalid_drops": sum(r.get("invalid_drops", 0) for r in group)}
        for field in ("delivery_simulation_s", "end_simulation_s", "delivery_real_time_factor"):
            values = [r[field] for r in successful]
            entry[field] = {"mean": statistics.mean(values), "sample_sd": statistics.stdev(values) if len(values) > 1 else None,
                            "min": min(values), "max": max(values)} if values else None
        aggregate.append(entry)
    write_json(output / "summary.json", aggregate)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New evidence directory; never overwritten")
    parser.add_argument("--webots-home", type=Path, default=Path(os.environ.get("WEBOTS_HOME", "C:/Program Files/Webots")))
    parser.add_argument("--seeds", type=int, nargs="+", default=[101, 102, 103, 104, 105])
    parser.add_argument("--cases", choices=list(CASES), nargs="+", default=list(CASES))
    parser.add_argument("--sim-timeout", type=float, default=240.0)
    parser.add_argument("--wall-timeout", type=float, default=240.0)
    parser.add_argument("--negative-control", action="store_true", help="Also deliberately release blue outside any destination")
    parser.add_argument("--prepare-only", action="store_true", help="Build isolated projects without starting Webots")
    args = parser.parse_args()
    if len(set(args.seeds)) != len(args.seeds) or any(s < 0 or s > 2147483647 for s in args.seeds):
        parser.error("Seeds must be unique integers between 0 and 2147483647")
    if len(set(args.cases)) != len(args.cases) or args.sim_timeout <= 0 or args.wall_timeout <= 0:
        parser.error("Cases must be unique and timeouts positive")
    webots = args.webots_home.resolve()
    windows = os.name == "nt"
    compiler = webots / "msys64/mingw64/bin/g++.exe" if windows else Path(shutil.which("g++") or "g++")
    executable = webots / ("msys64/mingw64/bin/webots.exe" if windows else "webots")
    env = os.environ.copy()
    env["WEBOTS_HOME"] = str(webots)
    if windows:
        env["PATH"] = os.pathsep.join([str(webots / "msys64/mingw64/bin/cpp"), str(webots / "msys64/mingw64/bin"),
                                       str(webots / "lib/controller"), env["PATH"]])
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    snapshot = output / "source_snapshot"
    files = [ROOT / "worlds/logistics_pbl_enu.wbt", HERE / "measurement.hpp", HERE / "run.py"]
    for controller in ("student_controller_cpp", "logistics_supervisor_cpp"):
        files += sorted(p for p in (ROOT / "controllers" / controller).iterdir() if p.suffix in (".cpp", ".hpp"))
    hashes = {}
    for source in files:
        relative = source.relative_to(ROOT)
        target = snapshot / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        hashes[str(relative).replace("\\", "/")] = digest(source)
    def capture(command):
        return subprocess.run(command, cwd=ROOT, env=env, text=True, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, check=True).stdout.strip()
    manifest = {"created_utc": datetime.now(timezone.utc).isoformat(), "git_head": capture(["git", "rev-parse", "HEAD"]),
                "working_tree_status": capture(["git", "status", "--short"]), "source_sha256": hashes,
                "platform": platform.platform(), "processor": platform.processor(), "python": sys.version,
                "webots_home": str(webots), "webots_version": (webots / "resources/version.txt").read_text().strip(),
                "compiler_version": capture([str(compiler), "--version"]),
                "seeds": args.seeds, "cases": args.cases, "machine_bays": {"A": 0, "B": 0},
                "debug_level": "unchanged from source snapshot", "sim_timeout_s": args.sim_timeout,
                "wall_timeout_s": args.wall_timeout, "prepared_only": args.prepare_only,
                "measurement": "Supervisor delivery acceptance; steady_clock wall interval from pre-first-step to delivery; startup/build excluded"}
    (output / "working_tree.diff").write_text(capture(["git", "diff", "--no-ext-diff", "--", "controllers", "worlds/logistics_pbl_enu.wbt"]), encoding="utf-8")
    write_json(output / "manifest.json", manifest)
    rows = []
    failed = False
    trials = [(case, seed, False) for case in args.cases for seed in args.seeds]
    if args.negative_control:
        trials.append(("blue", args.seeds[0], True))
    for case, seed, negative in trials:
        order, expected = CASES[case]
        trial = f"{case}_seed_{seed}" + ("_invalid_drop" if negative else "")
        project = output / "trials" / trial
        project.mkdir(parents=True)
        print(f"Preparing {trial}", flush=True)
        commands = build_project(project, snapshot, webots, compiler, env, order, seed, expected, args.sim_timeout, negative)
        command = [str(executable), "--batch", "--mode=fast", "--no-rendering", "--minimize", "--stdout", "--stderr",
                   str(project / "worlds/validation.wbt")]
        write_json(project / "commands.json", {"build": commands, "run": command})
        if args.prepare_only:
            continue
        print(f"Running {trial}", flush=True)
        started = time.perf_counter()
        with (project / "webots.log").open("w", encoding="utf-8") as log:
            process = subprocess.Popen(command, cwd=project, env=env, stdout=log, stderr=subprocess.STDOUT)
            try:
                code = process.wait(timeout=args.wall_timeout)
            except subprocess.TimeoutExpired:
                # Stop only the process tree created by this trial.
                if windows:
                    subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"], stdout=log, stderr=subprocess.STDOUT)
                else:
                    process.kill()
                process.wait()
                code = -999
        elapsed = time.perf_counter() - started
        result_file = project / "controllers/logistics_supervisor_cpp/measurement.json"
        row = {"case": case, "order": order, "seed": seed, "negative_control": negative,
               "process_wall_s": elapsed, "webots_exit_code": code, "trial_directory": str(project.relative_to(output))}
        if result_file.exists():
            row.update(json.loads(result_file.read_text(encoding="utf-8")))
            evidence_checks(project, row)
            ok = (code == 1 and not row["success"] and row["invalid_drops"] == 1 and not row["box_0_delivered"]
                  and row["controller_finished"] and row["supervisor_score"] == 0) if negative else (code == 0 and row["success"])
        else:
            row.update(success=False, error="No supervisor measurement file; inspect webots.log")
            ok = False
        row["validation_check_passed"] = ok
        failed |= not ok
        rows.append(row)
        summarize(output, rows)
        print(f"{trial}: check={'PASS' if ok else 'FAIL'} delivery={row.get('delivery_simulation_s')} score={row.get('supervisor_score')} invalid={row.get('invalid_drops')}", flush=True)
    manifest["source_unchanged_after_run"] = all(digest(ROOT / relative) == sha for relative, sha in hashes.items())
    manifest["completed_utc"] = datetime.now(timezone.utc).isoformat()
    manifest["validation_checks_passed"] = not failed and not args.prepare_only
    write_json(output / "manifest.json", manifest)
    if not manifest["source_unchanged_after_run"]:
        raise RuntimeError("Source files changed during this run")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
