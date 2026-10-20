"""Regression checks for finalization evidence; no Webots installation is needed."""
import copy
import csv
from pathlib import Path
import tempfile
import unittest

from run import evidence_checks


class FinalizationEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.folder = Path(self.temporary.name)
        self.events = [
            {"simulation_s": 0.0, "event": "start", "box": -1},
            {"simulation_s": 1.0, "event": "pickup", "box": 0},
            {"simulation_s": 2.0, "event": "delivery", "box": 0},
            {"simulation_s": 4.0, "event": "controller_finished", "box": 0},
        ]
        self.result = {
            "invalid_drops": 0, "box_0_delivered": True,
            "delivery_simulation_s": 2.0, "end_simulation_s": 4.0,
            "controller_finished": True, "supervisor_score": 1,
            "success": True, "expected_score": 1, "evaluated_drops": 1,
            "box_0_in_outgoing_at_end": True, "attached_index_at_end": -1,
            "robot_final_xy_m": [0.3, -0.2],
            "box_final_xyz_m": [0.24, -0.52, 0.026],
        }
        self.trace_time = 4.0
        self.trace = {"simulation_s": 4.0, "score": 1, "attached_index": -1,
                      "box_0_delivered": 1, "robot_x_m": 0.3, "robot_y_m": -0.2,
                      "box_x_m": 0.24, "box_y_m": -0.52, "box_z_m": 0.026}

    def check_records(self):
        controller = self.folder / "controllers/logistics_supervisor_cpp"
        student = self.folder / "controllers/student_controller_cpp"
        controller.mkdir(parents=True, exist_ok=True)
        student.mkdir(parents=True, exist_ok=True)
        with (controller / "events.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=["simulation_s", "event", "box"])
            writer.writeheader()
            writer.writerows(self.events)
        with (controller / "trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(self.trace))
            writer.writeheader()
            writer.writerow(dict(self.trace, simulation_s=self.trace_time))
        (student / "controller.log").write_text("State: FINISHED \n", encoding="utf-8")
        evidence_checks(self.folder, self.result)

    def test_consistent_delivery_records_pass(self):
        self.check_records()

    def test_json_overwritten_one_tick_later_is_rejected(self):
        self.result["end_simulation_s"] += 0.032
        with self.assertRaisesRegex(ValueError, "Final trajectory timestamp"):
            self.check_records()

    def test_shifted_terminal_event_is_rejected(self):
        self.events[-1]["simulation_s"] += 0.032
        with self.assertRaisesRegex(ValueError, "Terminal event timestamp"):
            self.check_records()

    def test_missing_terminal_event_is_rejected(self):
        self.events.pop()
        with self.assertRaisesRegex(ValueError, "exactly one matching terminal event"):
            self.check_records()

    def test_repeated_finalization_event_is_rejected(self):
        self.events.append(copy.deepcopy(self.events[-1]))
        with self.assertRaisesRegex(ValueError, "exactly one matching terminal event"):
            self.check_records()

    def test_events_after_finalization_are_rejected(self):
        self.events.append({"simulation_s": 4.032, "event": "pickup", "box": 1})
        with self.assertRaisesRegex(ValueError, "exactly one matching terminal event"):
            self.check_records()

    def test_wrong_terminal_state_is_rejected(self):
        self.events[-1]["event"] = "simulation_timeout"
        with self.assertRaisesRegex(ValueError, "exactly one matching terminal event"):
            self.check_records()

    def test_consistent_timeout_records_pass(self):
        self.result.update(success=False, controller_finished=False)
        self.events[-1]["event"] = "simulation_timeout"
        self.check_records()

    def test_consistent_invalid_drop_control_passes(self):
        self.events[2]["event"] = "invalid_drop"
        self.result.update(success=False, box_0_delivered=False, supervisor_score=0,
                           invalid_drops=1, delivery_simulation_s=-1.0,
                           box_0_in_outgoing_at_end=False, box_final_xyz_m=[-0.6, -0.2, 0.026])
        self.trace.update(score=0, box_0_delivered=0, box_x_m=-0.6, box_y_m=-0.2)
        self.check_records()

    def test_final_attachment_mismatch_is_rejected(self):
        self.trace["attached_index"] = 0
        with self.assertRaisesRegex(ValueError, "Final trajectory attachment"):
            self.check_records()

    def test_final_delivery_flag_mismatch_is_rejected(self):
        self.trace["box_0_delivered"] = 0
        with self.assertRaisesRegex(ValueError, "Final trajectory delivery flag"):
            self.check_records()

    def test_final_position_mismatch_is_rejected(self):
        self.trace["box_x_m"] += 0.001
        with self.assertRaisesRegex(ValueError, "Final trajectory position"):
            self.check_records()

    def test_nonfinite_final_position_is_rejected(self):
        self.trace["box_x_m"] = float("nan")
        with self.assertRaisesRegex(ValueError, "Final trajectory position"):
            self.check_records()

    def test_outgoing_flag_mismatch_is_rejected(self):
        self.result["box_0_in_outgoing_at_end"] = False
        with self.assertRaisesRegex(ValueError, "Outgoing-region flag"):
            self.check_records()


if __name__ == "__main__":
    unittest.main()
