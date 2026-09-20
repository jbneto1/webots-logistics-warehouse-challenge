"""Verify completed physics runs and precise arrival poses: verify_run.py CASE [bay]."""
from pathlib import Path
import math
import re
import sys

root = Path(__file__).resolve().parents[2]
case = root / 'tmp/navigation_qa' / sys.argv[1]
bay = int(sys.argv[2]) if len(sys.argv) > 2 else 0
evidence = (case / 'controllers/navigation_observer/evidence.log').read_text()
assert evidence.startswith('PASS '), evidence
log = (case / 'controllers/student_controller_cpp/controller.log').read_text()
expected = {
    'PICK_BOX_0': (-0.695, 0.425, math.pi / 2),
    'DROP_AT_MACHINE_A': (-0.520, -0.150 * bay, 0.0),
    'WAIT_FOR_MACHINE_A_READY': (0.0, -0.150 * bay, math.pi),
    'PICK_FROM_MACHINE_A': (-0.155, -0.150 * bay, math.pi),
    'DROP_AT_MACHINE_B': (0.175, 0.150 * bay, 0.0),
    'WAIT_FOR_MACHINE_B_READY': (0.695, 0.150 if bay else -0.010, math.pi),
    'PICK_FROM_MACHINE_B': (0.535, 0.150 if bay else -0.010, math.pi),
    'DROP_AT_OUTGOING': (0.245, -0.410, -math.pi / 2),
}
seen = set()
errors, heading_errors = [], []
for state, values in re.findall(r'^State: (\w+) t=[\d.]+ pose=\(([^)]+)\)', log, re.M):
    if state not in expected:
        continue
    seen.add(state)
    x, y, theta = map(float, values.split(','))
    gx, gy, gt = expected[state]
    error = math.hypot(x - gx, y - gy)
    angle = abs(math.remainder(theta - gt, 2 * math.pi))
    assert error <= 0.0125, f'{state}: position error {error:.4f} m'
    assert angle <= 0.065, f'{state}: heading error {angle:.4f} rad'
    errors.append(error)
    heading_errors.append(angle)
required = {'PICK_BOX_0', 'DROP_AT_OUTGOING'}
box_type = re.search(r'BOX_0 type is ([RGB])', log)[1]
if box_type in 'RG':
    required |= {'DROP_AT_MACHINE_B', 'WAIT_FOR_MACHINE_B_READY', 'PICK_FROM_MACHINE_B'}
if box_type == 'R':
    required |= {'DROP_AT_MACHINE_A', 'WAIT_FOR_MACHINE_A_READY', 'PICK_FROM_MACHINE_A'}
assert required <= seen, f'Missing arrival checks: {required - seen}'
print(sys.argv[1], evidence.strip(),
      f'max_arrival_error={1000 * max(errors):.1f}mm max_heading_error={math.degrees(max(heading_errors)):.2f}deg')
