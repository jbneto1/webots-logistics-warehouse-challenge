"""Check the map collision model and broad-arc geometry without starting Webots."""
from pathlib import Path
import os
import re
import subprocess

root = Path(__file__).resolve().parents[2]
world = (root / 'worlds/logistics_pbl_enu.wbt').read_text()
modeled = (root / 'controllers/student_controller_cpp/warehouse_clearance.hpp').read_text()
modeled = modeled.split('WAREHOUSE_WALLS[] = {', 1)[1].split('};', 1)[0]
rectangles = [tuple(map(float, values.split(','))) for values in re.findall(r'\{([^{}]+)\}', modeled)]
actual = []
for name, body in re.findall(r'^DEF (\w+) Solid \{\n(.*?)^\}', world, re.M | re.S):
    if name == 'SHOP_FLOOR' or name.startswith('BOX_'):
        continue
    position = re.search(r'^  translation ([^\n]+)', body, re.M)
    collider = re.search(r'boundingObject Box \{\s*size ([^\n]+)', body)
    if not position or not collider or re.search(r'^  rotation ', body, re.M):
        raise AssertionError(f'Update the planner collision model for {name}')
    x, y, _ = map(float, position[1].split())
    width, height, _ = map(float, collider[1].split())
    actual.append((x, y, width / 2, height / 2))
assert sorted(actual) == sorted(rectangles), 'Planner walls differ from the Webots world'
print(f'Collision model matches all {len(actual)} static walls.', flush=True)

webots = Path(os.environ.get('WEBOTS_HOME', 'C:/Program Files/Webots'))
windows = os.name == 'nt'
compiler = webots / 'msys64/mingw64/bin/g++.exe' if windows else 'g++'
env = os.environ.copy()
if windows:
    env['PATH'] = os.pathsep.join([str(webots / 'msys64/mingw64/bin/cpp'),
                                 str(webots / 'msys64/mingw64/bin'), str(webots / 'lib/controller'), env['PATH']])
else:
    env['LD_LIBRARY_PATH'] = str(webots / 'lib/controller') + os.pathsep + env.get('LD_LIBRARY_PATH', '')
output = root / 'tmp/navigation_qa'
output.mkdir(parents=True, exist_ok=True)
executable = output / ('planner_checks.exe' if windows else 'planner_checks')
subprocess.run([str(compiler), '-std=gnu++17', '-Wall', '-Wextra', '-O2', '-D_GLIBCXX_USE_CXX11_ABI=1',
                '-I' + str(webots / 'include/controller/cpp'), '-Icontrollers/student_controller_cpp',
                'tests/navigation/planner_checks.cpp', 'controllers/student_controller_cpp/robot_navigation.cpp',
                '-L' + str(webots / 'lib/controller'), '-lCppController', '-lController', '-o', str(executable)],
               cwd=root, env=env, check=True)
subprocess.run([str(executable)], cwd=root, env=env, check=True)
