"""Build an isolated Webots regression world: python tests/navigation/prepare.py red RRGB [bay].

Run the printed world with Webots --batch --mode=fast --no-rendering. The
observer writes evidence.log and trajectory.csv in its controller directory
and exits Webots with a failing status for collisions, timeout or bad score.
The user's world, order settings and demo selectors are never rewritten.
"""
from pathlib import Path
import os
import subprocess
import sys

root = Path(__file__).resolve().parents[2]
case, order = sys.argv[1:3]
bay = int(sys.argv[3]) if len(sys.argv) > 3 else 0
assert case.replace('_', '').isalnum() and len(order) == 4 and set(order) <= set('RGB') and bay in (0, 1)
project = root / 'tmp' / 'navigation_qa' / case
webots = Path(os.environ.get('WEBOTS_HOME', 'C:/Program Files/Webots'))
windows = os.name == 'nt'
compiler = webots / 'msys64/mingw64/bin/g++.exe' if windows else 'g++'
env = os.environ.copy()
if windows:
    env['PATH'] = os.pathsep.join([str(webots / 'msys64/mingw64/bin/cpp'),
                                 str(webots / 'msys64/mingw64/bin'), env['PATH']])


def build(folder, sources):
    subprocess.run([str(compiler), '-std=gnu++17', '-Wall', '-Wextra', '-O2', '-D_GLIBCXX_USE_CXX11_ABI=1',
                    '-I' + str(webots / 'include/controller/cpp'), *sources,
                    '-L' + str(webots / 'lib/controller'), '-lCppController', '-lController',
                    '-o', str(folder / (folder.name + ('.exe' if windows else '')))],
                   cwd=folder, env=env, check=True)


for controller, sources in [('student_controller_cpp', ['student_controller.cpp', 'robot_navigation.cpp']),
                            ('logistics_supervisor_cpp', ['logistics_supervisor_cpp.cpp'])]:
    folder = project / 'controllers' / controller
    folder.mkdir(parents=True, exist_ok=True)
    for source in (root / 'controllers' / controller).iterdir():
        if source.suffix not in ('.hpp', '.cpp'):
            continue
        content = source.read_text()
        if source.name == 'logistics_supervisor_cpp.cpp':
            content = content.replace('constexpr int kTaskOrderMode = kOrderModeRandom;',
                                      'constexpr int kTaskOrderMode = kOrderModeManual;')
            content = content.replace('constexpr const char *kManualOrder = "RRGB";',
                                      f'constexpr const char *kManualOrder = "{order}";')
            content = content.replace('rng_(static_cast<unsigned int>(std::time(nullptr)))', 'rng_(123)')
        if source.name == 'student_controller.cpp':
            content = content.replace('nav->init();', 'nav->init(); std::freopen("controller.log", "w", stdout);')
            for machine in ('A', 'B'):
                content = content.replace(f'constexpr int DEMO_{machine}_BAY = 0;',
                                          f'constexpr int DEMO_{machine}_BAY = {bay};')
        (folder / source.name).write_text(content)
    build(folder, sources)

observer = project / 'controllers' / 'navigation_observer'
observer.mkdir(parents=True, exist_ok=True)
(observer / 'observer.cpp').write_bytes(Path(__file__).with_name('observer.cpp').read_bytes())
build(observer, ['observer.cpp'])
world = (root / 'worlds/logistics_pbl_enu.wbt').read_text()
world += '\nRobot { children [ Receiver { name "task_rx" channel 2 } ] controller "navigation_observer" supervisor TRUE }\n'
(project / 'worlds').mkdir(exist_ok=True)
(project / 'worlds/navigation.wbt').write_text(world)
print(project / 'worlds/navigation.wbt')
