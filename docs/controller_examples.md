# Student Controller Examples

The active world uses the C++ controllers:

- Robot controller: `student_controller_cpp`
- Supervisor controller: `logistics_supervisor_cpp`

The repository also includes two compact student-controller examples:

| Language | Webots controller name | Main file |
| --- | --- | --- |
| C | `example_student_c` | `controllers/example_student_c/example_student_c.c` |
| Python | `example_student_python` | `controllers/example_student_python/example_student_python.py` |

Each example waits for `POSE` and `ORDER`, picks `BOX_0`, clears the incoming warehouse pocket, and delivers the box only when it is already blue/final (`B`). If `BOX_0` is red or green, the example stops and reports that a machine route is required.

## Switching Examples

In Webots, select the `MOBILE_ROBOT` node and change its `controller` field to one of:

```text
student_controller_cpp
example_student_c
example_student_python
```

Keep the supervisor controller as `logistics_supervisor_cpp`.

## Building

The C++ standard controller and C example build with Webots' standard Makefile flow:

```bash
cd controllers/student_controller_cpp
make
cd ../example_student_c
make
```

The Python example does not need a Makefile. Webots runs `example_student_python.py` directly.

## API Surface

- Standard C++ controller: uses the project `Navigation` wrapper from `controllers/student_controller_cpp/robot_navigation.cpp` and `warehouse_map.hpp`.
- C example: uses the Webots C API directly with motors, receiver/emitter, and `wb_connector_lock` / `wb_connector_unlock`.
- Python example: uses the Webots Python API directly with `Robot`, wheel `Motor`s, `Receiver.getString()`, `Emitter.send()`, and `Connector.lock()` / `Connector.unlock()`.

For the standard C++ controller, set `DEBUG_LEVEL` in `controllers/student_controller_cpp/debug_config.hpp` to switch between state-only logs and detailed route/pose/speed telemetry.
