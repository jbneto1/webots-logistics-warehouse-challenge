# Webots Logistics PBL Warehouse Challenge

A Webots R2025a project-based learning challenge for teaching mobile robot navigation, finite-state control, task sequencing, and logistics automation in C.

The repository provides a complete simulated warehouse cell with an incoming warehouse, two processing machines, an outgoing warehouse, a mobile robot, four boxes, and a C logistics supervisor. Students mainly work in the robot controller while the supervisor manages the order, machine processing, scoring, box state, and the virtual magnet required for the simulation to behave correctly.

The project is inspired by [RobotAtFactory Lite](https://github.com/P33a/RobotAtFactoryLite).

## Contents

- [Learning Goal](#learning-goal)
- [Requirements](#requirements)
- [Quick Start](#quick-start)
- [Repository Layout](#repository-layout)
- [Challenge Rules](#challenge-rules)
- [Architecture](#architecture)
- [Student API](#student-api)
- [Adapting the Difficulty](#adapting-the-difficulty)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)
- [License](#license)
- [Author](#author)

## Learning Goal

Students develop a controller that moves boxes through a simplified production flow:

```text
R -> Machine A -> G -> Machine B -> B -> Outgoing
G -> Machine B -> B -> Outgoing
B -> Outgoing
```

The starter controller solves the route for `BOX_0`. A typical assignment asks students to generalize the same state-machine pattern to all four boxes, handle different orders, choose machine bays, and improve the route strategy.

## Requirements

- Webots R2025a
- C compiler and GNU Make supported by Webots
- Windows, Linux, or macOS with a working Webots C controller toolchain

The controller Makefiles include Webots' standard `resources/Makefile.include`. On Windows, they default `WEBOTS_HOME` to `C:/PROGRA~1/Webots` when the variable is not already set.

## Quick Start

1. Open Webots R2025a.
2. Open `worlds/logistics_pbl_enu.wbt`.
3. Build the controllers if Webots does not build them automatically:

   ```bash
   cd controllers/logistics_supervisor_c
   make
   cd ../student_controller
   make
   ```

4. Press Play in Webots.
5. Watch the simulation overlay and the controller console output.

The `logistics_supervisor_c` controller should remain active in the world. It is responsible for order generation, machine behavior, scoring, valid drop detection, and virtual magnet attachment.

## Repository Layout

```text
webots_logistics_pbl/
├── controllers/
│   ├── logistics_supervisor_c/
│   │   ├── logistics_supervisor_c.c
│   │   └── Makefile
│   └── student_controller/
│       ├── student_controller.c
│       ├── robot_navigation.c
│       ├── robot_navigation.h
│       ├── warehouse_map.h
│       └── Makefile
├── worlds/
│   └── logistics_pbl_enu.wbt
├── LICENSE
└── README.md
```

| Path | Purpose |
| --- | --- |
| `worlds/logistics_pbl_enu.wbt` | Main Webots world with the robot, boxes, warehouses, machines, and floor map. |
| `controllers/logistics_supervisor_c/` | Required C supervisor for orders, box state, scoring, processing, and magnet behavior. |
| `controllers/student_controller/student_controller.c` | Starter finite-state controller and main file for student work. |
| `controllers/student_controller/robot_navigation.*` | High-level navigation, magnet, and supervisor-message API. |
| `controllers/student_controller/warehouse_map.h` | Named robot-center poses for warehouses, machines, clear points, and route nodes. |

## Challenge Rules

The supervisor creates a four-box order. Each box starts in the incoming warehouse as one of three part types:

| Type | Meaning | Required route |
| --- | --- | --- |
| `R` | Raw part | Machine A, then Machine B, then outgoing warehouse |
| `G` | Intermediate part | Machine B, then outgoing warehouse |
| `B` | Final part | Outgoing warehouse |

The supervisor awards one point when a box is accepted by a valid machine input or delivered to the outgoing warehouse. It also controls randomized or manually configured orders, machine delays, output bay readiness, box colors, collision state, physics resets, and the simulation overlay.

## Architecture

### Logistics Supervisor

`controllers/logistics_supervisor_c/logistics_supervisor_c.c` is a C-only Webots supervisor. It sends task information to the robot controller and receives magnet commands from it.

Supervisor messages include:

```text
START
ORDER RRGB
POSE x y theta score attached_box machine_a_ready_mask machine_b_ready_mask
READY A box_index bay_index robot_x robot_y robot_theta
READY B box_index bay_index robot_x robot_y robot_theta
CLEAR A box_index
CLEAR B box_index
```

Students normally do not parse these messages directly. `robot_navigation.c` converts them into helper functions.

### Student Controller

`controllers/student_controller/student_controller.c` is a beginner-friendly finite-state machine. It demonstrates how to:

- wait for the robot pose and order;
- pick a box from the incoming warehouse;
- branch based on the box type;
- use machine input, output, approach, and clear poses;
- wait for machine-ready events;
- deliver the processed box to the outgoing warehouse.

Movement functions are non-blocking. Call them every simulation tick until they return `true`, then transition to the next state.

```c
case ROUTE_TO_OUTGOING:
  if (nav_go_to_pose(MAP_OUT_DROP[0].x, MAP_OUT_DROP[0].y, MAP_OUT_DROP[0].theta))
    change_state(DROP_AT_OUTGOING);
  break;
```

## Student API

Student code should include `warehouse_map.h`, which also exposes the navigation API:

```c
#include "warehouse_map.h"
```

Useful API groups:

| Group | Functions |
| --- | --- |
| Lifecycle | `nav_init`, `nav_step`, `nav_stop`, `nav_reset_actions` |
| Status | `nav_pose_valid`, `nav_pose`, `nav_last_order`, `nav_score`, `nav_attached_box`, `nav_normalize_angle` |
| Movement | `nav_go_to`, `nav_go_through`, `nav_go_to_pose`, `nav_rotate_to`, `nav_wait`, `nav_back_up`, `nav_back_to`, `nav_move_arc`, `nav_move_circle` |
| Magnet | `magnet_pick`, `magnet_drop`, `magnet_is_on` |
| Machine readiness | `nav_machine_a_ready`, `nav_machine_b_ready`, ready bay helpers, and ready pose helpers |
| Map constants | `MAP_IN_PICK`, `MAP_OUT_DROP`, `MAP_MACHINE_A_*`, `MAP_MACHINE_B_*`, route nodes, and heading constants |

The world uses an ENU-style floor plane: `X` points east/right, `Y` points north/up, `Z` is height, and `theta` is the robot yaw around `+Z`.

## Adapting the Difficulty

This challenge can be adjusted to different higher-education student levels:

- Keep the provided navigation API for introductory robotics students so they can focus on finite-state logic, sequencing, and debugging.
- Partially hide or simplify `robot_navigation.c` so students must implement selected behaviors such as pose control, waypoint following, or message handling.
- Omit the high-level API for advanced students and ask them to develop navigation, supervisor communication, and control logic from scratch.
- Keep the logistics supervisor in use for all variants unless the assignment explicitly replaces the simulation rules. The supervisor is the source of truth for orders, scoring, machine processing, and magnet behavior.

Possible extensions include generalizing from one box to all four boxes, selecting machine bays dynamically, adding route planning through graph nodes, smoothing selected turns with `nav_move_arc`, and comparing strategies by final score and execution time.

## Configuration

### Order Mode

The order mode is configured near the top of `controllers/logistics_supervisor_c/logistics_supervisor_c.c`:

```c
#define ORDER_MODE_RANDOM 0
#define ORDER_MODE_MANUAL 1
#define TASK_ORDER_MODE ORDER_MODE_RANDOM
#define MANUAL_ORDER "RRGB"
```

Use random mode for normal exercises and manual mode when testing a specific order.

### Navigation Tuning

Main movement tuning constants are in `controllers/student_controller/robot_navigation.c`, including:

- `MAX_WHEEL_SPEED_RAD_S`
- `POSITION_TOLERANCE_M`
- `BACK_POSITION_TOLERANCE_M`
- `THROUGH_TOLERANCE_M`
- `ANGLE_TOLERANCE_RAD`
- `FINAL_SLOWDOWN_RADIUS_M`
- `FINAL_MAX_LINEAR_M_S`
- `THROUGH_MAX_LINEAR_M_S`
- `ARC_LINEAR_SPEED_M_S`

Tune these values conservatively and verify that higher speeds do not cause collisions at machine inputs, machine outputs, or warehouse pockets.

## Troubleshooting

| Symptom | Likely cause | Fix |
| --- | --- | --- |
| Robot stops at every route point | Intermediate nodes use stop-and-align movement. | Use `nav_go_through` for route waypoints and reserve `nav_go_to_pose` for final service poses. |
| Robot turns inside a warehouse pocket or bay | The route is missing a clear/reverse step. | Use `nav_back_to` before turning away from narrow spaces. |
| Box is dropped but not accepted | Wrong box type or invalid drop location. | Check the current box state and target pose. |
| Machine-ready mask stays at `0` | No processed box is waiting at that output. | Confirm that the box was accepted by the correct machine and wait for the ready event. |
| Robot cannot attach a box in a machine input | Input boxes are intentionally not attachable. | Wait for processing to finish and pick the box from the output bay. |

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE).

## Author

Created by [@jbneto1](https://github.com/jbneto1) for a robotics project-based learning activity at the Polytechnic Institute of Braganca.
