# Webots Logistics PBL Warehouse Challenge

A Webots R2025a warehouse logistics project for teaching mobile robot navigation, finite-state control, and task sequencing in C. The project is inspired by inspired by [RobotAtFactory Lite](https://github.com/P33a/RobotAtFactoryLite).

The project provides a complete simulated logistics cell with an incoming warehouse, two processing machines, an outgoing warehouse, a mobile robot, four boxes, and a C supervisor that manages orders, scoring, machine processing, and a virtual magnet. Students work primarily in a high-level robot controller instead of writing low-level Webots motor code. However, the challenge-based learning project can be adjusted for different difficulty levels where the professor/teacher can omit the navigation API so that the students need to develop everything from scratch.

## Contents

- [Requirements](#requirements)
- [Quick Start](#quick-start)
- [Repository Layout](#repository-layout)
- [How the Challenge Works](#how-the-challenge-works)
- [Architecture](#architecture)
- [Student Controller API](#student-controller-api)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)
- [License](#license)

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
| `worlds/logistics_pbl_enu.wbt` | Main Webots world with the robot, boxes, warehouses, machines, and floor guide map. |
| `controllers/logistics_supervisor_c/` | C supervisor that owns task generation, box state, machine logic, scoring, and messaging. |
| `controllers/student_controller/student_controller.c` | Starter finite-state controller. This is the main file students extend. |
| `controllers/student_controller/robot_navigation.*` | High-level navigation, magnet, and supervisor-message API. |
| `controllers/student_controller/warehouse_map.h` | Named robot-center poses for warehouses, machines, clear points, and route nodes. |

## How the Challenge Works

The supervisor creates a four-box order. Each box starts in the incoming warehouse as one of three part types:

| Box type | Meaning | Required route |
| --- | --- | --- |
| `R` | Raw part | Machine A, then Machine B, then outgoing warehouse |
| `G` | Intermediate part | Machine B, then outgoing warehouse |
| `B` | Final part | Outgoing warehouse |

Processing changes the box state:

```text
R -> Machine A -> G -> Machine B -> B -> Outgoing
G -> Machine B -> B -> Outgoing
B -> Outgoing
```

The supervisor awards one point when a box is accepted by a valid machine input or delivered to the outgoing warehouse. It also controls:

- randomized or manually configured orders;
- virtual magnet attachment and release;
- machine processing delays;
- machine-ready events;
- output bay occupancy;
- box colors, placement, collision, and physics reset;
- simulation overlay status.

The starter controller solves only `BOX_0`. A typical assignment is to generalize the same state-machine pattern to `BOX_1`, `BOX_2`, and `BOX_3`.

## Architecture

### Supervisor

`controllers/logistics_supervisor_c/logistics_supervisor_c.c` is a C-only Webots supervisor. It sends task information to the robot controller and receives magnet commands from it.

Messages sent by the supervisor include:

```text
START
ORDER RRGB
POSE x y theta score attached_box machine_a_ready_mask machine_b_ready_mask
READY A box_index bay_index robot_x robot_y robot_theta
READY B box_index bay_index robot_x robot_y robot_theta
CLEAR A box_index
CLEAR B box_index
```

Students normally do not parse these messages directly. `robot_navigation.c` converts them into API functions.

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

### Coordinate System

The world uses an ENU-style floor plane:

```text
X     east / right on the floor
Y     north / up on the floor
Z     height
theta robot yaw around +Z
```

`Pose2D` represents the robot center:

```c
typedef struct {
  double x;
  double y;
  double theta;
} Pose2D;
```

Heading constants are defined in `warehouse_map.h`:

```c
#define FACE_EAST   0.0
#define FACE_NORTH  (M_PI / 2.0)
#define FACE_WEST   M_PI
#define FACE_SOUTH  (-M_PI / 2.0)
```

## Student Controller API

Student code should include `warehouse_map.h`, which also exposes `robot_navigation.h`.

```c
#include "warehouse_map.h"
```

### Lifecycle and Status

| Function | Description |
| --- | --- |
| `nav_init()` | Initializes Webots devices and navigation state. |
| `nav_step()` | Advances one simulation step and reads supervisor messages. |
| `nav_stop()` | Stops both wheels. |
| `nav_reset_actions()` | Resets timers and action state after a state transition. |
| `nav_pose_valid()` | Returns `true` after the first valid `POSE` message. |
| `nav_pose()` | Returns the latest robot pose. |
| `nav_last_order()` | Returns the latest order string, for example `"RRGB"`. |
| `nav_score()` | Returns the latest score. |
| `nav_attached_box()` | Returns the attached box name, or `"none"`. |
| `nav_normalize_angle(angle)` | Normalizes an angle to `[-pi, +pi]`. |

### Movement

| Function | Description |
| --- | --- |
| `nav_go_to(x, y)` | Drives to an `(x, y)` point and stops. |
| `nav_go_through(x, y)` | Drives through an intermediate waypoint without stopping. |
| `nav_go_to_pose(x, y, theta)` | Drives to a point, then rotates to the final heading. |
| `nav_rotate_to(theta)` | Rotates in place to a heading. |
| `nav_wait(seconds)` | Waits while stopped. |
| `nav_back_up(seconds)` | Reverses for a fixed time. |
| `nav_back_to(x, y)` | Reverses to a clear point while keeping the current front direction. |
| `nav_move_arc(radius_m, angle_rad, clockwise)` | Drives a circular arc. |
| `nav_move_circle(radius_m, angle_rad, clockwise)` | Alias for `nav_move_arc()`. |

### Magnet

| Function | Description |
| --- | --- |
| `magnet_pick()` | Requests virtual magnet attachment. |
| `magnet_drop()` | Requests virtual magnet release and drop evaluation. |
| `magnet_is_on()` | Returns the local magnet state. |

### Machine Readiness

When processing finishes, the supervisor reports the machine, box index, output bay, and robot-center pickup pose.

| Function | Description |
| --- | --- |
| `nav_machine_a_ready(box_index)` | Returns `true` if the box is ready at Machine A output. |
| `nav_machine_b_ready(box_index)` | Returns `true` if the box is ready at Machine B output. |
| `nav_machine_a_ready_bay(box_index)` | Returns the Machine A output bay, or `-1`. |
| `nav_machine_b_ready_bay(box_index)` | Returns the Machine B output bay, or `-1`. |
| `nav_machine_a_ready_pose(box_index, &pose)` | Copies the Machine A pickup pose when available. |
| `nav_machine_b_ready_pose(box_index, &pose)` | Copies the Machine B pickup pose when available. |

## Map Constants

`warehouse_map.h` provides named robot-center poses for all important locations:

| Constant | Description |
| --- | --- |
| `MAP_IN_PICK[0..3]` | Incoming warehouse pickup poses. |
| `MAP_IN_FRONT[0..3]` | Incoming approach and clear poses. |
| `MAP_OUT_FRONT[0..3]` | Outgoing approach and clear poses. |
| `MAP_OUT_DROP[0..3]` | Outgoing drop poses. |
| `MAP_MACHINE_A_INPUT_BAY[0..1]` | Machine A input service poses. |
| `MAP_MACHINE_A_INPUT_CLEAR_BAY[0..1]` | Machine A input clear poses. |
| `MAP_MACHINE_A_OUTPUT_APPROACH_BAY[0..1]` | Machine A output approach poses. |
| `MAP_MACHINE_A_OUTPUT_BAY[0..1]` | Machine A output pickup poses. |
| `MAP_MACHINE_A_OUTPUT_CLEAR_BAY[0..1]` | Machine A output clear poses. |
| `MAP_MACHINE_B_INPUT_BAY[0..1]` | Machine B input service poses. |
| `MAP_MACHINE_B_INPUT_CLEAR_BAY[0..1]` | Machine B input clear poses. |
| `MAP_MACHINE_B_OUTPUT_APPROACH_BAY[0..1]` | Machine B output approach poses. |
| `MAP_MACHINE_B_OUTPUT_BAY[0..1]` | Machine B output pickup poses. |
| `MAP_MACHINE_B_OUTPUT_CLEAR_BAY[0..1]` | Machine B output clear poses. |

Use `nav_go_through()` for intermediate graph nodes and `nav_go_to_pose()` for final pickup, drop, input, and output poses.

## Configuration

### Order Mode

The order mode is configured near the top of `logistics_supervisor_c.c`:

```c
#define ORDER_MODE_RANDOM 0
#define ORDER_MODE_MANUAL 1
#define TASK_ORDER_MODE ORDER_MODE_RANDOM
#define MANUAL_ORDER "RRGB"
```

Use random mode for normal exercises. Use manual mode when testing a specific order.

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

| Symptom | Likely Cause | Fix |
| --- | --- | --- |
| Robot stops at every route point | Intermediate nodes use stop-and-align movement. | Use `nav_go_through()` for route waypoints and reserve `nav_go_to_pose()` for final service poses. |
| Robot turns inside a garage or bay | The route is missing a clear/reverse step. | Use `nav_back_to()` before turning away from narrow pockets. |
| Box is dropped but not accepted | Wrong box type or invalid drop location. | Check the current box state and target pose. |
| Machine-ready mask stays at `0` | No processed box is waiting at that output. | Confirm that the box was accepted by the correct machine and wait for the ready event. |
| Overlay bay value shows `0` | Bay contains `BOX_0`. | Remember that `-1` means empty and `0..3` are box indexes. |
| Robot cannot attach a box in a machine input | Input boxes are intentionally not attachable. | Wait for processing to finish and pick the box from the output bay. |
| Robot clips a machine output box | The approach is too diagonal or too close. | Enter outputs through the approach pose, then drive straight into the reported pickup pose. |

## Educational Use

This repository is designed for project-based learning in robotics and logistics automation. Good extensions include:

- generalizing the starter controller from one box to all four boxes;
- replacing hard-coded demo constants with `current_box_index`;
- choosing machine bays dynamically;
- improving route planning with intermediate graph nodes;
- smoothing selected turns with `nav_move_arc()`;
- comparing strategies by final score and execution time.

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE).

## Author

Created by [@jbneto1](https://github.com/jbneto1) for a Robotics / Polytechnic Institute of Braganca Webots logistics warehouse project-based learning activity.
