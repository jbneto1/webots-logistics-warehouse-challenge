## Author
**João Afonso Braun Neto © 2026**
- GitHub: [@jbneto1](https://github.com/jbneto1)
- Project created for: Robotics class / Polytechnic Institute of Bragança

# Webots Logistics PBL Warehouse Challenge

Beginner-friendly Webots R2025a project inspired by [RobotAtFactory Lite](https://github.com/P33a/RobotAtFactoryLite). The environment lets students practise mobile-robot navigation, sequencing, warehouse logistics and state machines without writing low-level Webots motor code.

The student-facing controller uses a high-level C API. The supervisor handles the warehouse order, virtual magnet, machine processing, ready messages, scoring, box colours and box placement.

---

## 1. Project structure

```text
webots_logistics_pbl_v10_4/
├── worlds/
│   └── logistics_pbl_enu.wbt
└── controllers/
    ├── logistics_supervisor_c/
    │   ├── logistics_supervisor_c.c
    │   └── Makefile
    └── student_controller/
        ├── student_controller.c
        ├── warehouse_map.h
        ├── robot_navigation.c
        ├── robot_navigation.h
        └── Makefile
```

### Main files

| File | Purpose | Who normally edits it? |
|---|---|---|
| `worlds/logistics_pbl_enu.wbt` | Webots world: robot, warehouses, machines, boxes, floor guides. | Teacher / project maintainer |
| `controllers/logistics_supervisor_c/logistics_supervisor_c.c` | C supervisor: order generation, machine logic, scoring, magnet messages. | Teacher / project maintainer |
| `controllers/student_controller/student_controller.c` | Student state machine. The included example solves `BOX_0` dynamically. | Students |
| `controllers/student_controller/warehouse_map.h` | Named robot poses and map constants. | Students may read; teacher may tune |
| `controllers/student_controller/robot_navigation.h` | High-level API available to students. | Usually read-only |
| `controllers/student_controller/robot_navigation.c` | Navigation implementation and speed tuning. | Advanced students / teacher |

---

## 2. How to run

1. Open **Webots R2025a**.
2. Open:

```text
worlds/logistics_pbl_enu.wbt
```

3. Build the two C controllers if Webots does not build them automatically:

```bash
cd controllers/logistics_supervisor_c
make
cd ../student_controller
make
```

4. Press **Play** in Webots.
5. Watch the simulation overlay and the controller console output.

---

## 3. Educational objective

Students should learn to:

- read a warehouse order;
- represent a logistics task as a finite-state machine;
- use named map poses instead of raw motor commands;
- pick, transport, process and deliver boxes;
- react to asynchronous machine-ready messages;
- reason about robot orientation before entering and leaving narrow garages;
- improve a route without needing to implement a full low-level controller.

The starter controller solves only `BOX_0`. A typical practical assignment is to extend it to `BOX_1`, `BOX_2`, and `BOX_3`.

---

## 4. Coordinate system

The world uses an ENU-style floor plane:

```text
X = east / right on the floor
Y = north / up on the floor
Z = height
theta = robot yaw around +Z
```

Heading constants in `warehouse_map.h`:

```c
#define FACE_EAST   0.0
#define FACE_NORTH  (M_PI / 2.0)
#define FACE_WEST   M_PI
#define FACE_SOUTH  (-M_PI / 2.0)
```

`Pose2D` always describes the **robot centre**, not the box centre:

```c
typedef struct {
  double x;
  double y;
  double theta;
} Pose2D;
```

---

## 5. Logistics rules implemented by the supervisor

There are four boxes in the incoming warehouse. Each box has a colour/type:

| Colour | Meaning | Required destination |
|---|---|---|
| Blue (`B`) | Final part | Outgoing warehouse |
| Green (`G`) | Intermediate part | Machine B, then outgoing warehouse |
| Red (`R`) | Raw part | Machine A, then Machine B, then outgoing warehouse |

Processing sequence:

```text
Red box   -> Machine A -> becomes Green -> Machine B -> becomes Blue -> Outgoing
Green box -> Machine B -> becomes Blue  -> Outgoing
Blue box  -> Outgoing
```

Scoring is handled by the supervisor. A point is awarded when a box is accepted by a valid machine input or delivered to the outgoing warehouse.

---

## 6. Version 10.4 feature summary

This version includes:

- deeper incoming pickup and outgoing drop poses;
- explicit clear poses so the robot can reverse out before turning;
- box collision and physics in the Webots world;
- two active garages/bays for Machine A and two for Machine B;
- random machine processing time from `0` to `5` seconds per box;
- event-based machine-ready messages with box index, bay index and pickup pose;
- ready masks that reset when the processed box is removed from the output;
- `nav_go_through()` for intermediate waypoints so the robot does not stop and rotate at every graph node;
- `nav_back_to()` for faster reverse clearance from garages;
- optional arc movement through `nav_move_arc()` / `nav_move_circle()`.

---

## 7. Student high-level API

Include only `warehouse_map.h` in the student controller:

```c
#include "warehouse_map.h"
```

The map header includes `robot_navigation.h`, so the API is available automatically.

### 7.1 Controller lifecycle and task status

| Function | Meaning |
|---|---|
| `nav_init()` | Initialise Webots devices and the navigation helper. |
| `nav_step()` | Advance one Webots time step and read supervisor messages. Use it in the main loop. |
| `nav_stop()` | Stop both wheels immediately. |
| `nav_reset_actions()` | Reset timers/arcs when changing state. |
| `nav_pose_valid()` | True after the first valid `POSE` message. |
| `nav_pose()` | Latest robot pose as `Pose2D`. |
| `nav_last_order()` | Latest order string, e.g. `"RRGB"`. |
| `nav_score()` | Latest score reported by the supervisor. |
| `nav_attached_box()` | Name of the attached box, or `"none"`. |
| `nav_normalize_angle(angle)` | Normalise an angle to `[-pi, +pi]`. |

### 7.2 Movement actions

All movement actions are **non-blocking**: call them repeatedly from the current state until they return `true`.

| Function | Use |
|---|---|
| `nav_go_to(x, y)` | Drive accurately to an `(x, y)` point and stop. |
| `nav_go_through(x, y)` | Drive through an intermediate waypoint without stopping. Useful for smoother paths. |
| `nav_go_to_pose(x, y, theta)` | Drive to `(x, y)`, stop, then rotate to `theta`. Use for pickup/drop/service poses. |
| `nav_rotate_to(theta)` | Rotate in place to a target heading. |
| `nav_wait(seconds)` | Wait while stopped. |
| `nav_back_up(seconds)` | Reverse for a fixed time. |
| `nav_back_to(x, y)` | Reverse to a clear point while keeping the robot front facing the garage/output. |
| `nav_move_arc(radius_m, angle_rad, clockwise)` | Drive a circular arc. Optional for advanced route smoothing. |
| `nav_move_circle(radius_m, angle_rad, clockwise)` | Alias of `nav_move_arc()`. |

Example state:

```c
case GO_TO_BOX:
  if (nav_go_to_pose(MAP_IN_PICK[0].x, MAP_IN_PICK[0].y, MAP_IN_PICK[0].theta))
    change_state(PICK_BOX);
  break;
```

### 7.3 Magnet API

| Function | Meaning |
|---|---|
| `magnet_pick()` | Request virtual magnet ON. The supervisor attaches the nearest valid box. |
| `magnet_drop()` | Request virtual magnet OFF. The supervisor evaluates the drop location. |
| `magnet_is_on()` | Local magnet state. |

The magnet does not directly move physics bodies. The supervisor keeps the attached box in front of the robot while the magnet is on.

### 7.4 Machine-ready API

When a machine finishes processing, the supervisor sends the machine, box index, output bay and robot pickup pose. The navigation API stores this information.

| Function | Meaning |
|---|---|
| `nav_machine_a_ready(box_index)` | True if that box is ready at a Machine A output. |
| `nav_machine_b_ready(box_index)` | True if that box is ready at a Machine B output. |
| `nav_machine_a_ready_garage(box_index)` | Output bay for that box at Machine A, or `-1` if not ready. |
| `nav_machine_b_ready_garage(box_index)` | Output bay for that box at Machine B, or `-1` if not ready. |
| `nav_machine_a_ready_pose(box_index, &pose)` | Copies the robot-centre pickup pose for Machine A if ready. |
| `nav_machine_b_ready_pose(box_index, &pose)` | Copies the robot-centre pickup pose for Machine B if ready. |

Example:

```c
Pose2D pick_pose;

if (nav_machine_a_ready_pose(2, &pick_pose)) {
  int bay = nav_machine_a_ready_garage(2);
  printf("BOX_2 ready at Machine A bay %d\n", bay);
  // Drive to pick_pose to collect the processed box.
}
```

---

## 8. Map constants in `warehouse_map.h`

### 8.1 General constants

```c
#define MAP_BOX_COUNT 4
#define MAP_MACHINE_BAY_COUNT 2
```

### 8.2 Incoming warehouse

```c
MAP_IN_PICK[0..3]   // deeper pickup poses inside the incoming garages
MAP_IN_FRONT[0..3]  // approach/clear poses outside the incoming garages
```

### 8.3 Outgoing warehouse

```c
MAP_OUT_FRONT[0..3] // approach/clear poses outside the outgoing garages
MAP_OUT_DROP[0..3]  // deeper drop poses inside the outgoing garages
```

### 8.4 Machine A poses

Machine A has two active bays:

```c
MAP_MACHINE_A_INPUT_BAY[0..1]
MAP_MACHINE_A_INPUT_CLEAR_BAY[0..1]
MAP_MACHINE_A_OUTPUT_APPROACH_BAY[0..1]
MAP_MACHINE_A_OUTPUT_BAY[0..1]
MAP_MACHINE_A_OUTPUT_CLEAR_BAY[0..1]
```

Bay `0` is the original/main bay. Bay `1` is the second parallel bay.

Backward-compatible bay-0 aliases also exist:

```c
MAP_MACHINE_A_INPUT
MAP_MACHINE_A_INPUT_CLEAR
MAP_MACHINE_A_OUTPUT_APPROACH
MAP_MACHINE_A_OUTPUT
MAP_MACHINE_A_OUTPUT_CLEAR
```

### 8.5 Machine B poses

```c
MAP_MACHINE_B_INPUT_BAY[0..1]
MAP_MACHINE_B_INPUT_CLEAR_BAY[0..1]
MAP_MACHINE_B_OUTPUT_APPROACH_BAY[0..1]
MAP_MACHINE_B_OUTPUT_BAY[0..1]
MAP_MACHINE_B_OUTPUT_CLEAR_BAY[0..1]
```

Backward-compatible bay-0 aliases:

```c
MAP_MACHINE_B_INPUT
MAP_MACHINE_B_INPUT_CLEAR
MAP_MACHINE_B_OUTPUT_APPROACH
MAP_MACHINE_B_OUTPUT
MAP_MACHINE_B_OUTPUT_CLEAR
```

### 8.6 Main graph nodes

These are intermediate navigation nodes used to form routes:

```c
MAP_P4_TOP_CENTER
MAP_P4V_NORTH_CENTER
MAP_P5_NORTH_EAST
MAP_P10_WEST_CENTER
MAP_P13_CENTER
MAP_P16_EAST_CENTER
MAP_P21_WEST_SOUTH
MAP_P22V_SOUTH_CENTER
MAP_P22_CENTER_SOUTH
```

Use `nav_go_through()` for intermediate graph nodes and `nav_go_to_pose()` for final service poses.

---

## 9. Navigation tuning constants

The main tuning values are in `controllers/student_controller/robot_navigation.c`:

```c
#define MAX_WHEEL_SPEED_RAD_S 18.5
#define POSITION_TOLERANCE_M 0.025
#define BACK_POSITION_TOLERANCE_M 0.055
#define THROUGH_TOLERANCE_M 0.100
#define ANGLE_TOLERANCE_RAD 0.055
#define FINAL_SLOWDOWN_RADIUS_M 0.045
#define FINAL_MIN_LINEAR_M_S 0.035
#define FINAL_MAX_LINEAR_M_S 0.170
#define THROUGH_MIN_LINEAR_M_S 0.085
#define THROUGH_MAX_LINEAR_M_S 0.225
#define ARC_LINEAR_SPEED_M_S 0.110
```

Most useful values to tune:

| Constant | Effect |
|---|---|
| `MAX_WHEEL_SPEED_RAD_S` | Global wheel speed ceiling. |
| `THROUGH_TOLERANCE_M` | Radius used to switch to the next intermediate waypoint. Larger values make routes smoother but less exact. |
| `FINAL_SLOWDOWN_RADIUS_M` | Distance from final targets where the robot begins to slow down. Smaller values keep speed higher for longer. |
| `THROUGH_MAX_LINEAR_M_S` | Maximum speed through intermediate waypoints. |
| `FINAL_MAX_LINEAR_M_S` | Maximum speed for final approach movements. |
| `BACK_POSITION_TOLERANCE_M` | Reverse clearance tolerance used by `nav_back_to()`. |
| `ARC_LINEAR_SPEED_M_S` | Speed used by `nav_move_arc()`. |

---

## 10. Supervisor behaviour

The supervisor is a C-only Webots supervisor. It replaces a Python supervisor to avoid Python-related Webots crashes on some installations.

### 10.1 Order modes

At the top of `logistics_supervisor_c.c`:

```c
#define ORDER_MODE_RANDOM 0
#define ORDER_MODE_MANUAL 1
#define TASK_ORDER_MODE ORDER_MODE_RANDOM
#define MANUAL_ORDER "RRGB"
```

Use random mode for normal exercises. Use manual mode when testing a specific case.

### 10.2 Supervisor messages

The supervisor sends these messages to the student controller:

```text
START
ORDER RRGB
POSE x y theta score attached_box machine_a_ready_mask machine_b_ready_mask
READY A box_index bay_index robot_x robot_y robot_theta
READY B box_index bay_index robot_x robot_y robot_theta
CLEAR A box_index
CLEAR B box_index
```

Students normally do not parse these messages manually. `robot_navigation.c` converts them into API functions.

### 10.3 Ready masks

The overlay line:

```text
ready A=0 B=0
```

means no processed boxes are currently waiting at Machine A or Machine B outputs.

Ready masks are bitmasks:

| Mask value | Meaning |
|---|---|
| `0` | no box ready |
| `1` | `BOX_0` ready |
| `2` | `BOX_1` ready |
| `4` | `BOX_2` ready |
| `8` | `BOX_3` ready |
| `3` | `BOX_0` and `BOX_1` ready |
| `5` | `BOX_0` and `BOX_2` ready |

The mask tells **which box** is ready. The `READY` message tells **which bay** and **which pickup pose**.

### 10.4 Bay status in the overlay

The overlay also shows bay input/output status, for example:

```text
A bays in/out: -1/-1 0/-1
```

Meaning:

```text
Machine A bay 0: input empty, output empty
Machine A bay 1: input has BOX_0, output empty
```

Bay values:

| Value | Meaning |
|---|---|
| `-1` | empty |
| `0` | contains `BOX_0` |
| `1` | contains `BOX_1` |
| `2` | contains `BOX_2` |
| `3` | contains `BOX_3` |

Important: `0` does not mean false/empty. It means box index `0`.

### 10.5 Machine processing logic

- A red box dropped in a valid Machine A input bay is accepted and scores `+1`.
- A green box dropped in a valid Machine B input bay is accepted and scores `+1`.
- Processing time is random from `0` to `5` seconds.
- If the input bay is empty, a box may be placed there.
- If the output of the same bay is occupied, the box waits in the input.
- Processing starts only when the input has a box, the bay is not already processing, and the output is empty.
- When processing finishes, the supervisor moves the box to the output, changes its colour, and sends a `READY` message.
- When the robot attaches a ready output box, the supervisor sends `CLEAR` and frees that output bay.

### 10.6 Magnet logic

- `MAGNET_ON` attaches the nearest valid box near the robot magnet point.
- Boxes already processing, delivered, or waiting inside a machine input are not attachable.
- While attached, the supervisor keeps the box in front of the robot.
- `MAGNET_OFF` drops the box and evaluates whether it is in a valid destination zone.

### 10.7 Box collision and physics

Boxes have Webots collision and physics. The supervisor resets box physics after supervised pose changes to avoid stale velocities and jitter.

---

## 11. Student controller design

The included `student_controller.c` is intentionally a beginner example:

- it waits for a valid pose;
- it waits for the order string;
- it solves only `BOX_0`;
- it dynamically chooses the route based on whether `BOX_0` is `R`, `G`, or `B`;
- it drops parts at machine inputs and drives to the output approach soon after;
- it waits at the output approach only until the supervisor reports that the box is ready;
- it then enters the correct output bay using the ready pickup pose.

The key state-machine pattern is:

```c
case SOME_STATE:
  if (some_non_blocking_action())
    change_state(NEXT_STATE);
  break;
```

The helper `follow_route_with_final_alignment()` drives through intermediate graph nodes, stops at the aligned approach pose, and then enters the final garage straight.

---

## 12. Suggested student extensions

1. Repeat the `BOX_0` logic for `BOX_1`, `BOX_2`, and `BOX_3`.
2. Replace hard-coded `BOX_0` constants with `current_box_index`.
3. Use `MAP_IN_PICK[current_box_index]` and `MAP_OUT_DROP[current_box_index]`.
4. Use `nav_machine_a_ready_pose(current_box_index, &pose)` and `nav_machine_b_ready_pose(current_box_index, &pose)`.
5. Select machine bays intelligently instead of always using bay 0.
6. Use `nav_go_through()` for all intermediate route nodes.
7. Optionally use `nav_move_arc()` for smoother transitions between perpendicular corridors.
8. Tune speed constants while checking that collisions do not increase.

---

## 13. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Robot stops at every waypoint | Using `nav_go_to_pose()` for intermediate nodes. | Use `nav_go_through()` for intermediate route points. |
| Robot hits a box at machine output | Entering the output diagonally or too close. | Use the machine output approach pose, then enter straight. |
| Robot turns inside a garage | Missing clear/reverse state. | Use `nav_back_to(clear_pose.x, clear_pose.y)` before rotating. |
| `ready A=0 B=0` | No processed boxes are ready. | Wait at the output approach or check whether processing started. |
| Bay display shows `0` | The bay contains `BOX_0`. | Remember: `-1` is empty; `0..3` are box indices. |
| Box is dropped but not accepted | Wrong colour or outside valid input/output zone. | Check the box type and target pose. |
| Robot cannot attach a box in a machine input | Boxes waiting in machine inputs are intentionally not attachable. | Wait for the processed box at the output. |

---

## 14. Minimal full-task pseudocode

```text
wait until pose is valid
wait until order is received

for each box index from 0 to 3:
  go to incoming pickup pose for that box
  pick the box
  reverse to the incoming clear pose

  if box is red:
    go to a Machine A input bay
    drop the box
    reverse to the input clear pose
    go to the Machine A output approach
    wait until that same box is ready
    enter the reported output bay and pick it
    reverse to the output clear pose

  if box is green or was processed by Machine A:
    go to a Machine B input bay
    drop the box
    reverse to the input clear pose
    go to the Machine B output approach
    wait until that same box is ready
    enter the reported output bay and pick it
    reverse to the output clear pose

  go to outgoing warehouse drop pose
  drop the blue box
  reverse to the outgoing clear pose

stop
```

---

## 15. Design note

The grey floor paths are visual guides for the intended graph. They are not sensors and they do not provide obstacle avoidance. The controller drives between named poses. If a route cuts across a wall or enters a garage at a bad angle, the robot can collide.

