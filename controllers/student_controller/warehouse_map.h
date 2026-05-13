#ifndef WAREHOUSE_MAP_H
#define WAREHOUSE_MAP_H

/*
 * warehouse_map.h
 *
 * Named robot-center poses for the logistics warehouse.
 *
 * ENU coordinate system:
 *   X = east/right on the floor
 *   Y = north/up on the floor
 *   Z = height
 *   theta = robot yaw around Z
 */

#include "robot_navigation.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FACE_EAST  0.0
#define FACE_NORTH (M_PI / 2.0)
#define FACE_WEST  M_PI
#define FACE_SOUTH (-M_PI / 2.0)

#define MAP_BOX_COUNT 4
#define MAP_MACHINE_BAY_COUNT 2

/* Official-style start pose from the simulator scene: lower-left, facing north. */
static const Pose2D MAP_START = {-0.725, -0.400, FACE_NORTH};

/* Incoming warehouse pickup row.  These poses are intentionally deeper inside
 * the pockets so the magnet reaches the box reliably.
 */
static const Pose2D MAP_IN_PICK[MAP_BOX_COUNT] = {
  {-0.695, 0.445, FACE_NORTH},
  {-0.545, 0.445, FACE_NORTH},
  {-0.400, 0.445, FACE_NORTH},
  {-0.245, 0.445, FACE_NORTH}
};

/* Incoming approach/clear row.  Reverse to this row before turning. */
static const Pose2D MAP_IN_FRONT[MAP_BOX_COUNT] = {
  {-0.695, 0.244, FACE_NORTH},
  {-0.545, 0.244, FACE_NORTH},
  {-0.400, 0.244, FACE_NORTH},
  {-0.245, 0.244, FACE_NORTH}
};

/* Main graph nodes from the navigation map. */
static const Pose2D MAP_P4_TOP_CENTER = {0.000, 0.244, FACE_SOUTH};
static const Pose2D MAP_P4V_NORTH_CENTER = {0.000, 0.425, FACE_EAST};
static const Pose2D MAP_P5_NORTH_EAST = {0.695, 0.425, FACE_SOUTH};
static const Pose2D MAP_P10_WEST_CENTER = {-0.695, 0.000, FACE_NORTH};
static const Pose2D MAP_P13_CENTER = {0.000, 0.000, FACE_EAST};
static const Pose2D MAP_P16_EAST_CENTER = {0.695, -0.010, FACE_SOUTH};
static const Pose2D MAP_P21_WEST_SOUTH = {-0.695, -0.425, FACE_EAST};
static const Pose2D MAP_P22V_SOUTH_CENTER = {0.000, -0.425, FACE_NORTH};
static const Pose2D MAP_P22_CENTER_SOUTH = {0.000, -0.244, FACE_EAST};

/* Machine A: input on the west/left, output on the east/right. */
static const Pose2D MAP_MACHINE_A_INPUT_BAY[MAP_MACHINE_BAY_COUNT] = {
  {-0.520,  0.000, FACE_EAST},
  {-0.520, -0.150, FACE_EAST}
};

static const Pose2D MAP_MACHINE_A_INPUT_CLEAR_BAY[MAP_MACHINE_BAY_COUNT] = {
  {-0.695,  0.000, FACE_EAST},
  {-0.695, -0.150, FACE_EAST}
};

static const Pose2D MAP_MACHINE_A_OUTPUT_APPROACH_BAY[MAP_MACHINE_BAY_COUNT] = {
  {0.000,  0.000, FACE_WEST},
  {0.000, -0.150, FACE_WEST}
};

static const Pose2D MAP_MACHINE_A_OUTPUT_BAY[MAP_MACHINE_BAY_COUNT] = {
  {-0.155,  0.000, FACE_WEST},
  {-0.155, -0.150, FACE_WEST}
};

static const Pose2D MAP_MACHINE_A_OUTPUT_CLEAR_BAY[MAP_MACHINE_BAY_COUNT] = {
  {0.000,  0.000, FACE_WEST},
  {0.000, -0.150, FACE_WEST}
};

/* Machine B: input on the west/left, output on the east/right. */
static const Pose2D MAP_MACHINE_B_INPUT_BAY[MAP_MACHINE_BAY_COUNT] = {
  {0.175,  0.000, FACE_EAST},
  {0.175,  0.150, FACE_EAST}
};

static const Pose2D MAP_MACHINE_B_INPUT_CLEAR_BAY[MAP_MACHINE_BAY_COUNT] = {
  {0.000,  0.000, FACE_EAST},
  {0.000,  0.150, FACE_EAST}
};

static const Pose2D MAP_MACHINE_B_OUTPUT_APPROACH_BAY[MAP_MACHINE_BAY_COUNT] = {
  {0.695, -0.010, FACE_WEST},
  {0.695,  0.150, FACE_WEST}
};

static const Pose2D MAP_MACHINE_B_OUTPUT_BAY[MAP_MACHINE_BAY_COUNT] = {
  {0.535, -0.010, FACE_WEST},
  {0.535,  0.150, FACE_WEST}
};

static const Pose2D MAP_MACHINE_B_OUTPUT_CLEAR_BAY[MAP_MACHINE_BAY_COUNT] = {
  {0.695, -0.010, FACE_WEST},
  {0.695,  0.150, FACE_WEST}
};

/* Outgoing approach row and deeper drop row. */
static const Pose2D MAP_OUT_FRONT[MAP_BOX_COUNT] = {
  {0.245, -0.244, FACE_SOUTH},
  {0.395, -0.244, FACE_SOUTH},
  {0.545, -0.244, FACE_SOUTH},
  {0.695, -0.244, FACE_SOUTH}
};

static const Pose2D MAP_OUT_DROP[MAP_BOX_COUNT] = {
  {0.245, -0.455, FACE_SOUTH},
  {0.395, -0.455, FACE_SOUTH},
  {0.545, -0.455, FACE_SOUTH},
  {0.695, -0.455, FACE_SOUTH}
};

#endif
