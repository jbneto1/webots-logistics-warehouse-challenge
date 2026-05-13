#include "warehouse_map.h"

#include <stdio.h>
#include <string.h>

/*
 * Beginner controller.
 *
 * Students should mainly edit this file.  The navigation library hides Webots
 * motors, messages, and wheel math.  The map header gives named graph poses.
 *
 * This example solves one selected box dynamically.
 *
 * Machine outputs are event/status based: the robot goes to the output approach
 * node soon after dropping a box, then waits there only until the supervisor
 * says that specific box is ready.  The READY message also contains the output
 * bay and robot-center pickup pose.
 */

#define ARRAY_COUNT(a) ((int)(sizeof(a) / sizeof((a)[0])))

/* Beginner demo settings.  Students later replace DEMO_BOX with a loop from 0 to 3. */
#define DEMO_BOX 0
#define DEMO_A_BAY 0
#define DEMO_B_BAY 0

typedef enum {
  WAIT_FOR_POSE,
  WAIT_FOR_ORDER,

  ROUTE_TO_BOX_0,
  PICK_BOX_0,
  CLEAR_BOX_0,
  DECIDE_BOX_0_DESTINATION,

  ROUTE_BOX_0_TO_A_INPUT,
  DROP_AT_MACHINE_A,
  CLEAR_MACHINE_A_INPUT,
  ROUTE_TO_A_OUTPUT_APPROACH,
  WAIT_FOR_MACHINE_A_READY,
  ENTER_A_OUTPUT,
  PICK_FROM_MACHINE_A,
  CLEAR_MACHINE_A_OUTPUT,

  ROUTE_TO_B_INPUT,
  DROP_AT_MACHINE_B,
  CLEAR_MACHINE_B_INPUT,
  ROUTE_TO_B_OUTPUT_APPROACH,
  WAIT_FOR_MACHINE_B_READY,
  ENTER_B_OUTPUT,
  PICK_FROM_MACHINE_B,
  CLEAR_MACHINE_B_OUTPUT,

  ROUTE_TO_OUTGOING,
  DROP_AT_OUTGOING,
  CLEAR_OUTGOING,
  FINISHED
} State;

static State state = WAIT_FOR_POSE;
static char demo_box_type = '?';
static int final_message_printed = 0;
static int route_index = 0;
static int active_a_bay = DEMO_A_BAY;
static int active_b_bay = DEMO_B_BAY;
static Pose2D a_ready_pick_pose;
static Pose2D b_ready_pick_pose;

static const char *STATE_NAME[] = {
  "WAIT_FOR_POSE",
  "WAIT_FOR_ORDER",
  "ROUTE_TO_BOX_0",
  "PICK_BOX_0",
  "CLEAR_BOX_0",
  "DECIDE_BOX_0_DESTINATION",
  "ROUTE_BOX_0_TO_A_INPUT",
  "DROP_AT_MACHINE_A",
  "CLEAR_MACHINE_A_INPUT",
  "ROUTE_TO_A_OUTPUT_APPROACH",
  "WAIT_FOR_MACHINE_A_READY",
  "ENTER_A_OUTPUT",
  "PICK_FROM_MACHINE_A",
  "CLEAR_MACHINE_A_OUTPUT",
  "ROUTE_TO_B_INPUT",
  "DROP_AT_MACHINE_B",
  "CLEAR_MACHINE_B_INPUT",
  "ROUTE_TO_B_OUTPUT_APPROACH",
  "WAIT_FOR_MACHINE_B_READY",
  "ENTER_B_OUTPUT",
  "PICK_FROM_MACHINE_B",
  "CLEAR_MACHINE_B_OUTPUT",
  "ROUTE_TO_OUTGOING",
  "DROP_AT_OUTGOING",
  "CLEAR_OUTGOING",
  "FINISHED"
};

/* Change state and reset action timers. */
static void change_state(State next_state) {
  state = next_state;
  route_index = 0;
  nav_reset_actions();
  printf("State: %s\n", STATE_NAME[state]);
}

/* Follow a route whose last two poses are an aligned approach point and the
 * final bay/machine point.  The robot drives through normal graph nodes,
 * stops and faces the correct direction at the approach point, then enters the
 * bay straight.
 */
static int follow_route_with_final_alignment(const Pose2D route[], int count) {
  if (count <= 0)
    return 1;

  if (count == 1) {
    Pose2D goal = route[0];
    return nav_go_to_pose(goal.x, goal.y, goal.theta);
  }

  if (route_index < count - 2) {
    Pose2D waypoint = route[route_index];
    if (nav_go_through(waypoint.x, waypoint.y))
      ++route_index;
    return 0;
  }

  if (route_index == count - 2) {
    Pose2D approach = route[route_index];
    if (nav_go_to_pose(approach.x, approach.y, approach.theta))
      ++route_index;
    return 0;
  }

  Pose2D goal = route[count - 1];
  return nav_go_to_pose(goal.x, goal.y, goal.theta);
}

/* Drive to a single service pose and stop facing its final heading. */
static int go_to_pose(Pose2D pose) {
  return nav_go_to_pose(pose.x, pose.y, pose.theta);
}

/* Print the first valid pose once for debugging. */
static void print_initial_pose_once(void) {
  static int printed = 0;

  if (!printed && nav_pose_valid()) {
    Pose2D pose = nav_pose();
    printf("Initial pose: x=%.3f y=%.3f theta=%.3f\n", pose.x, pose.y, pose.theta);
    printed = 1;
  }
}

static int valid_bay(int bay) {
  return bay >= 0 && bay < MAP_MACHINE_BAY_COUNT;
}

int main(void) {
  nav_init();

  while (nav_step()) {
    print_initial_pose_once();

    switch (state) {
      case WAIT_FOR_POSE:
        if (nav_pose_valid())
          change_state(WAIT_FOR_ORDER);
        break;

      case WAIT_FOR_ORDER:
        if (strlen(nav_last_order()) >= MAP_BOX_COUNT) {
          demo_box_type = nav_last_order()[DEMO_BOX];
          printf("ORDER is %s. BOX_%d type is %c. A bay=%d, B bay=%d.\n",
                 nav_last_order(), DEMO_BOX, demo_box_type, DEMO_A_BAY, DEMO_B_BAY);
          change_state(ROUTE_TO_BOX_0);
        }
        break;

      case ROUTE_TO_BOX_0: {
        const Pose2D path[] = {
          MAP_P21_WEST_SOUTH,
          MAP_P10_WEST_CENTER,
          MAP_IN_FRONT[DEMO_BOX],
          MAP_IN_PICK[DEMO_BOX]
        };
        if (follow_route_with_final_alignment(path, ARRAY_COUNT(path)))
          change_state(PICK_BOX_0);
        break;
      }

      case PICK_BOX_0:
        magnet_pick();
        if (nav_wait(0.45))
          change_state(CLEAR_BOX_0);
        break;

      case CLEAR_BOX_0:
        if (nav_back_to(MAP_IN_FRONT[DEMO_BOX].x, MAP_IN_FRONT[DEMO_BOX].y))
          change_state(DECIDE_BOX_0_DESTINATION);
        break;

      case DECIDE_BOX_0_DESTINATION:
        if (demo_box_type == 'R')
          change_state(ROUTE_BOX_0_TO_A_INPUT);
        else if (demo_box_type == 'G')
          change_state(ROUTE_TO_B_INPUT);
        else
          change_state(ROUTE_TO_OUTGOING);
        break;

      case ROUTE_BOX_0_TO_A_INPUT: {
        const Pose2D path[] = {
          MAP_MACHINE_A_INPUT_CLEAR_BAY[DEMO_A_BAY],
          MAP_MACHINE_A_INPUT_BAY[DEMO_A_BAY]
        };
        if (follow_route_with_final_alignment(path, ARRAY_COUNT(path)))
          change_state(DROP_AT_MACHINE_A);
        break;
      }

      case DROP_AT_MACHINE_A:
        magnet_drop();
        if (nav_wait(0.25))
          change_state(CLEAR_MACHINE_A_INPUT);
        break;

      case CLEAR_MACHINE_A_INPUT:
        if (nav_back_to(MAP_MACHINE_A_INPUT_CLEAR_BAY[DEMO_A_BAY].x,
                        MAP_MACHINE_A_INPUT_CLEAR_BAY[DEMO_A_BAY].y))
          change_state(ROUTE_TO_A_OUTPUT_APPROACH);
        break;

      case ROUTE_TO_A_OUTPUT_APPROACH: {
        const Pose2D path[] = {
          MAP_P10_WEST_CENTER,
          MAP_IN_FRONT[DEMO_BOX],
          MAP_P4_TOP_CENTER,
          MAP_MACHINE_A_OUTPUT_APPROACH_BAY[DEMO_A_BAY]
        };
        if (follow_route_with_final_alignment(path, ARRAY_COUNT(path)))
          change_state(WAIT_FOR_MACHINE_A_READY);
        break;
      }

      case WAIT_FOR_MACHINE_A_READY: {
        nav_stop();
        if (nav_machine_a_ready_pose(DEMO_BOX, &a_ready_pick_pose)) {
          int bay = nav_machine_a_ready_bay(DEMO_BOX);
          if (valid_bay(bay))
            active_a_bay = bay;
          change_state(ENTER_A_OUTPUT);
        }
        break;
      }

      case ENTER_A_OUTPUT:
        if (go_to_pose(a_ready_pick_pose))
          change_state(PICK_FROM_MACHINE_A);
        break;

      case PICK_FROM_MACHINE_A:
        magnet_pick();
        if (nav_wait(0.45))
          change_state(CLEAR_MACHINE_A_OUTPUT);
        break;

      case CLEAR_MACHINE_A_OUTPUT:
        if (nav_back_to(MAP_MACHINE_A_OUTPUT_CLEAR_BAY[active_a_bay].x,
                        MAP_MACHINE_A_OUTPUT_CLEAR_BAY[active_a_bay].y))
          change_state(ROUTE_TO_B_INPUT);
        break;

      case ROUTE_TO_B_INPUT: {
        if (demo_box_type == 'R') {
          const Pose2D path[] = {
            MAP_P13_CENTER,
            MAP_MACHINE_B_INPUT_CLEAR_BAY[DEMO_B_BAY],
            MAP_MACHINE_B_INPUT_BAY[DEMO_B_BAY]
          };
          if (follow_route_with_final_alignment(path, ARRAY_COUNT(path)))
            change_state(DROP_AT_MACHINE_B);
        } else {
          const Pose2D path[] = {
            MAP_IN_FRONT[DEMO_BOX],
            MAP_P4_TOP_CENTER,
            MAP_P13_CENTER,
            MAP_MACHINE_B_INPUT_CLEAR_BAY[DEMO_B_BAY],
            MAP_MACHINE_B_INPUT_BAY[DEMO_B_BAY]
          };
          if (follow_route_with_final_alignment(path, ARRAY_COUNT(path)))
            change_state(DROP_AT_MACHINE_B);
        }
        break;
      }

      case DROP_AT_MACHINE_B:
        magnet_drop();
        if (nav_wait(0.25))
          change_state(CLEAR_MACHINE_B_INPUT);
        break;

      case CLEAR_MACHINE_B_INPUT:
        if (nav_back_to(MAP_MACHINE_B_INPUT_CLEAR_BAY[DEMO_B_BAY].x,
                        MAP_MACHINE_B_INPUT_CLEAR_BAY[DEMO_B_BAY].y))
          change_state(ROUTE_TO_B_OUTPUT_APPROACH);
        break;

      case ROUTE_TO_B_OUTPUT_APPROACH: {
        const Pose2D path[] = {
          MAP_P13_CENTER,
          MAP_P4V_NORTH_CENTER,
          MAP_P5_NORTH_EAST,
          MAP_MACHINE_B_OUTPUT_APPROACH_BAY[DEMO_B_BAY]
        };
        if (follow_route_with_final_alignment(path, ARRAY_COUNT(path)))
          change_state(WAIT_FOR_MACHINE_B_READY);
        break;
      }

      case WAIT_FOR_MACHINE_B_READY: {
        nav_stop();
        if (nav_machine_b_ready_pose(DEMO_BOX, &b_ready_pick_pose)) {
          int bay = nav_machine_b_ready_bay(DEMO_BOX);
          if (valid_bay(bay))
            active_b_bay = bay;
          change_state(ENTER_B_OUTPUT);
        }
        break;
      }

      case ENTER_B_OUTPUT:
        if (go_to_pose(b_ready_pick_pose))
          change_state(PICK_FROM_MACHINE_B);
        break;

      case PICK_FROM_MACHINE_B:
        magnet_pick();
        if (nav_wait(0.45))
          change_state(CLEAR_MACHINE_B_OUTPUT);
        break;

      case CLEAR_MACHINE_B_OUTPUT:
        if (nav_back_to(MAP_MACHINE_B_OUTPUT_CLEAR_BAY[active_b_bay].x,
                        MAP_MACHINE_B_OUTPUT_CLEAR_BAY[active_b_bay].y))
          change_state(ROUTE_TO_OUTGOING);
        break;

      case ROUTE_TO_OUTGOING:
        if (demo_box_type == 'B') {
          const Pose2D path[] = {
            MAP_IN_FRONT[DEMO_BOX],
            MAP_P10_WEST_CENTER,
            MAP_P21_WEST_SOUTH,
            MAP_P22_CENTER_SOUTH,
            MAP_OUT_FRONT[DEMO_BOX],
            MAP_OUT_DROP[DEMO_BOX]
          };
          if (follow_route_with_final_alignment(path, ARRAY_COUNT(path)))
            change_state(DROP_AT_OUTGOING);
        } else {
          const Pose2D path[] = {
            MAP_MACHINE_B_OUTPUT_CLEAR_BAY[active_b_bay],
            MAP_OUT_FRONT[3],
            MAP_OUT_FRONT[DEMO_BOX],
            MAP_OUT_DROP[DEMO_BOX]
          };
          if (follow_route_with_final_alignment(path, ARRAY_COUNT(path)))
            change_state(DROP_AT_OUTGOING);
        }
        break;

      case DROP_AT_OUTGOING:
        magnet_drop();
        if (nav_wait(0.40))
          change_state(CLEAR_OUTGOING);
        break;

      case CLEAR_OUTGOING:
        if (nav_back_to(MAP_OUT_FRONT[DEMO_BOX].x, MAP_OUT_FRONT[DEMO_BOX].y))
          change_state(FINISHED);
        break;

      case FINISHED:
        nav_stop();
        if (!final_message_printed) {
          printf("Example finished. Score reported by supervisor: %d\n", nav_score());
          printf("Next exercise: repeat this same state pattern for the remaining boxes.\n");
          printf("Optional exercise: replace selected right-angle transitions with nav_move_arc().\n");
          final_message_printed = 1;
        }
        break;
    }
  }

  nav_stop();
  return 0;
}
