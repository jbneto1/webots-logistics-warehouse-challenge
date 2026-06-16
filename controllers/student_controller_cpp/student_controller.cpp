#include "warehouse_map.hpp"
#include "debug_config.hpp"

#include <cstdio>

#define ARRAY_COUNT(a) (static_cast<int>(sizeof(a) / sizeof((a)[0])))

// Demo selector tuning. The reference controller solves one box so students can
// inspect the full flow before generalizing to all boxes and dynamic bay choice.
constexpr int DEMO_BOX = 0;
constexpr int DEMO_A_BAY = 0;
constexpr int DEMO_B_BAY = 0;

enum State {
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
};

static State state = WAIT_FOR_POSE;
static char demoBoxType = '?';
static bool finalMessagePrinted = false;
static int routeIndex = 0;
static int activeABay = DEMO_A_BAY;
static int activeBBay = DEMO_B_BAY;
static Pose2D aReadyPickPose;
static Pose2D bReadyPickPose;
static Navigation *nav = nullptr;

struct DebugMotionTarget {
  const char *action = "idle";
  Pose2D pose = {0.0, 0.0, 0.0};
  bool valid = false;
  int index = 0;
  int count = 0;
};

static DebugMotionTarget debugTarget;

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

static void clearDebugTarget() {
  debugTarget.action = "idle";
  debugTarget.valid = false;
  debugTarget.index = routeIndex;
  debugTarget.count = 0;
}

static void setDebugTarget(const char *action, Pose2D pose, int index, int count) {
  debugTarget.action = action;
  debugTarget.pose = pose;
  debugTarget.valid = true;
  debugTarget.index = index;
  debugTarget.count = count;
}

static void printDetailedDebugIfDue() {
  static double lastPrintTime = -1.0;

  if (!debugEnabled(DEBUG_DETAIL))
    return;

  const double now = nav->time();
  if (lastPrintTime >= 0.0 && now - lastPrintTime < DEBUG_DETAIL_PERIOD_S)
    return;
  lastPrintTime = now;

  Pose2D pose = {0.0, 0.0, 0.0};
  if (nav->poseValid())
    pose = nav->pose();

  if (debugTarget.valid) {
    std::printf("DBG2 t=%.2f state=%s route=%d/%d action=%s target=(%.3f,%.3f,%.3f) "
                "pose=(%.3f,%.3f,%.3f) cmd=(v=%.3f,w=%.3f) wheel=(L=%.2f,R=%.2f) "
                "magnet=%s attached=%s score=%d ready=(A:%d,B:%d)\n",
                now, STATE_NAME[state], debugTarget.index + 1, debugTarget.count, debugTarget.action,
                debugTarget.pose.x, debugTarget.pose.y, debugTarget.pose.theta,
                pose.x, pose.y, pose.theta,
                nav->commandedLinearSpeed(), nav->commandedAngularSpeed(),
                nav->leftWheelSpeedRadS(), nav->rightWheelSpeedRadS(),
                nav->magnetIsOn() ? "ON" : "OFF", nav->attachedBox().c_str(), nav->score(),
                nav->machineAReadyMask(), nav->machineBReadyMask());
  } else {
    std::printf("DBG2 t=%.2f state=%s route=- action=idle pose=(%.3f,%.3f,%.3f) "
                "cmd=(v=%.3f,w=%.3f) wheel=(L=%.2f,R=%.2f) magnet=%s attached=%s "
                "score=%d ready=(A:%d,B:%d)\n",
                now, STATE_NAME[state], pose.x, pose.y, pose.theta,
                nav->commandedLinearSpeed(), nav->commandedAngularSpeed(),
                nav->leftWheelSpeedRadS(), nav->rightWheelSpeedRadS(),
                nav->magnetIsOn() ? "ON" : "OFF", nav->attachedBox().c_str(), nav->score(),
                nav->machineAReadyMask(), nav->machineBReadyMask());
  }
  std::fflush(stdout);
}

static void changeState(State nextState) {
  state = nextState;
  routeIndex = 0;
  clearDebugTarget();
  nav->resetActions();
  if (debugEnabled(DEBUG_STATE)) {
    std::printf("State: %s\n", STATE_NAME[state]);
    std::fflush(stdout);
  }
}

static int followRouteWithFinalAlignment(const Pose2D route[], int count) {
  if (count <= 0)
    return 1;

  // Open aisle points use goThrough() so the robot does not stop at every node.
  // The approach and final poses use goToPose() for accurate alignment before
  // entering tight warehouse or machine service areas.
  if (count == 1) {
    Pose2D goal = route[0];
    setDebugTarget("goToPose", goal, 0, count);
    return nav->goToPose(goal.x, goal.y, goal.theta);
  }

  if (routeIndex < count - 2) {
    Pose2D waypoint = route[routeIndex];
    setDebugTarget("goThrough", waypoint, routeIndex, count);
    if (nav->goThrough(waypoint.x, waypoint.y))
      ++routeIndex;
    return 0;
  }

  if (routeIndex == count - 2) {
    Pose2D approach = route[routeIndex];
    setDebugTarget("goToPose", approach, routeIndex, count);
    if (nav->goToPose(approach.x, approach.y, approach.theta))
      ++routeIndex;
    return 0;
  }

  Pose2D goal = route[count - 1];
  setDebugTarget("goToPose", goal, routeIndex, count);
  return nav->goToPose(goal.x, goal.y, goal.theta);
}

static int goToPose(Pose2D pose) {
  setDebugTarget("goToPose", pose, 0, 1);
  return nav->goToPose(pose.x, pose.y, pose.theta);
}

static int backToPose(Pose2D pose) {
  setDebugTarget("backTo", pose, 0, 1);
  return nav->backTo(pose.x, pose.y);
}

static void printInitialPoseOnce() {
  static bool printed = false;

  if (debugEnabled(DEBUG_STATE) && !printed && nav->poseValid()) {
    Pose2D pose = nav->pose();
    std::printf("Initial pose: x=%.3f y=%.3f theta=%.3f\n", pose.x, pose.y, pose.theta);
    printed = true;
  }
}

static bool validBay(int bay) {
  return bay >= 0 && bay < MAP_MACHINE_BAY_COUNT;
}

int main() {
  Navigation navigation;
  nav = &navigation;
  nav->init();

  while (nav->step()) {
    printInitialPoseOnce();

    switch (state) {
      case WAIT_FOR_POSE:
        if (nav->poseValid())
          changeState(WAIT_FOR_ORDER);
        break;

      case WAIT_FOR_ORDER:
        if (nav->lastOrder().size() >= MAP_BOX_COUNT) {
          demoBoxType = nav->lastOrder()[DEMO_BOX];
          if (debugEnabled(DEBUG_STATE)) {
            std::printf("ORDER is %s. BOX_%d type is %c. A bay=%d, B bay=%d.\n",
                        nav->lastOrder().c_str(), DEMO_BOX, demoBoxType, DEMO_A_BAY, DEMO_B_BAY);
            std::fflush(stdout);
          }
          changeState(ROUTE_TO_BOX_0);
        }
        break;

      case ROUTE_TO_BOX_0: {
        const Pose2D path[] = {
          MAP_P21_WEST_SOUTH,
          MAP_P10_WEST_CENTER,
          MAP_IN_FRONT[DEMO_BOX],
          MAP_IN_PICK[DEMO_BOX]
        };
        if (followRouteWithFinalAlignment(path, ARRAY_COUNT(path)))
          changeState(PICK_BOX_0);
        break;
      }

      case PICK_BOX_0:
        nav->magnetPick();
        if (nav->magnetIsOn() && nav->wait(0.45))
          changeState(CLEAR_BOX_0);
        break;

      case CLEAR_BOX_0:
        if (backToPose(MAP_IN_FRONT[DEMO_BOX]))
          changeState(DECIDE_BOX_0_DESTINATION);
        break;

      case DECIDE_BOX_0_DESTINATION:
        if (demoBoxType == 'R')
          changeState(ROUTE_BOX_0_TO_A_INPUT);
        else if (demoBoxType == 'G')
          changeState(ROUTE_TO_B_INPUT);
        else
          changeState(ROUTE_TO_OUTGOING);
        break;

      case ROUTE_BOX_0_TO_A_INPUT: {
        const Pose2D path[] = {
          MAP_MACHINE_A_INPUT_CLEAR_BAY[DEMO_A_BAY],
          MAP_MACHINE_A_INPUT_BAY[DEMO_A_BAY]
        };
        if (followRouteWithFinalAlignment(path, ARRAY_COUNT(path)))
          changeState(DROP_AT_MACHINE_A);
        break;
      }

      case DROP_AT_MACHINE_A:
        nav->magnetDrop();
        if (nav->wait(0.25))
          changeState(CLEAR_MACHINE_A_INPUT);
        break;

      case CLEAR_MACHINE_A_INPUT:
        if (backToPose(MAP_MACHINE_A_INPUT_CLEAR_BAY[DEMO_A_BAY]))
          changeState(ROUTE_TO_A_OUTPUT_APPROACH);
        break;

      case ROUTE_TO_A_OUTPUT_APPROACH: {
        const Pose2D path[] = {
          MAP_P10_WEST_CENTER,
          MAP_IN_FRONT[DEMO_BOX],
          MAP_P4_TOP_CENTER,
          MAP_MACHINE_A_OUTPUT_APPROACH_BAY[DEMO_A_BAY]
        };
        if (followRouteWithFinalAlignment(path, ARRAY_COUNT(path)))
          changeState(WAIT_FOR_MACHINE_A_READY);
        break;
      }

      case WAIT_FOR_MACHINE_A_READY:
        nav->stop();
        if (nav->machineAReadyPose(DEMO_BOX, aReadyPickPose)) {
          int bay = nav->machineAReadyBay(DEMO_BOX);
          if (validBay(bay))
            activeABay = bay;
          changeState(ENTER_A_OUTPUT);
        }
        break;

      case ENTER_A_OUTPUT:
        if (goToPose(aReadyPickPose))
          changeState(PICK_FROM_MACHINE_A);
        break;

      case PICK_FROM_MACHINE_A:
        nav->magnetPick();
        if (nav->magnetIsOn() && nav->wait(0.45))
          changeState(CLEAR_MACHINE_A_OUTPUT);
        break;

      case CLEAR_MACHINE_A_OUTPUT:
        if (backToPose(MAP_MACHINE_A_OUTPUT_CLEAR_BAY[activeABay]))
          changeState(ROUTE_TO_B_INPUT);
        break;

      case ROUTE_TO_B_INPUT:
        if (demoBoxType == 'R') {
          const Pose2D path[] = {
            MAP_P13_CENTER,
            MAP_MACHINE_B_INPUT_CLEAR_BAY[DEMO_B_BAY],
            MAP_MACHINE_B_INPUT_BAY[DEMO_B_BAY]
          };
          if (followRouteWithFinalAlignment(path, ARRAY_COUNT(path)))
            changeState(DROP_AT_MACHINE_B);
        } else {
          const Pose2D path[] = {
            MAP_IN_FRONT[DEMO_BOX],
            MAP_P4_TOP_CENTER,
            MAP_P13_CENTER,
            MAP_MACHINE_B_INPUT_CLEAR_BAY[DEMO_B_BAY],
            MAP_MACHINE_B_INPUT_BAY[DEMO_B_BAY]
          };
          if (followRouteWithFinalAlignment(path, ARRAY_COUNT(path)))
            changeState(DROP_AT_MACHINE_B);
        }
        break;

      case DROP_AT_MACHINE_B:
        nav->magnetDrop();
        if (nav->wait(0.25))
          changeState(CLEAR_MACHINE_B_INPUT);
        break;

      case CLEAR_MACHINE_B_INPUT:
        if (backToPose(MAP_MACHINE_B_INPUT_CLEAR_BAY[DEMO_B_BAY]))
          changeState(ROUTE_TO_B_OUTPUT_APPROACH);
        break;

      case ROUTE_TO_B_OUTPUT_APPROACH: {
        const Pose2D path[] = {
          MAP_P13_CENTER,
          MAP_P4V_NORTH_CENTER,
          MAP_P5_NORTH_EAST,
          MAP_MACHINE_B_OUTPUT_APPROACH_BAY[DEMO_B_BAY]
        };
        if (followRouteWithFinalAlignment(path, ARRAY_COUNT(path)))
          changeState(WAIT_FOR_MACHINE_B_READY);
        break;
      }

      case WAIT_FOR_MACHINE_B_READY:
        nav->stop();
        if (nav->machineBReadyPose(DEMO_BOX, bReadyPickPose)) {
          int bay = nav->machineBReadyBay(DEMO_BOX);
          if (validBay(bay))
            activeBBay = bay;
          changeState(ENTER_B_OUTPUT);
        }
        break;

      case ENTER_B_OUTPUT:
        if (goToPose(bReadyPickPose))
          changeState(PICK_FROM_MACHINE_B);
        break;

      case PICK_FROM_MACHINE_B:
        nav->magnetPick();
        if (nav->magnetIsOn() && nav->wait(0.45))
          changeState(CLEAR_MACHINE_B_OUTPUT);
        break;

      case CLEAR_MACHINE_B_OUTPUT:
        if (backToPose(MAP_MACHINE_B_OUTPUT_CLEAR_BAY[activeBBay]))
          changeState(ROUTE_TO_OUTGOING);
        break;

      case ROUTE_TO_OUTGOING:
        if (demoBoxType == 'B') {
          const Pose2D path[] = {
            MAP_IN_FRONT[DEMO_BOX],
            MAP_P10_WEST_CENTER,
            MAP_P21_WEST_SOUTH,
            MAP_P22_CENTER_SOUTH,
            MAP_OUT_FRONT[DEMO_BOX],
            MAP_OUT_DROP[DEMO_BOX]
          };
          if (followRouteWithFinalAlignment(path, ARRAY_COUNT(path)))
            changeState(DROP_AT_OUTGOING);
        } else {
          const Pose2D path[] = {
            MAP_MACHINE_B_OUTPUT_CLEAR_BAY[activeBBay],
            MAP_OUT_FRONT[3],
            MAP_OUT_FRONT[DEMO_BOX],
            MAP_OUT_DROP[DEMO_BOX]
          };
          if (followRouteWithFinalAlignment(path, ARRAY_COUNT(path)))
            changeState(DROP_AT_OUTGOING);
        }
        break;

      case DROP_AT_OUTGOING:
        nav->magnetDrop();
        if (nav->wait(0.40))
          changeState(CLEAR_OUTGOING);
        break;

      case CLEAR_OUTGOING:
        if (backToPose(MAP_OUT_FRONT[DEMO_BOX]))
          changeState(FINISHED);
        break;

      case FINISHED:
        nav->stop();
        if (debugEnabled(DEBUG_STATE) && !finalMessagePrinted) {
          std::printf("Standard C++ controller finished. Score reported by supervisor: %d\n", nav->score());
          std::printf("Use example_student_c or example_student_python for compact language examples.\n");
          finalMessagePrinted = true;
        }
        break;
    }

    printDetailedDebugIfDue();
  }

  nav->stop();
  return 0;
}
