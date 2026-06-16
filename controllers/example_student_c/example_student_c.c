#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/emitter.h>
#include <webots/connector.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/robot.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ARRAY_COUNT(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define TIME_STEP 32
#define EXAMPLE_BOX 0
#define MAP_BOX_COUNT 4

/* Physical and navigation tuning mirrored from the C++ controller. Through
 * tolerance/speeds affect intermediate waypoint slowdowns; position/angle
 * tolerances affect final stop-and-align poses. */
#define WHEEL_RADIUS_M 0.022
#define AXLE_LENGTH_M 0.128
#define MAX_WHEEL_SPEED_RAD_S 18.5
#define POSITION_TOLERANCE_M 0.025
#define THROUGH_TOLERANCE_M 0.100
#define BACK_POSITION_TOLERANCE_M 0.055
#define ANGLE_TOLERANCE_RAD 0.055

typedef struct {
  double x;
  double y;
  double theta;
} Pose2D;

static const double FACE_EAST = 0.0;
static const double FACE_NORTH = M_PI / 2.0;
static const double FACE_SOUTH = -M_PI / 2.0;

static const Pose2D MAP_P10_WEST_CENTER = {-0.695, 0.000, -M_PI / 2.0};
static const Pose2D MAP_P21_WEST_SOUTH = {-0.695, -0.425, FACE_EAST};
static const Pose2D MAP_P22_CENTER_SOUTH = {0.000, -0.244, FACE_EAST};

static const Pose2D MAP_IN_PICK[MAP_BOX_COUNT] = {
  {-0.695, 0.395, FACE_NORTH},
  {-0.545, 0.395, FACE_NORTH},
  {-0.400, 0.395, FACE_NORTH},
  {-0.245, 0.395, FACE_NORTH}
};

static const Pose2D MAP_IN_FRONT[MAP_BOX_COUNT] = {
  {-0.695, 0.244, FACE_NORTH},
  {-0.545, 0.244, FACE_NORTH},
  {-0.400, 0.244, FACE_NORTH},
  {-0.245, 0.244, FACE_NORTH}
};

static const Pose2D MAP_OUT_FRONT[MAP_BOX_COUNT] = {
  {0.245, -0.244, FACE_SOUTH},
  {0.395, -0.244, FACE_SOUTH},
  {0.545, -0.244, FACE_SOUTH},
  {0.695, -0.244, FACE_SOUTH}
};

static const Pose2D MAP_OUT_DROP[MAP_BOX_COUNT] = {
  {0.245, -0.430, FACE_SOUTH},
  {0.395, -0.430, FACE_SOUTH},
  {0.545, -0.430, FACE_SOUTH},
  {0.695, -0.430, FACE_SOUTH}
};

typedef enum {
  WAIT_FOR_POSE,
  WAIT_FOR_ORDER,
  ROUTE_TO_BOX,
  PICK_BOX,
  CLEAR_BOX,
  DECIDE_DESTINATION,
  ROUTE_TO_OUTGOING,
  DROP_AT_OUTGOING,
  CLEAR_OUTGOING,
  FINISHED
} State;

static WbDeviceTag left_motor = 0;
static WbDeviceTag right_motor = 0;
static WbDeviceTag magnet_emitter = 0;
static WbDeviceTag task_receiver = 0;
static WbDeviceTag electromagnet_connector = 0;

static Pose2D current_pose = {0.0, 0.0, 0.0};
static bool have_pose = false;
static char order[16] = "";
static int score = 0;
static char attached_box[64] = "none";
static bool magnet_on = false;

static State state = WAIT_FOR_POSE;
static int route_index = 0;
static char box_type = '?';
static bool wait_active = false;
static double wait_start_time = 0.0;
static int final_message_printed = 0;

static double clamp(double value, double low, double high) {
  if (value < low)
    return low;
  if (value > high)
    return high;
  return value;
}

static double normalize_angle(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

static void set_wheel_speeds(double linear_m_s, double angular_rad_s) {
  double left = (linear_m_s - angular_rad_s * AXLE_LENGTH_M * 0.5) / WHEEL_RADIUS_M;
  double right = (linear_m_s + angular_rad_s * AXLE_LENGTH_M * 0.5) / WHEEL_RADIUS_M;

  wb_motor_set_velocity(left_motor, clamp(left, -MAX_WHEEL_SPEED_RAD_S, MAX_WHEEL_SPEED_RAD_S));
  wb_motor_set_velocity(right_motor, clamp(right, -MAX_WHEEL_SPEED_RAD_S, MAX_WHEEL_SPEED_RAD_S));
}

static void stop_robot(void) {
  set_wheel_speeds(0.0, 0.0);
}

static void send_text(const char *message) {
  wb_emitter_send(magnet_emitter, message, (int)strlen(message));
}

static void read_messages(void) {
  while (wb_receiver_get_queue_length(task_receiver) > 0) {
    const void *raw = wb_receiver_get_data(task_receiver);
    int size = wb_receiver_get_data_size(task_receiver);
    char msg[128];

    if (size >= (int)sizeof(msg))
      size = (int)sizeof(msg) - 1;
    memcpy(msg, raw, size);
    msg[size] = '\0';

    double x, y, theta;
    int pose_score;
    char attached[64] = "none";

    if (sscanf(msg, "POSE %lf %lf %lf %d %63s", &x, &y, &theta, &pose_score, attached) == 5) {
      current_pose.x = x;
      current_pose.y = y;
      current_pose.theta = normalize_angle(theta);
      score = pose_score;
      strncpy(attached_box, attached, sizeof(attached_box) - 1);
      attached_box[sizeof(attached_box) - 1] = '\0';
      have_pose = true;
    } else if (strncmp(msg, "ORDER ", 6) == 0) {
      strncpy(order, msg + 6, sizeof(order) - 1);
      order[sizeof(order) - 1] = '\0';
      printf("C example received ORDER %s\n", order);
    } else if (strcmp(msg, "START") == 0) {
      printf("C example received START\n");
    }

    wb_receiver_next_packet(task_receiver);
  }
}

static bool rotate_to(double theta) {
  const double error = normalize_angle(theta - current_pose.theta);

  if (fabs(error) < ANGLE_TOLERANCE_RAD) {
    stop_robot();
    return true;
  }

  double angular = clamp(2.5 * error, -1.25, 1.25);
  if (fabs(angular) < 0.22)
    angular = (angular < 0.0) ? -0.22 : 0.22;

  set_wheel_speeds(0.0, angular);
  return false;
}

static bool drive_to_xy(double x, double y, bool stop_at_goal, bool through_waypoint) {
  if (!have_pose) {
    stop_robot();
    return false;
  }

  const double dx = x - current_pose.x;
  const double dy = y - current_pose.y;
  const double distance = sqrt(dx * dx + dy * dy);
  const double tolerance = through_waypoint ? THROUGH_TOLERANCE_M : POSITION_TOLERANCE_M;

  if (distance < tolerance) {
    if (stop_at_goal)
      stop_robot();
    return true;
  }

  const double target_heading = atan2(dy, dx);
  const double heading_error = normalize_angle(target_heading - current_pose.theta);
  double linear = clamp(1.35 * distance, through_waypoint ? 0.085 : 0.035, through_waypoint ? 0.225 : 0.170);

  if (fabs(heading_error) > 1.05)
    linear = 0.0;
  else
    linear *= clamp(1.0 - fabs(heading_error) / 1.20, 0.35, 1.0);

  set_wheel_speeds(linear, clamp(3.2 * heading_error, -1.70, 1.70));
  return false;
}

static bool go_through(Pose2D pose) {
  return drive_to_xy(pose.x, pose.y, false, true);
}

static bool go_to_pose(Pose2D pose) {
  if (!drive_to_xy(pose.x, pose.y, true, false))
    return false;
  return rotate_to(pose.theta);
}

static bool wait_seconds(double seconds) {
  if (!wait_active) {
    wait_active = true;
    wait_start_time = wb_robot_get_time();
    stop_robot();
  }

  if (wb_robot_get_time() - wait_start_time >= seconds) {
    wait_active = false;
    return true;
  }

  stop_robot();
  return false;
}

static bool back_to(Pose2D pose) {
  const double dx = pose.x - current_pose.x;
  const double dy = pose.y - current_pose.y;
  const double distance = sqrt(dx * dx + dy * dy);

  if (distance < BACK_POSITION_TOLERANCE_M) {
    stop_robot();
    return true;
  }

  const double desired_front_heading = normalize_angle(atan2(dy, dx) + M_PI);
  const double heading_error = normalize_angle(desired_front_heading - current_pose.theta);
  double linear = -clamp(1.85 * distance, 0.090, 0.220);

  if (fabs(heading_error) > 1.05)
    linear = 0.0;
  else
    linear *= clamp(1.0 - fabs(heading_error) / 1.20, 0.55, 1.0);

  set_wheel_speeds(linear, clamp(3.4 * heading_error, -1.75, 1.75));
  return false;
}

static void magnet_pick(void) {
  if (magnet_on)
    return;

  if (wb_connector_get_presence(electromagnet_connector))
    wb_connector_lock(electromagnet_connector);

  if (wb_connector_is_locked(electromagnet_connector)) {
    send_text("MAGNET_ON");
    magnet_on = true;
    printf("C example connector locked\n");
  }
}

static void magnet_drop(void) {
  if (magnet_on) {
    wb_connector_unlock(electromagnet_connector);
    send_text("MAGNET_OFF");
    magnet_on = false;
    printf("C example connector unlocked\n");
  }
}

static void change_state(State next_state) {
  state = next_state;
  route_index = 0;
  wait_active = false;
}

static bool follow_route(const Pose2D route[], int count) {
  if (count <= 0)
    return true;

  if (route_index < count - 1) {
    if (go_through(route[route_index]))
      ++route_index;
    return false;
  }

  return go_to_pose(route[count - 1]);
}

int main(void) {
  wb_robot_init();

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  magnet_emitter = wb_robot_get_device("magnet_emitter");
  task_receiver = wb_robot_get_device("task_receiver");
  electromagnet_connector = wb_robot_get_device("electromagnet_connector");

  if (!left_motor || !right_motor || !magnet_emitter || !task_receiver || !electromagnet_connector) {
    fprintf(stderr, "C example missing a required Webots device.\n");
    return 1;
  }

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  stop_robot();
  wb_receiver_enable(task_receiver, TIME_STEP);
  wb_connector_enable_presence(electromagnet_connector, TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {
    read_messages();

    switch (state) {
      case WAIT_FOR_POSE:
        if (have_pose)
          change_state(WAIT_FOR_ORDER);
        break;

      case WAIT_FOR_ORDER:
        if ((int)strlen(order) >= MAP_BOX_COUNT) {
          box_type = order[EXAMPLE_BOX];
          printf("C example: BOX_%d type is %c.\n", EXAMPLE_BOX, box_type);
          change_state(ROUTE_TO_BOX);
        }
        break;

      case ROUTE_TO_BOX: {
        const Pose2D path[] = {
          MAP_P21_WEST_SOUTH,
          MAP_P10_WEST_CENTER,
          MAP_IN_FRONT[EXAMPLE_BOX],
          MAP_IN_PICK[EXAMPLE_BOX]
        };
        if (follow_route(path, ARRAY_COUNT(path)))
          change_state(PICK_BOX);
        break;
      }

      case PICK_BOX:
        magnet_pick();
        if (magnet_on && wait_seconds(0.45))
          change_state(CLEAR_BOX);
        break;

      case CLEAR_BOX:
        if (back_to(MAP_P10_WEST_CENTER))
          change_state(DECIDE_DESTINATION);
        break;

      case DECIDE_DESTINATION:
        if (box_type == 'B') {
          change_state(ROUTE_TO_OUTGOING);
        } else {
          printf("C example stops here: BOX_%d is %c, so it needs a machine route.\n", EXAMPLE_BOX, box_type);
          change_state(FINISHED);
        }
        break;

      case ROUTE_TO_OUTGOING: {
        const Pose2D path[] = {
          MAP_P10_WEST_CENTER,
          MAP_P21_WEST_SOUTH,
          MAP_P22_CENTER_SOUTH,
          MAP_OUT_FRONT[EXAMPLE_BOX],
          MAP_OUT_DROP[EXAMPLE_BOX]
        };
        if (follow_route(path, ARRAY_COUNT(path)))
          change_state(DROP_AT_OUTGOING);
        break;
      }

      case DROP_AT_OUTGOING:
        magnet_drop();
        if (wait_seconds(0.40))
          change_state(CLEAR_OUTGOING);
        break;

      case CLEAR_OUTGOING:
        if (back_to(MAP_OUT_FRONT[EXAMPLE_BOX]))
          change_state(FINISHED);
        break;

      case FINISHED:
        stop_robot();
        if (!final_message_printed) {
          printf("C example finished. Score=%d attached=%s\n", score, attached_box);
          final_message_printed = 1;
        }
        break;
    }
  }

  stop_robot();
  wb_robot_cleanup();
  return 0;
}
