#include "robot_navigation.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/emitter.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/robot.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* These values must match the robot model in logistics_pbl_enu.wbt. */
#define WHEEL_RADIUS_M 0.022
#define AXLE_LENGTH_M 0.128
/* Tuning: wheel speed ceiling for all forward/reverse/arc commands. */
#define MAX_WHEEL_SPEED_RAD_S 18.5

/* Navigation tolerances: precise stops, drive-through waypoints, and about 3 degrees. */
#define POSITION_TOLERANCE_M 0.025
#define BACK_POSITION_TOLERANCE_M 0.055
/* Tuning: drive-through waypoints are considered reached inside this radius. */
#define THROUGH_TOLERANCE_M 0.100
#define ANGLE_TOLERANCE_RAD 0.055
/* Tuning: final-target speed reduction starts only inside this distance. */
#define FINAL_SLOWDOWN_RADIUS_M 0.045
/* Tuning: forward drive speed limits. */
#define FINAL_MIN_LINEAR_M_S 0.035
#define FINAL_MAX_LINEAR_M_S 0.170
#define THROUGH_MIN_LINEAR_M_S 0.085
#define THROUGH_MAX_LINEAR_M_S 0.225
#define ARC_LINEAR_SPEED_M_S 0.110

/* Webots device handles. */
static WbDeviceTag left_motor = 0;
static WbDeviceTag right_motor = 0;
static WbDeviceTag magnet_emitter = 0;
static WbDeviceTag task_receiver = 0;

/* Latest pose and task information received from the supervisor. */
static Pose2D current_pose = {0.0, 0.0, 0.0};
static bool have_pose = false;
static char last_order[16] = "";
static int latest_score = 0;
static char attached_box[64] = "none";
static int machine_a_ready_mask = 0;
static int machine_b_ready_mask = 0;
static int machine_a_ready_bay[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
static int machine_b_ready_bay[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
static Pose2D machine_a_ready_pose[8];
static Pose2D machine_b_ready_pose[8];
static bool machine_a_have_ready_pose[8] = {false};
static bool machine_b_have_ready_pose[8] = {false};
static bool magnet_on = false;

/* Memory used by nav_wait(). */
static bool wait_active = false;
static double wait_start_time = 0.0;

static bool back_up_active = false;
static double back_up_start_time = 0.0;

/* Memory used by nav_move_arc(). */
static bool arc_active = false;
static double arc_radius = 0.0;
static double arc_target_angle = 0.0;
static bool arc_clockwise = false;
static double arc_previous_theta = 0.0;
static double arc_progress = 0.0;

/* Limit a value to the range [low, high]. */
static double clamp(double value, double low, double high) {
  if (value < low)
    return low;
  if (value > high)
    return high;
  return value;
}

/* Convert any angle to the range -pi to +pi. */
double nav_normalize_angle(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

/* Convert desired robot speed into left and right wheel speeds. */
static void set_wheel_speeds(double linear_m_s, double angular_rad_s) {
  double left = (linear_m_s - angular_rad_s * AXLE_LENGTH_M * 0.5) / WHEEL_RADIUS_M;
  double right = (linear_m_s + angular_rad_s * AXLE_LENGTH_M * 0.5) / WHEEL_RADIUS_M;

  left = clamp(left, -MAX_WHEEL_SPEED_RAD_S, MAX_WHEEL_SPEED_RAD_S);
  right = clamp(right, -MAX_WHEEL_SPEED_RAD_S, MAX_WHEEL_SPEED_RAD_S);

  wb_motor_set_velocity(left_motor, left);
  wb_motor_set_velocity(right_motor, right);
}

void nav_stop(void) {
  set_wheel_speeds(0.0, 0.0);
}

void nav_reset_actions(void) {
  wait_active = false;
  back_up_active = false;
  arc_active = false;
}

/* Send one text message to the supervisor. */
static void send_text(const char *message) {
  if (magnet_emitter)
    wb_emitter_send(magnet_emitter, message, (int)strlen(message));
}

/* Read all available messages from the supervisor. */
static void read_task_messages(void) {
  while (wb_receiver_get_queue_length(task_receiver) > 0) {
    const void *raw = wb_receiver_get_data(task_receiver);
    int size = wb_receiver_get_data_size(task_receiver);
    char msg[128];

    if (size >= (int)sizeof(msg))
      size = (int)sizeof(msg) - 1;
    memcpy(msg, raw, size);
    msg[size] = '\0';

    double x, y, theta;
    int score;
    int ready_a = 0;
    int ready_b = 0;
    char attached[64];

    int pose_fields = sscanf(msg, "POSE %lf %lf %lf %d %63s %d %d",
                             &x, &y, &theta, &score, attached, &ready_a, &ready_b);
    if (pose_fields == 7 || pose_fields == 5) {
      current_pose.x = x;
      current_pose.y = y;
      current_pose.theta = nav_normalize_angle(theta);
      latest_score = score;
      strncpy(attached_box, attached, sizeof(attached_box) - 1);
      attached_box[sizeof(attached_box) - 1] = '\0';
      if (pose_fields == 7) {
        machine_a_ready_mask = ready_a;
        machine_b_ready_mask = ready_b;
        for (int i = 0; i < 8; ++i) {
          if ((machine_a_ready_mask & (1 << i)) == 0) {
            machine_a_ready_bay[i] = -1;
            machine_a_have_ready_pose[i] = false;
          }
          if ((machine_b_ready_mask & (1 << i)) == 0) {
            machine_b_ready_bay[i] = -1;
            machine_b_have_ready_pose[i] = false;
          }
        }
      }
      have_pose = true;
    } else if (strncmp(msg, "READY ", 6) == 0) {
      char machine = '?';
      int box_index = -1;
      int ready_bay = -1;
      double rx = 0.0, ry = 0.0, rtheta = 0.0;
      int fields = sscanf(msg, "READY %c %d %d %lf %lf %lf",
                          &machine, &box_index, &ready_bay, &rx, &ry, &rtheta);
      if (box_index >= 0 && box_index < 8) {
        if (machine == 'A') {
          machine_a_ready_mask |= (1 << box_index);
          if (fields == 6) {
            machine_a_ready_bay[box_index] = ready_bay;
            machine_a_ready_pose[box_index].x = rx;
            machine_a_ready_pose[box_index].y = ry;
            machine_a_ready_pose[box_index].theta = nav_normalize_angle(rtheta);
            machine_a_have_ready_pose[box_index] = true;
          }
        } else if (machine == 'B') {
          machine_b_ready_mask |= (1 << box_index);
          if (fields == 6) {
            machine_b_ready_bay[box_index] = ready_bay;
            machine_b_ready_pose[box_index].x = rx;
            machine_b_ready_pose[box_index].y = ry;
            machine_b_ready_pose[box_index].theta = nav_normalize_angle(rtheta);
            machine_b_have_ready_pose[box_index] = true;
          }
        }
      }
    } else if (strncmp(msg, "CLEAR ", 6) == 0) {
      char machine = '?';
      int box_index = -1;
      if (sscanf(msg, "CLEAR %c %d", &machine, &box_index) == 2 && box_index >= 0 && box_index < 8) {
        if (machine == 'A') {
          machine_a_ready_mask &= ~(1 << box_index);
          machine_a_ready_bay[box_index] = -1;
          machine_a_have_ready_pose[box_index] = false;
        } else if (machine == 'B') {
          machine_b_ready_mask &= ~(1 << box_index);
          machine_b_ready_bay[box_index] = -1;
          machine_b_have_ready_pose[box_index] = false;
        }
      }
    } else if (strncmp(msg, "ORDER ", 6) == 0) {
      strncpy(last_order, msg + 6, sizeof(last_order) - 1);
      last_order[sizeof(last_order) - 1] = '\0';
      printf("Received ORDER %s\n", last_order);
    } else if (strcmp(msg, "START") == 0) {
      printf("Received START\n");
    }

    wb_receiver_next_packet(task_receiver);
  }
}

void nav_init(void) {
  wb_robot_init();

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  magnet_emitter = wb_robot_get_device("magnet_emitter");
  task_receiver = wb_robot_get_device("task_receiver");

  if (!left_motor || !right_motor || !magnet_emitter || !task_receiver) {
    fprintf(stderr, "Missing a required Webots device. Check names in logistics_pbl_enu.wbt.\n");
    exit(1);
  }

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  wb_receiver_enable(task_receiver, TIME_STEP);

  printf("Navigation API ready. Students edit student_controller.c and warehouse_map.h first.\n");
}

bool nav_step(void) {
  if (wb_robot_step(TIME_STEP) == -1)
    return false;

  read_task_messages();
  return true;
}

bool nav_pose_valid(void) {
  return have_pose;
}

Pose2D nav_pose(void) {
  return current_pose;
}

const char *nav_last_order(void) {
  return last_order;
}

int nav_score(void) {
  return latest_score;
}

const char *nav_attached_box(void) {
  return attached_box;
}

bool nav_machine_a_ready(int box_index) {
  if (box_index < 0 || box_index >= 8)
    return false;
  return (machine_a_ready_mask & (1 << box_index)) != 0;
}

bool nav_machine_b_ready(int box_index) {
  if (box_index < 0 || box_index >= 8)
    return false;
  return (machine_b_ready_mask & (1 << box_index)) != 0;
}

int nav_machine_a_ready_bay(int box_index) {
  if (!nav_machine_a_ready(box_index))
    return -1;
  return machine_a_ready_bay[box_index];
}

int nav_machine_b_ready_bay(int box_index) {
  if (!nav_machine_b_ready(box_index))
    return -1;
  return machine_b_ready_bay[box_index];
}

bool nav_machine_a_ready_pose(int box_index, Pose2D *pose_out) {
  if (!nav_machine_a_ready(box_index) || !pose_out || !machine_a_have_ready_pose[box_index])
    return false;
  *pose_out = machine_a_ready_pose[box_index];
  return true;
}

bool nav_machine_b_ready_pose(int box_index, Pose2D *pose_out) {
  if (!nav_machine_b_ready(box_index) || !pose_out || !machine_b_have_ready_pose[box_index])
    return false;
  *pose_out = machine_b_ready_pose[box_index];
  return true;
}

bool nav_rotate_to(double theta) {
  if (!have_pose) {
    nav_stop();
    return false;
  }

  double error = nav_normalize_angle(theta - current_pose.theta);

  if (fabs(error) < ANGLE_TOLERANCE_RAD) {
    nav_stop();
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
    nav_stop();
    return false;
  }

  double dx = x - current_pose.x;
  double dy = y - current_pose.y;
  double distance = sqrt(dx * dx + dy * dy);
  const double tolerance = through_waypoint ? THROUGH_TOLERANCE_M : POSITION_TOLERANCE_M;

  if (distance < tolerance) {
    if (stop_at_goal)
      nav_stop();
    return true;
  }

  double target_heading = atan2(dy, dx);
  double heading_error = nav_normalize_angle(target_heading - current_pose.theta);

  double min_linear = through_waypoint ? THROUGH_MIN_LINEAR_M_S : FINAL_MIN_LINEAR_M_S;
  double max_linear = through_waypoint ? THROUGH_MAX_LINEAR_M_S : FINAL_MAX_LINEAR_M_S;
  double linear = clamp(1.35 * distance, min_linear, max_linear);

  /* Tuning: final stops only ease in inside FINAL_SLOWDOWN_RADIUS_M.
   * Drive-through waypoints keep speed so the next target can be sent sooner.
   */
  if (!through_waypoint && distance < FINAL_SLOWDOWN_RADIUS_M)
    linear = clamp(0.95 * distance, 0.030, 0.115);

  if (fabs(heading_error) > 1.05)
    linear = 0.0;
  else
    linear *= clamp(1.0 - fabs(heading_error) / 1.20, 0.35, 1.0);

  double angular = clamp(3.2 * heading_error, -1.70, 1.70);
  set_wheel_speeds(linear, angular);
  return false;
}

bool nav_go_to(double x, double y) {
  return drive_to_xy(x, y, true, false);
}

bool nav_go_through(double x, double y) {
  return drive_to_xy(x, y, false, true);
}

bool nav_go_to_pose(double x, double y, double theta) {
  if (!nav_go_to(x, y))
    return false;

  return nav_rotate_to(theta);
}

bool nav_wait(double seconds) {
  if (!wait_active) {
    wait_active = true;
    wait_start_time = wb_robot_get_time();
    nav_stop();
  }

  if (wb_robot_get_time() - wait_start_time >= seconds) {
    wait_active = false;
    return true;
  }

  nav_stop();
  return false;
}


bool nav_back_up(double seconds) {
  /* Move slowly backward for a short time.
   * Students use this to clear a warehouse or machine before rotating.
   */
  if (!back_up_active) {
    back_up_active = true;
    back_up_start_time = wb_robot_get_time();
  }

  if (wb_robot_get_time() - back_up_start_time >= seconds) {
    back_up_active = false;
    nav_stop();
    return true;
  }

  set_wheel_speeds(-0.120, 0.0);
  return false;
}

bool nav_back_to(double x, double y) {
  if (!have_pose) {
    nav_stop();
    return false;
  }

  double dx = x - current_pose.x;
  double dy = y - current_pose.y;
  double distance = sqrt(dx * dx + dy * dy);

  /* Reversing is a clearance maneuver, not a precision docking maneuver.  A
   * slightly larger tolerance makes the robot clear the pocket quickly instead
   * of creeping for the last few centimeters.
   */
  if (distance < BACK_POSITION_TOLERANCE_M) {
    nav_stop();
    return true;
  }

  /* When reversing to a target, the robot's front should point away from the
   * target.  This lets it leave a bay or machine pocket before rotating.
   */
  double desired_front_heading = nav_normalize_angle(atan2(dy, dx) + M_PI);
  double heading_error = nav_normalize_angle(desired_front_heading - current_pose.theta);

  double linear = -clamp(1.85 * distance, 0.090, 0.220);
  if (distance < 0.090)
    linear = -clamp(1.55 * distance, 0.075, 0.140);

  /* Be less conservative than forward driving: while backing out, small heading
   * errors are corrected while still moving so the clearance step is faster.
   */
  if (fabs(heading_error) > 1.05)
    linear = 0.0;
  else
    linear *= clamp(1.0 - fabs(heading_error) / 1.20, 0.55, 1.0);

  double angular = clamp(3.4 * heading_error, -1.75, 1.75);
  set_wheel_speeds(linear, angular);
  return false;
}

bool nav_move_arc(double radius_m, double angle_rad, bool clockwise) {
  if (!have_pose) {
    nav_stop();
    return false;
  }

  if (radius_m < 0.06)
    radius_m = 0.06;

  double target_angle = fabs(angle_rad);

  if (!arc_active || fabs(arc_radius - radius_m) > 1e-6 ||
      fabs(arc_target_angle - target_angle) > 1e-6 || arc_clockwise != clockwise) {
    arc_active = true;
    arc_radius = radius_m;
    arc_target_angle = target_angle;
    arc_clockwise = clockwise;
    arc_previous_theta = current_pose.theta;
    arc_progress = 0.0;
  }

  double delta = nav_normalize_angle(current_pose.theta - arc_previous_theta);
  arc_progress += fabs(delta);
  arc_previous_theta = current_pose.theta;

  if (arc_progress >= arc_target_angle) {
    arc_active = false;
    nav_stop();
    return true;
  }

  double linear = ARC_LINEAR_SPEED_M_S;
  double angular = linear / radius_m;

  if (clockwise)
    angular = -angular;

  set_wheel_speeds(linear, angular);
  return false;
}

bool nav_move_circle(double radius_m, double angle_rad, bool clockwise) {
  return nav_move_arc(radius_m, angle_rad, clockwise);
}

void magnet_pick(void) {
  if (!magnet_on) {
    magnet_on = true;
    send_text("MAGNET_ON");
    printf("Electromagnet ON\n");
  }
}

void magnet_drop(void) {
  if (magnet_on) {
    magnet_on = false;
    send_text("MAGNET_OFF");
    printf("Electromagnet OFF\n");
  }
}

bool magnet_is_on(void) {
  return magnet_on;
}
