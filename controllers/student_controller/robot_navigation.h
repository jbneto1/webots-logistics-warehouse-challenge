#ifndef ROBOT_NAVIGATION_H
#define ROBOT_NAVIGATION_H

#include <stdbool.h>

/* Webots controller time step in milliseconds. */
#define TIME_STEP 32

/* Pose2D is the robot pose on the ENU floor. */
typedef struct {
  double x;      /* Robot center X position in meters. */
  double y;      /* Robot center Y position in meters. */
  double theta;  /* Robot yaw angle in radians around +Z. */
} Pose2D;

void nav_init(void);                         /* Start the Webots devices and navigation helper. */
bool nav_step(void);                         /* Advance one step and read supervisor messages. */
void nav_stop(void);                         /* Stop both wheels immediately. */
void nav_reset_actions(void);                /* Reset timers/arcs when a state changes. */

bool nav_pose_valid(void);                   /* True after the first POSE message arrives. */
Pose2D nav_pose(void);                       /* Latest robot pose. */
const char *nav_last_order(void);            /* Latest order string, for example "RRGB". */
int nav_score(void);                         /* Latest score from the supervisor. */
const char *nav_attached_box(void);          /* Attached box name, or "none". */

/* Machine readiness API.
 * box_index is 0..3.  The functions return information sent by the supervisor
 * when a processed box appears at a machine output bay.
 */
bool nav_machine_a_ready(int box_index);     /* True when this box is ready at a Machine A output. */
bool nav_machine_b_ready(int box_index);     /* True when this box is ready at a Machine B output. */
int nav_machine_a_ready_bay(int box_index);  /* Ready output bay, or -1 if not ready. */
int nav_machine_b_ready_bay(int box_index);  /* Ready output bay, or -1 if not ready. */
bool nav_machine_a_ready_pose(int box_index, Pose2D *pose_out); /* Robot-center pickup pose if ready. */
bool nav_machine_b_ready_pose(int box_index, Pose2D *pose_out); /* Robot-center pickup pose if ready. */

double nav_normalize_angle(double angle);    /* Keep angle inside -pi to +pi. */

/* Non-blocking actions: call every loop tick until they return true. */
bool nav_go_to(double x, double y);          /* Drive accurately to an X,Y point and stop there. */
bool nav_go_through(double x, double y);     /* Drive through a waypoint without stopping at it. */
bool nav_go_to_pose(double x, double y, double theta); /* Drive to X,Y, then face theta. */
bool nav_rotate_to(double theta);            /* Rotate in place to theta. */
bool nav_wait(double seconds);               /* Wait while stopped. */
bool nav_back_up(double seconds);            /* Reverse for a timed movement. */
bool nav_back_to(double x, double y);        /* Reverse to a clear point while keeping the current front direction. */
bool nav_move_arc(double radius_m, double angle_rad, bool clockwise); /* Optional non-blocking arc. */
bool nav_move_circle(double radius_m, double angle_rad, bool clockwise); /* Arc alias. */

void magnet_pick(void);                      /* Request box attachment. */
void magnet_drop(void);                      /* Request box release. */
bool magnet_is_on(void);                     /* Local magnet state. */

#endif
