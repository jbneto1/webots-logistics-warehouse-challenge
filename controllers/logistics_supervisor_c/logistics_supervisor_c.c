/*
 * C-only logistics supervisor for Webots R2025a.
 *
 * The supervisor randomizes incoming boxes, broadcasts ORDER/POSE/READY,
 * implements a virtual magnet, processes red -> green at Machine A and
 * green -> blue at Machine B, and scores boxes in the outgoing warehouse.
 *
 * ENU note: Webots translations are [x, y, z].  The floor is X-Y.  Z is up.
 * Robot yaw is theta about +Z.  Robot local +X is forward.
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TIME_STEP 32
#define BOX_COUNT 4
#define MACHINE_BAY_COUNT 2
#define MSG_SIZE 224

/* Change this line to switch between random and teacher-defined boxes. */
#define ORDER_MODE_RANDOM 0
#define ORDER_MODE_MANUAL 1
#define TASK_ORDER_MODE ORDER_MODE_RANDOM

/* Used only when TASK_ORDER_MODE is ORDER_MODE_MANUAL. */
#define MANUAL_ORDER "RRGB"

#define MAGNET_POINT_DISTANCE 0.095
#define BOX_CENTER_DISTANCE_FROM_ROBOT 0.125
#define ATTACH_DISTANCE 0.115
#define BOX_Z 0.0325
#define PROCESSING_TIME_MIN 15.0
#define PROCESSING_TIME_MAX 25.0

typedef enum {
  PART_RED = 0,
  PART_GREEN = 1,
  PART_BLUE = 2
} PartState;

typedef struct {
  const char *box_def;
  const char *shape_def;
  WbNodeRef node;
  WbNodeRef shape;
  WbFieldRef translation;
  WbFieldRef rotation;
  WbFieldRef base_color;
  PartState state;
  double processing_until;
  PartState target_state;
  double target_pose[4];
  bool scored_a;
  bool scored_b;
  bool ready_a;
  bool ready_b;
  int ready_a_bay;
  int ready_b_bay;
  bool in_machine_input;
  char input_machine;
  int input_bay;
  bool delivered;
} BoxInfo;

typedef struct {
  char machine;
  int bay_index;
  double input_zone[4];       /* xmin, xmax, ymin, ymax, box-center coordinates */
  double output_box_pose[4];  /* box x, y, z, yaw */
  double output_robot_pose[3];/* robot x, y, theta for pickup */
  int input_box;              /* waiting or processing box index, -1 if empty */
  int output_box;             /* ready box index, -1 if output empty */
  bool processing;
} MachineBay;

static const char *BOX_DEFS[BOX_COUNT] = {"BOX_0", "BOX_1", "BOX_2", "BOX_3"};
static const char *SHAPE_DEFS[BOX_COUNT] = {"BOX_0_SHAPE", "BOX_1_SHAPE", "BOX_2_SHAPE", "BOX_3_SHAPE"};

static const double RED_COLOR[3] = {0.85, 0.05, 0.05};
static const double GREEN_COLOR[3] = {0.05, 0.75, 0.12};
static const double BLUE_COLOR[3] = {0.05, 0.20, 0.90};
static const double GRAY_COLOR[3] = {0.55, 0.55, 0.55};

static const double INCOMING_SLOTS[BOX_COUNT][2] = {
  {-0.695, 0.535},
  {-0.545, 0.535},
  {-0.400, 0.535},
  {-0.245, 0.535}
};

/* Rectangular outgoing scoring zone, box-center coordinates. */
static const double OUTGOING[4] = {0.160, 0.785, -0.590, -0.420};

static WbDeviceTag receiver = 0;
static WbDeviceTag emitter = 0;
static WbNodeRef mobile = 0;
static WbFieldRef mobile_translation = 0;
static BoxInfo boxes[BOX_COUNT];
static MachineBay machine_a_bays[MACHINE_BAY_COUNT];
static MachineBay machine_b_bays[MACHINE_BAY_COUNT];

static char order[BOX_COUNT + 1] = "RRGB";
static bool magnet_on = false;
static int attached_index = -1;
static int score = 0;
static bool sent_start_order = false;

static double normalize_angle(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

static double random_processing_time(void) {
  const double u = (double)rand() / (double)RAND_MAX;
  return PROCESSING_TIME_MIN + u * (PROCESSING_TIME_MAX - PROCESSING_TIME_MIN);
}

static const double *color_for_state(PartState state) {
  if (state == PART_RED)
    return RED_COLOR;
  if (state == PART_GREEN)
    return GREEN_COLOR;
  if (state == PART_BLUE)
    return BLUE_COLOR;
  return GRAY_COLOR;
}

static char letter_for_state(PartState state) {
  if (state == PART_RED)
    return 'R';
  if (state == PART_GREEN)
    return 'G';
  if (state == PART_BLUE)
    return 'B';
  return '?';
}

static PartState state_for_letter(char c) {
  if (c == 'R')
    return PART_RED;
  if (c == 'G')
    return PART_GREEN;
  return PART_BLUE;
}

static const char *state_name(PartState state) {
  if (state == PART_RED)
    return "red";
  if (state == PART_GREEN)
    return "green";
  if (state == PART_BLUE)
    return "blue";
  return "unknown";
}

static double distance_xy3(const double *a, const double *b) {
  const double dx = a[0] - b[0];
  const double dy = a[1] - b[1];
  return sqrt(dx * dx + dy * dy);
}

static bool in_zone_xy(const double *position, const double zone[4]) {
  const double x = position[0];
  const double y = position[1];
  return x >= zone[0] && x <= zone[1] && y >= zone[2] && y <= zone[3];
}

static MachineBay *machine_bays_for(char machine) {
  return (machine == 'A') ? machine_a_bays : machine_b_bays;
}

static MachineBay *bay_for_box_input(const BoxInfo *box) {
  if (!box->in_machine_input || box->input_bay < 0 || box->input_bay >= MACHINE_BAY_COUNT)
    return NULL;
  return &machine_bays_for(box->input_machine)[box->input_bay];
}

static int machine_ready_mask(char machine) {
  int mask = 0;
  for (int i = 0; i < BOX_COUNT; ++i) {
    if ((machine == 'A' && boxes[i].ready_a) || (machine == 'B' && boxes[i].ready_b))
      mask |= (1 << i);
  }
  return mask;
}

static void send_ready_message(char machine, int box_index, int bay_index, const double robot_pose[3]) {
  char msg[MSG_SIZE];
  snprintf(msg, sizeof(msg), "READY %c %d %d %.5f %.5f %.5f",
           machine, box_index, bay_index, robot_pose[0], robot_pose[1], robot_pose[2]);
  wb_emitter_send(emitter, msg, (int)strlen(msg));
}

static void send_clear_message(char machine, int box_index) {
  char msg[MSG_SIZE];
  snprintf(msg, sizeof(msg), "CLEAR %c %d", machine, box_index);
  wb_emitter_send(emitter, msg, (int)strlen(msg));
}

static void broadcast_ready_boxes(void) {
  for (int i = 0; i < BOX_COUNT; ++i) {
    if (boxes[i].ready_a && boxes[i].ready_a_bay >= 0 && boxes[i].ready_a_bay < MACHINE_BAY_COUNT) {
      MachineBay *bay = &machine_a_bays[boxes[i].ready_a_bay];
      send_ready_message('A', i, bay->bay_index, bay->output_robot_pose);
    }
    if (boxes[i].ready_b && boxes[i].ready_b_bay >= 0 && boxes[i].ready_b_bay < MACHINE_BAY_COUNT) {
      MachineBay *bay = &machine_b_bays[boxes[i].ready_b_bay];
      send_ready_message('B', i, bay->bay_index, bay->output_robot_pose);
    }
  }
}

static void set_box_color(BoxInfo *box, PartState state) {
  if (box->base_color)
    wb_supervisor_field_set_sf_color(box->base_color, color_for_state(state));
}

static void reset_box_physics(BoxInfo *box) {
  if (box->node)
    wb_supervisor_node_reset_physics(box->node);
}

static void set_box_pose(BoxInfo *box, double x, double y, double z, double yaw) {
  const double translation[3] = {x, y, z};
  const double rotation[4] = {0.0, 0.0, 1.0, yaw};
  wb_supervisor_field_set_sf_vec3f(box->translation, translation);
  wb_supervisor_field_set_sf_rotation(box->rotation, rotation);
  reset_box_physics(box);
}

static void get_mobile_pose(double translation_out[3], double *theta_out, double *fx_out, double *fy_out) {
  const double *t = wb_supervisor_field_get_sf_vec3f(mobile_translation);
  translation_out[0] = t[0];
  translation_out[1] = t[1];
  translation_out[2] = t[2];

  const double *m = wb_supervisor_node_get_orientation(mobile);
  const double theta = normalize_angle(atan2(m[3], m[0]));
  *theta_out = theta;
  *fx_out = cos(theta);
  *fy_out = sin(theta);
}

static void compute_magnet_point(double point_out[3]) {
  double t[3], theta, fx, fy;
  (void)theta;
  get_mobile_pose(t, &theta, &fx, &fy);
  point_out[0] = t[0] + fx * MAGNET_POINT_DISTANCE;
  point_out[1] = t[1] + fy * MAGNET_POINT_DISTANCE;
  point_out[2] = BOX_Z;
}

static void try_start_processing(MachineBay *bay) {
  if (!bay || bay->input_box < 0 || bay->output_box >= 0 || bay->processing)
    return;

  BoxInfo *box = &boxes[bay->input_box];
  const double now = wb_robot_get_time();
  const double delay = random_processing_time();

  bay->processing = true;
  box->processing_until = now + delay;
  box->target_state = (bay->machine == 'A') ? PART_GREEN : PART_BLUE;
  memcpy(box->target_pose, bay->output_box_pose, sizeof(box->target_pose));

  printf("%s started processing in Machine %c bay %d; delay %.2f s.\n",
         box->box_def, bay->machine, bay->bay_index, delay);
  fflush(stdout);
}

static bool accept_box_at_machine(int index, char machine, MachineBay *bay) {
  BoxInfo *box = &boxes[index];

  if (!bay)
    return false;

  if (bay->input_box >= 0) {
    printf("%s not accepted by Machine %c bay %d: input already occupied by %s.\n",
           box->box_def, machine, bay->bay_index, boxes[bay->input_box].box_def);
    fflush(stdout);
    return false;
  }

  if (machine == 'A') {
    if (box->scored_a)
      return false;
    box->scored_a = true;
    ++score;
    printf("%s accepted by Machine A bay %d. +1. red -> green.\n", box->box_def, bay->bay_index);
  } else if (machine == 'B') {
    if (box->scored_b)
      return false;
    box->scored_b = true;
    ++score;
    printf("%s accepted by Machine B bay %d. +1. green -> blue.\n", box->box_def, bay->bay_index);
  }

  bay->input_box = index;
  box->in_machine_input = true;
  box->input_machine = machine;
  box->input_bay = bay->bay_index;

  if (bay->output_box >= 0) {
    printf("Machine %c bay %d output occupied; %s waits in the input.\n",
           machine, bay->bay_index, box->box_def);
    fflush(stdout);
  }

  try_start_processing(bay);
  fflush(stdout);
  return true;
}

static MachineBay *find_input_bay(char machine, const double *pos) {
  MachineBay *bays = machine_bays_for(machine);
  for (int i = 0; i < MACHINE_BAY_COUNT; ++i) {
    if (in_zone_xy(pos, bays[i].input_zone))
      return &bays[i];
  }
  return NULL;
}

static void clear_ready_output_if_needed(int index) {
  BoxInfo *box = &boxes[index];

  if (box->ready_a && box->ready_a_bay >= 0 && box->ready_a_bay < MACHINE_BAY_COUNT) {
    MachineBay *bay = &machine_a_bays[box->ready_a_bay];
    if (bay->output_box == index)
      bay->output_box = -1;
    box->ready_a = false;
    box->ready_a_bay = -1;
    send_clear_message('A', index);
    printf("%s removed from Machine A output; bay is free.\n", box->box_def);
    try_start_processing(bay);
  }

  if (box->ready_b && box->ready_b_bay >= 0 && box->ready_b_bay < MACHINE_BAY_COUNT) {
    MachineBay *bay = &machine_b_bays[box->ready_b_bay];
    if (bay->output_box == index)
      bay->output_box = -1;
    box->ready_b = false;
    box->ready_b_bay = -1;
    send_clear_message('B', index);
    printf("%s removed from Machine B output; bay is free.\n", box->box_def);
    try_start_processing(bay);
  }
  fflush(stdout);
}

static void attach_nearest_box(void) {
  if (attached_index >= 0)
    return;

  double mp[3];
  compute_magnet_point(mp);

  int best = -1;
  double best_distance = ATTACH_DISTANCE;
  for (int i = 0; i < BOX_COUNT; ++i) {
    if (boxes[i].processing_until >= 0.0 || boxes[i].delivered || boxes[i].in_machine_input)
      continue;

    const double *pos = wb_supervisor_field_get_sf_vec3f(boxes[i].translation);
    const double d = distance_xy3(mp, pos);
    if (d < best_distance) {
      best = i;
      best_distance = d;
    }
  }

  if (best >= 0) {
    clear_ready_output_if_needed(best);
    attached_index = best;
    printf("Attached %s as %s.\n", boxes[best].box_def, state_name(boxes[best].state));
    fflush(stdout);
  }
}

static void update_attached_box(void) {
  if (attached_index < 0)
    return;

  double t[3], theta, fx, fy;
  get_mobile_pose(t, &theta, &fx, &fy);

  const double x = t[0] + fx * BOX_CENTER_DISTANCE_FROM_ROBOT;
  const double y = t[1] + fy * BOX_CENTER_DISTANCE_FROM_ROBOT;
  const double box_yaw = normalize_angle(theta - M_PI / 2.0);
  set_box_pose(&boxes[attached_index], x, y, BOX_Z, box_yaw);
}

static void evaluate_drop(int index) {
  BoxInfo *box = &boxes[index];
  const double *pos = wb_supervisor_field_get_sf_vec3f(box->translation);

  if (box->state == PART_RED) {
    MachineBay *bay = find_input_bay('A', pos);
    if (bay && accept_box_at_machine(index, 'A', bay))
      return;
  } else if (box->state == PART_GREEN) {
    MachineBay *bay = find_input_bay('B', pos);
    if (bay && accept_box_at_machine(index, 'B', bay))
      return;
  } else if (box->state == PART_BLUE && in_zone_xy(pos, OUTGOING) && !box->delivered) {
    box->delivered = true;
    ++score;
    printf("%s delivered to outgoing warehouse. +1.\n", box->box_def);
    fflush(stdout);
    return;
  }

  printf("%s dropped outside a valid destination for %s.\n", box->box_def, state_name(box->state));
  fflush(stdout);
}

static void finish_processing_if_needed(void) {
  const double now = wb_robot_get_time();
  for (int i = 0; i < BOX_COUNT; ++i) {
    BoxInfo *box = &boxes[i];
    if (box->processing_until < 0.0 || now < box->processing_until)
      continue;

    MachineBay *bay = bay_for_box_input(box);
    if (!bay)
      continue;

    box->state = box->target_state;
    set_box_color(box, box->state);
    set_box_pose(box, box->target_pose[0], box->target_pose[1], box->target_pose[2], box->target_pose[3]);
    box->processing_until = -1.0;
    box->in_machine_input = false;
    box->input_machine = '?';
    box->input_bay = -1;

    bay->input_box = -1;
    bay->output_box = i;
    bay->processing = false;

    if (bay->machine == 'A') {
      box->ready_a = true;
      box->ready_a_bay = bay->bay_index;
      send_ready_message('A', i, bay->bay_index, bay->output_robot_pose);
    } else if (bay->machine == 'B') {
      box->ready_b = true;
      box->ready_b_bay = bay->bay_index;
      send_ready_message('B', i, bay->bay_index, bay->output_robot_pose);
    }

    printf("%s ready as %s at Machine %c bay %d output.\n",
           box->box_def, state_name(box->state), bay->machine, bay->bay_index);
    fflush(stdout);
  }
}

static void handle_messages(void) {
  while (wb_receiver_get_queue_length(receiver) > 0) {
    const void *raw = wb_receiver_get_data(receiver);
    int size = wb_receiver_get_data_size(receiver);
    char msg[MSG_SIZE];

    if (size >= MSG_SIZE)
      size = MSG_SIZE - 1;
    memcpy(msg, raw, size);
    msg[size] = '\0';

    if (strcmp(msg, "MAGNET_ON") == 0) {
      magnet_on = true;
      attach_nearest_box();
    } else if (strcmp(msg, "MAGNET_OFF") == 0) {
      magnet_on = false;
      if (attached_index >= 0) {
        const int dropped = attached_index;
        attached_index = -1;
        evaluate_drop(dropped);
      }
    }

    wb_receiver_next_packet(receiver);
  }
}

static void send_start_and_order_once(void) {
  if (sent_start_order || wb_robot_get_time() < 0.5)
    return;

  wb_emitter_send(emitter, "START", 5);
  char msg[MSG_SIZE];
  snprintf(msg, sizeof(msg), "ORDER %s", order);
  wb_emitter_send(emitter, msg, (int)strlen(msg));
  sent_start_order = true;
}

static void broadcast_pose(void) {
  double t[3], theta, fx, fy;
  (void)fx;
  (void)fy;
  get_mobile_pose(t, &theta, &fx, &fy);

  char msg[MSG_SIZE];
  const char *attached = (attached_index >= 0) ? boxes[attached_index].box_def : "none";

  /* The last two fields are bitmasks: bit 0 means BOX_0 ready, bit 1 means
   * BOX_1 ready, and so on.  They reset automatically when the output part is
   * removed by the robot.
   */
  snprintf(msg, sizeof(msg), "POSE %.5f %.5f %.5f %d %s %d %d",
           t[0], t[1], theta, score, attached,
           machine_ready_mask('A'), machine_ready_mask('B'));
  wb_emitter_send(emitter, msg, (int)strlen(msg));
}

static void update_overlay(void) {
  const char *attached = (attached_index >= 0) ? boxes[attached_index].box_def : "none";

  char line0[180];
  char line1[180];
  char line2[180];

  const char *mode = (TASK_ORDER_MODE == ORDER_MODE_RANDOM) ? "random" : "manual";

  snprintf(line0, sizeof(line0), "ORDER %s   mode=%s   score=%d   magnet=%s",
           order, mode, score, magnet_on ? "ON" : "OFF");
  snprintf(line1, sizeof(line1), "attached=%s   boxes: 0:%c  1:%c  2:%c  3:%c",
           attached,
           letter_for_state(boxes[0].state), letter_for_state(boxes[1].state),
           letter_for_state(boxes[2].state), letter_for_state(boxes[3].state));
  snprintf(line2, sizeof(line2), "readyMask A=%d B=%d   bay values are box index; -1=empty\nA in/out: %d/%d %d/%d   B in/out: %d/%d %d/%d",
           machine_ready_mask('A'), machine_ready_mask('B'),
           machine_a_bays[0].input_box, machine_a_bays[0].output_box,
           machine_a_bays[1].input_box, machine_a_bays[1].output_box,
           machine_b_bays[0].input_box, machine_b_bays[0].output_box,
           machine_b_bays[1].input_box, machine_b_bays[1].output_box);

  /* Parameters are: id, text, x, y, size, color, transparency, font. */
  wb_supervisor_set_label(0, line0, 0.015, 0.015, 0.07, 0x000000, 0.0, "Verdana");
  wb_supervisor_set_label(1, line1, 0.015, 0.055, 0.07, 0x000000, 0.0, "Verdana");
  wb_supervisor_set_label(2, line2, 0.015, 0.095, 0.055, 0x000000, 0.0, "Verdana");
}

static bool valid_order_letter(char c) {
  return c == 'R' || c == 'G' || c == 'B';
}

static void make_random_order(char letters[BOX_COUNT]) {
  /* Default challenge set: two raw, one intermediate, one final part. */
  char default_letters[BOX_COUNT] = {'R', 'R', 'G', 'B'};
  memcpy(letters, default_letters, sizeof(default_letters));

  /* Fisher-Yates shuffle. */
  for (int i = BOX_COUNT - 1; i > 0; --i) {
    const int j = rand() % (i + 1);
    const char tmp = letters[i];
    letters[i] = letters[j];
    letters[j] = tmp;
  }
}

static void make_manual_order(char letters[BOX_COUNT]) {
  /* Teacher-defined mode: edit MANUAL_ORDER near the top of this file. */
  const char *manual = MANUAL_ORDER;

  for (int i = 0; i < BOX_COUNT; ++i) {
    char c = manual[i];

    /* Invalid or missing letters fall back to blue so the simulation still runs. */
    if (!valid_order_letter(c))
      c = 'B';

    letters[i] = c;
  }
}

static void initialize_machine_bays(void) {
  const MachineBay defaults_a[MACHINE_BAY_COUNT] = {
    {'A', 0, {-0.500, -0.380, -0.055,  0.055}, {-0.260,  0.000, BOX_Z, M_PI / 2.0}, {-0.155,  0.000, M_PI}, -1, -1, false},
    {'A', 1, {-0.500, -0.380, -0.205, -0.095}, {-0.260, -0.150, BOX_Z, M_PI / 2.0}, {-0.155, -0.150, M_PI}, -1, -1, false}
  };

  const MachineBay defaults_b[MACHINE_BAY_COUNT] = {
    {'B', 0, { 0.215,  0.325, -0.055,  0.055}, { 0.430, -0.010, BOX_Z, M_PI / 2.0}, { 0.535, -0.010, M_PI}, -1, -1, false},
    {'B', 1, { 0.215,  0.325,  0.095,  0.205}, { 0.430,  0.150, BOX_Z, M_PI / 2.0}, { 0.535,  0.150, M_PI}, -1, -1, false}
  };

  memcpy(machine_a_bays, defaults_a, sizeof(machine_a_bays));
  memcpy(machine_b_bays, defaults_b, sizeof(machine_b_bays));
}

static void initialize_order_and_boxes(void) {
  char letters[BOX_COUNT];

  initialize_machine_bays();

  if (TASK_ORDER_MODE == ORDER_MODE_MANUAL)
    make_manual_order(letters);
  else
    make_random_order(letters);

  score = 0;
  magnet_on = false;
  attached_index = -1;
  sent_start_order = false;

  for (int i = 0; i < BOX_COUNT; ++i) {
    order[i] = letters[i];

    BoxInfo *box = &boxes[i];

    /* Reset the box state at every simulation reset. */
    box->state = state_for_letter(letters[i]);
    box->processing_until = -1.0;
    box->target_state = box->state;
    memset(box->target_pose, 0, sizeof(box->target_pose));
    box->scored_a = false;
    box->scored_b = false;
    box->ready_a = false;
    box->ready_b = false;
    box->ready_a_bay = -1;
    box->ready_b_bay = -1;
    box->in_machine_input = false;
    box->input_machine = '?';
    box->input_bay = -1;
    box->delivered = false;

    /* Put the box back into its incoming warehouse slot. */
    set_box_color(box, box->state);
    set_box_pose(box, INCOMING_SLOTS[i][0], INCOMING_SLOTS[i][1], BOX_Z, 0.0);
  }

  order[BOX_COUNT] = '\0';

  printf("Warehouse order left-to-right: %s (%s mode)\n", order,
         (TASK_ORDER_MODE == ORDER_MODE_RANDOM) ? "random" : "manual");
  fflush(stdout);
}

static bool init_nodes_and_devices(void) {
  receiver = wb_robot_get_device("magnet_rx");
  emitter = wb_robot_get_device("task_tx");
  if (!receiver || !emitter) {
    fprintf(stderr, "Supervisor missing magnet_rx or task_tx devices.\n");
    return false;
  }
  wb_receiver_enable(receiver, TIME_STEP);

  mobile = wb_supervisor_node_get_from_def("MOBILE_ROBOT");
  if (!mobile) {
    fprintf(stderr, "MOBILE_ROBOT DEF not found.\n");
    return false;
  }
  mobile_translation = wb_supervisor_node_get_field(mobile, "translation");

  for (int i = 0; i < BOX_COUNT; ++i) {
    boxes[i].box_def = BOX_DEFS[i];
    boxes[i].shape_def = SHAPE_DEFS[i];
    boxes[i].node = wb_supervisor_node_get_from_def(BOX_DEFS[i]);
    boxes[i].shape = wb_supervisor_node_get_from_def(SHAPE_DEFS[i]);
    if (!boxes[i].node) {
      fprintf(stderr, "%s DEF not found.\n", BOX_DEFS[i]);
      return false;
    }
    boxes[i].translation = wb_supervisor_node_get_field(boxes[i].node, "translation");
    boxes[i].rotation = wb_supervisor_node_get_field(boxes[i].node, "rotation");
    boxes[i].base_color = 0;

    if (boxes[i].shape) {
      WbFieldRef appearance_field = wb_supervisor_node_get_field(boxes[i].shape, "appearance");
      WbNodeRef appearance = wb_supervisor_field_get_sf_node(appearance_field);
      if (appearance)
        boxes[i].base_color = wb_supervisor_node_get_field(appearance, "baseColor");
    }
  }

  return true;
}

int main(void) {
  wb_robot_init();
  srand((unsigned int)time(NULL));

  if (!init_nodes_and_devices()) {
    wb_robot_cleanup();
    return 1;
  }

  initialize_order_and_boxes();
  printf("C logistics supervisor started. No Python controller is used.\n");
  fflush(stdout);

  while (wb_robot_step(TIME_STEP) != -1) {
    send_start_and_order_once();
    broadcast_pose();
    broadcast_ready_boxes();
    handle_messages();

    if (magnet_on && attached_index < 0)
      attach_nearest_box();
    if (magnet_on && attached_index >= 0)
      update_attached_box();

    finish_processing_if_needed();
    update_overlay();
  }

  wb_robot_cleanup();
  return 0;
}
