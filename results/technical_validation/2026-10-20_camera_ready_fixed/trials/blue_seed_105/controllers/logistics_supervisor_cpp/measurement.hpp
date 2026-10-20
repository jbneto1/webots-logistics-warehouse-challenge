// Injected into temporary supervisor copies only. No gameplay decisions here.
#ifndef TECHNICAL_VALIDATION_MEASUREMENT_HPP
#define TECHNICAL_VALIDATION_MEASUREMENT_HPP

#include <webots/Field.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>

class ValidationMeasurement {
public:
  void begin(double now) {
    startSim_ = now;
    startWall_ = std::chrono::steady_clock::now();
    events_ = std::fopen("events.csv", "w");
    trace_ = std::fopen("trajectory.csv", "w");
    if (!events_ || !trace_) {
      std::fprintf(stderr, "Cannot create validation evidence.\n");
      std::exit(2);
    }
    std::fprintf(events_, "simulation_s,event,box,machine,bay,score,assigned_delay_s,wall_s\n");
    std::fprintf(trace_, "simulation_s,robot_x_m,robot_y_m,robot_yaw_rad,box_x_m,box_y_m,box_z_m,score,attached_index,box_0_delivered\n");
    event(now, "start", -1, '?', -1, 0);
  }

  void event(double now, const char *name, int box, char machine, int bay,
             int score, double delay = -1.0) {
    std::fprintf(events_, "%.9f,%s,%d,%c,%d,%d,%.9f,%.9f\n",
                 now - startSim_, name, box, machine, bay, score, delay, wall());
    std::fflush(events_);
  }

  void dropAttempt() { ++drops_; }

  void reject(double now, int box, int score) {
    ++invalid_;
    event(now, "invalid_drop", box, '?', -1, score);
  }

  void delivery(double now, int box, int score) {
    event(now, "delivery", box, '?', -1, score);
    if (box == 0 && deliverySim_ < 0.0) {
      deliverySim_ = now - startSim_;
      deliveryWall_ = wall();
      deliveryScore_ = score;
    }
  }

  bool step(webots::Supervisor &supervisor, webots::Node *robot,
            webots::Node *box, int score, bool delivered, int attached) {
    if (finalized_) return true;
    const double sim = supervisor.getTime() - startSim_;
    const double *r = robot->getPosition();
    const double *b = box->getPosition();
    const double *orientation = robot->getOrientation();
    const double yaw = std::atan2(orientation[3], orientation[0]);
    std::fprintf(trace_, "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%d,%d,%d\n",
                 sim, r[0], r[1], yaw, b[0], b[1], b[2], score, attached, delivered ? 1 : 0);
    const bool finished = robot->getField("customData")->getSFString() == "technical_validation_finished";
    if (!finished && sim < VALIDATION_TIMEOUT_S)
      return false;
    finalized_ = true;
    const bool inOutgoing = b[0] >= 0.160 && b[0] <= 0.785 && b[1] >= -0.590 && b[1] <= -0.420;
    const bool success = finished && delivered && deliverySim_ >= 0.0 &&
      score == VALIDATION_EXPECTED_SCORE && invalid_ == 0 && attached == -1 && inOutgoing;
    event(supervisor.getTime(), finished ? "controller_finished" : "simulation_timeout", 0, '?', -1, score);
    FILE *result = std::fopen("measurement.json", "w");
    if (!result) {
      supervisor.simulationQuit(2);
      return true;
    }
    std::fprintf(result,
      "{\n  \"success\": %s,\n  \"box_0_delivered\": %s,\n  \"box_0_in_outgoing_at_end\": %s,\n"
      "  \"controller_finished\": %s,\n  \"supervisor_score\": %d,\n  \"score_at_delivery\": %d,\n"
      "  \"expected_score\": %d,\n  \"invalid_drops\": %d,\n  \"evaluated_drops\": %d,\n"
      "  \"delivery_simulation_s\": %.9f,\n  \"delivery_wall_s\": %.9f,\n"
      "  \"delivery_real_time_factor\": %.9f,\n  \"end_simulation_s\": %.9f,\n  \"end_wall_s\": %.9f,\n"
      "  \"attached_index_at_end\": %d,\n  \"robot_final_xy_m\": [%.9f, %.9f],\n"
      "  \"box_final_xyz_m\": [%.9f, %.9f, %.9f]\n}\n",
      success ? "true" : "false", delivered ? "true" : "false", inOutgoing ? "true" : "false",
      finished ? "true" : "false", score, deliveryScore_, VALIDATION_EXPECTED_SCORE, invalid_, drops_,
      deliverySim_, deliveryWall_, deliveryWall_ > 0.0 ? deliverySim_ / deliveryWall_ : -1.0,
      sim, wall(), attached, r[0], r[1], b[0], b[1], b[2]);
    std::fclose(result);
    std::fclose(events_);
    std::fclose(trace_);
    events_ = nullptr;
    trace_ = nullptr;
    supervisor.simulationQuit(success ? 0 : 1);
    return true;
  }

private:
  bool finalized_ = false;
  double wall() const {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - startWall_).count();
  }
  std::chrono::steady_clock::time_point startWall_;
  FILE *events_ = nullptr;
  FILE *trace_ = nullptr;
  double startSim_ = 0.0;
  double deliverySim_ = -1.0;
  double deliveryWall_ = -1.0;
  int deliveryScore_ = -1;
  int invalid_ = 0;
  int drops_ = 0;
};

#endif
