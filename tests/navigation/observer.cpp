#include <webots/Field.hpp>
#include <webots/Node.hpp>
#include <webots/Receiver.hpp>
#include <webots/Supervisor.hpp>
#include <cstdio>
#include <cmath>
#include <string>
#include <vector>

struct Obstacle {
  std::string name;
  double center[3];
  double halfSize[3];
};

// A separate supervisor checks actual physics contacts, including the carried
// box. Score alone would miss the corner clipping this regression covers.
int main() {
  webots::Supervisor observer;
  auto *receiver = observer.getReceiver("task_rx");
  receiver->enable(32);
  auto *robot = observer.getFromDef("MOBILE_ROBOT");
  auto *box = observer.getFromDef("BOX_0");
  std::vector<Obstacle> obstacles;
  auto *children = observer.getRoot()->getField("children");
  for (int i = 0; i < children->getCount(); ++i) {
    auto *node = children->getMFNode(i);
    const auto def = node->getDef();
    if (node->getType() != webots::Node::SOLID || def == "SHOP_FLOOR" || def.rfind("BOX_", 0) == 0)
      continue;
    // Every static obstacle in this world has an axis-aligned Box collider.
    auto *collider = node->getField("boundingObject")->getSFNode();
    if (!collider || collider->getType() != webots::Node::BOX)
      return 2;
    const double *position = node->getPosition();
    const double *size = collider->getField("size")->getSFVec3f();
    Obstacle obstacle;
    obstacle.name = def;
    for (int axis = 0; axis < 3; ++axis) {
      obstacle.center[axis] = position[axis];
      obstacle.halfSize[axis] = size[axis] * 0.5;
    }
    obstacles.push_back(obstacle);
  }
  FILE *trace = std::fopen("trajectory.csv", "w");
  FILE *evidence = std::fopen("evidence.log", "w");
  if (!trace || !evidence)
    return 2;
  std::fprintf(trace, "time,x,y,theta,box_x,box_y,score,attached\n");
  int expected = 0, score = 0, contacts = 0;
  char attached[32] = "none";
  double successTime = -1.0;
  while (observer.step(32) != -1) {
    double x = 0, y = 0, theta = 0;
    while (receiver->getQueueLength()) {
      std::string message(static_cast<const char *>(receiver->getData()), receiver->getDataSize());
      if (message.rfind("ORDER ", 0) == 0)
        expected = message[6] == 'R' ? 3 : message[6] == 'G' ? 2 : 1;
      std::sscanf(message.c_str(), "POSE %lf %lf %lf %d %31s", &x, &y, &theta, &score, attached);
      receiver->nextPacket();
    }
    const double time = observer.getTime();
    for (auto *moving : {robot, box}) {
      if (moving == box && std::string(attached) != "BOX_0")
        continue;
      int count = 0;
      const auto *points = moving->getContactPoints(true, &count);
      for (int i = 0; i < count; ++i) {
        // node_id identifies the contacting part of 'moving', not the other
        // object. Match the world-space point to each obstacle's collider.
        for (const auto &obstacle : obstacles) {
          bool inside = true;
          for (int axis = 0; axis < 3; ++axis)
            inside = inside && std::fabs(points[i].point[axis] - obstacle.center[axis]) <= obstacle.halfSize[axis] + 0.001;
          if (inside && contacts++ < 10) {
            std::fprintf(evidence, "CONTACT t=%.3f moving=%s obstacle=%s at=(%.4f,%.4f,%.4f)\n",
                         time, moving->getDef().c_str(), obstacle.name.c_str(),
                         points[i].point[0], points[i].point[1], points[i].point[2]);
            std::fflush(evidence);
          }
        }
      }
    }
    const double *r = robot->getPosition(), *b = box->getPosition();
    std::fprintf(trace, "%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%d,%s\n", time, r[0], r[1], theta, b[0], b[1], score, attached);
    if (expected && score == expected && std::string(attached) == "none" && successTime < 0.0)
      successTime = time;
    if (contacts || (successTime >= 0.0 && time - successTime >= 8.0) || time >= 180.0 || (expected && score > expected)) {
      const bool passed = expected && score == expected && successTime >= 0.0 && contacts == 0 && r[1] > -0.265;
      std::fprintf(evidence, "%s score=%d/%d contacts=%d delivery_time=%.3f final=(%.4f,%.4f)\n",
                   passed ? "PASS" : "FAIL", score, expected, contacts, successTime, r[0], r[1]);
      std::fclose(trace);
      std::fclose(evidence);
      observer.simulationQuit(passed ? 0 : 1);
      return passed ? 0 : 1;
    }
  }
  std::fclose(trace);
  std::fclose(evidence);
  return 1;
}
