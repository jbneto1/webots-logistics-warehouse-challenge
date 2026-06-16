/*
 * C++ logistics supervisor for Webots R2025a.
 *
 * This controller owns the challenge rules and broadcasts a small text
 * protocol that the C++, C, and Python student controllers can consume.
 * Box carrying is handled physically by Webots Connector nodes; the supervisor
 * only tracks logical attachment for scoring and machine bookkeeping.
 */

#include "../student_controller_cpp/warehouse_map.hpp"

#include <webots/Emitter.hpp>
#include <webots/Field.hpp>
#include <webots/Node.hpp>
#include <webots/Receiver.hpp>
#include <webots/Supervisor.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <random>
#include <string>

namespace {

constexpr int kTimeStep = 32;
constexpr int kBoxCount = 4;
constexpr int kMachineBayCount = 2;
constexpr int kMessageSize = 224;

// Order tuning. Use manual mode while debugging routes so every run follows the
// same box types; random mode is better for normal challenge attempts.
constexpr int kOrderModeRandom = 0;
constexpr int kOrderModeManual = 1;
constexpr int kTaskOrderMode = kOrderModeRandom;
constexpr const char *kManualOrder = "RRGB";

// Supervisor/gameplay tuning. These values define where the virtual magnet is,
// how close a box must be to attach, and how long machines take to process.
constexpr double kMagnetPointDistance = 0.0850;
constexpr double kAttachDistance = 0.055;
constexpr double kBoxZ = 0.0325;
constexpr double kProcessingTimeMin = 15.0;
constexpr double kProcessingTimeMax = 25.0;

// Visual-only waypoint circles imported from warehouse_map.hpp at startup.
constexpr double kMapMarkerZ = 0.008;
constexpr double kMapMarkerRadius = 0.020;
constexpr double kMapMarkerHeight = 0.004;

constexpr std::array<const char *, kBoxCount> kBoxDefs = {"BOX_0", "BOX_1", "BOX_2", "BOX_3"};
constexpr std::array<const char *, kBoxCount> kShapeDefs = {"BOX_0_SHAPE", "BOX_1_SHAPE", "BOX_2_SHAPE", "BOX_3_SHAPE"};

constexpr std::array<double, 3> kRedColor = {0.85, 0.05, 0.05};
constexpr std::array<double, 3> kGreenColor = {0.05, 0.75, 0.12};
constexpr std::array<double, 3> kBlueColor = {0.05, 0.20, 0.90};
constexpr std::array<double, 3> kGrayColor = {0.55, 0.55, 0.55};

constexpr std::array<std::array<double, 2>, kBoxCount> kIncomingSlots = {{
  {-0.695, 0.535},
  {-0.545, 0.535},
  {-0.400, 0.535},
  {-0.245, 0.535}
}};

enum class PartState {
  Red,
  Green,
  Blue
};

struct Zone {
  double xmin;
  double xmax;
  double ymin;
  double ymax;

  bool contains(const double *position) const {
    const double x = position[0];
    const double y = position[1];
    return x >= xmin && x <= xmax && y >= ymin && y <= ymax;
  }
};

constexpr Zone kOutgoingZone = {0.160, 0.785, -0.590, -0.420};

struct Pose4 {
  double x;
  double y;
  double z;
  double yaw;
};

struct RobotPickupPose {
  double x;
  double y;
  double theta;
};

struct MobilePose {
  std::array<double, 3> translation;
  double theta;
  double fx;
  double fy;
};

struct BoxInfo {
  std::string boxDef;
  std::string shapeDef;
  webots::Node *node = nullptr;
  webots::Node *shape = nullptr;
  webots::Field *translation = nullptr;
  webots::Field *rotation = nullptr;
  webots::Field *baseColor = nullptr;
  PartState state = PartState::Blue;
  double processingUntil = -1.0;
  PartState targetState = PartState::Blue;
  Pose4 targetPose = {0.0, 0.0, 0.0, 0.0};
  bool scoredA = false;
  bool scoredB = false;
  bool readyA = false;
  bool readyB = false;
  int readyABay = -1;
  int readyBBay = -1;
  bool inMachineInput = false;
  char inputMachine = '?';
  int inputBay = -1;
  bool delivered = false;
};

struct MachineBay {
  char machine;
  int bayIndex;
  Zone inputZone;
  Pose4 outputBoxPose;
  RobotPickupPose outputRobotPose;
  int inputBox = -1;
  int outputBox = -1;
  bool processing = false;
};

double normalizeAngle(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

const std::array<double, 3> &colorForState(PartState state) {
  switch (state) {
    case PartState::Red:
      return kRedColor;
    case PartState::Green:
      return kGreenColor;
    case PartState::Blue:
      return kBlueColor;
  }
  return kGrayColor;
}

char letterForState(PartState state) {
  switch (state) {
    case PartState::Red:
      return 'R';
    case PartState::Green:
      return 'G';
    case PartState::Blue:
      return 'B';
  }
  return '?';
}

PartState stateForLetter(char c) {
  if (c == 'R')
    return PartState::Red;
  if (c == 'G')
    return PartState::Green;
  return PartState::Blue;
}

const char *stateName(PartState state) {
  switch (state) {
    case PartState::Red:
      return "red";
    case PartState::Green:
      return "green";
    case PartState::Blue:
      return "blue";
  }
  return "unknown";
}

bool validOrderLetter(char c) {
  return c == 'R' || c == 'G' || c == 'B';
}

double distanceXY(const std::array<double, 3> &a, const double *b) {
  const double dx = a[0] - b[0];
  const double dy = a[1] - b[1];
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

class LogisticsSupervisor {
public:
  LogisticsSupervisor() : rng_(static_cast<unsigned int>(std::time(nullptr))) {}

  bool init() {
    receiver_ = robot_.getReceiver("magnet_rx");
    emitter_ = robot_.getEmitter("task_tx");
    if (!receiver_ || !emitter_) {
      std::fprintf(stderr, "Supervisor missing magnet_rx or task_tx devices.\n");
      return false;
    }
    receiver_->enable(kTimeStep);

    mobile_ = robot_.getFromDef("MOBILE_ROBOT");
    if (!mobile_) {
      std::fprintf(stderr, "MOBILE_ROBOT DEF not found.\n");
      return false;
    }
    mobileTranslation_ = mobile_->getField("translation");

    drawMapPointMarkers();

    for (int i = 0; i < kBoxCount; ++i) {
      BoxInfo &box = boxes_[i];
      box.boxDef = kBoxDefs[i];
      box.shapeDef = kShapeDefs[i];
      box.node = robot_.getFromDef(box.boxDef);
      box.shape = robot_.getFromDef(box.shapeDef);
      if (!box.node) {
        std::fprintf(stderr, "%s DEF not found.\n", box.boxDef.c_str());
        return false;
      }

      box.translation = box.node->getField("translation");
      box.rotation = box.node->getField("rotation");
      box.baseColor = nullptr;

      if (box.shape) {
        webots::Field *appearanceField = box.shape->getField("appearance");
        webots::Node *appearance = appearanceField ? appearanceField->getSFNode() : nullptr;
        if (appearance)
          box.baseColor = appearance->getField("baseColor");
      }
    }

    return true;
  }

  void run() {
    initializeOrderAndBoxes();
    std::printf("C++ logistics supervisor started.\n");
    std::fflush(stdout);

    while (robot_.step(kTimeStep) != -1) {
      sendStartAndOrderOnce();
      broadcastPose();
      broadcastReadyBoxes();
      handleMessages();

      if (magnetOn_ && attachedIndex_ < 0)
        attachNearestBox();
      finishProcessingIfNeeded();
      updateOverlay();
    }
  }

private:
  void drawMapPointMarkers() {
    if (robot_.getFromDef("MAP_POINT_MARKERS"))
      return;

    webots::Node *root = robot_.getRoot();
    webots::Field *children = root ? root->getField("children") : nullptr;
    if (!children) {
      std::fprintf(stderr, "Could not access root children field for map point markers.\n");
      return;
    }

    std::string markerGroup = "DEF MAP_POINT_MARKERS Group { children [\n";
    for (int i = 0; i < MAP_VISUAL_POINT_COUNT; ++i) {
      const MapPointMarker &point = MAP_VISUAL_POINTS[i];
      char node[512];
      std::snprintf(node, sizeof(node),
                    "  DEF MAP_POINT_%s Pose { translation %.5f %.5f %.5f children [ "
                    "Shape { appearance PBRAppearance { baseColor 0.06 0.32 0.95 transparency 0.18 "
                    "roughness 0.65 metalness 0 } geometry Cylinder { height %.5f radius %.5f subdivision 32 } } "
                    "] }\n",
                    point.defName, point.pose.x, point.pose.y, kMapMarkerZ,
                    kMapMarkerHeight, kMapMarkerRadius);
      markerGroup += node;
    }
    markerGroup += "] }";

    children->importMFNodeFromString(-1, markerGroup);
    std::printf("Drew %d map point circles from warehouse_map.hpp.\n", MAP_VISUAL_POINT_COUNT);
    std::fflush(stdout);
  }

  std::array<MachineBay, kMachineBayCount> &machineBaysFor(char machine) {
    return (machine == 'A') ? machineABays_ : machineBBays_;
  }

  MachineBay *bayForBoxInput(const BoxInfo &box) {
    if (!box.inMachineInput || box.inputBay < 0 || box.inputBay >= kMachineBayCount)
      return nullptr;
    return &machineBaysFor(box.inputMachine)[box.inputBay];
  }

  double randomProcessingTime() {
    std::uniform_real_distribution<double> distribution(kProcessingTimeMin, kProcessingTimeMax);
    return distribution(rng_);
  }

  int machineReadyMask(char machine) const {
    int mask = 0;
    for (int i = 0; i < kBoxCount; ++i) {
      if ((machine == 'A' && boxes_[i].readyA) || (machine == 'B' && boxes_[i].readyB))
        mask |= (1 << i);
    }
    return mask;
  }

  void sendText(const std::string &message) {
    if (emitter_)
      emitter_->send(message.c_str(), static_cast<int>(message.size()));
  }

  void sendReadyMessage(char machine, int boxIndex, int bayIndex, const RobotPickupPose &robotPose) {
    char msg[kMessageSize];
    std::snprintf(msg, sizeof(msg), "READY %c %d %d %.5f %.5f %.5f",
                  machine, boxIndex, bayIndex, robotPose.x, robotPose.y, robotPose.theta);
    sendText(msg);
  }

  void sendClearMessage(char machine, int boxIndex) {
    char msg[kMessageSize];
    std::snprintf(msg, sizeof(msg), "CLEAR %c %d", machine, boxIndex);
    sendText(msg);
  }

  void broadcastReadyBoxes() {
    for (int i = 0; i < kBoxCount; ++i) {
      if (boxes_[i].readyA && boxes_[i].readyABay >= 0 && boxes_[i].readyABay < kMachineBayCount) {
        MachineBay &bay = machineABays_[boxes_[i].readyABay];
        sendReadyMessage('A', i, bay.bayIndex, bay.outputRobotPose);
      }
      if (boxes_[i].readyB && boxes_[i].readyBBay >= 0 && boxes_[i].readyBBay < kMachineBayCount) {
        MachineBay &bay = machineBBays_[boxes_[i].readyBBay];
        sendReadyMessage('B', i, bay.bayIndex, bay.outputRobotPose);
      }
    }
  }

  void setBoxColor(BoxInfo &box, PartState state) {
    if (box.baseColor)
      box.baseColor->setSFColor(colorForState(state).data());
  }

  void resetBoxPhysics(BoxInfo &box) {
    if (box.node)
      box.node->resetPhysics();
  }

  void setBoxPose(BoxInfo &box, double x, double y, double z, double yaw) {
    const std::array<double, 3> translation = {x, y, z};
    const std::array<double, 4> rotation = {0.0, 0.0, 1.0, yaw};
    box.translation->setSFVec3f(translation.data());
    box.rotation->setSFRotation(rotation.data());
    resetBoxPhysics(box);
  }

  MobilePose getMobilePose() const {
    MobilePose pose;
    const double *translation = mobileTranslation_->getSFVec3f();
    std::copy(translation, translation + 3, pose.translation.begin());

    const double *matrix = mobile_->getOrientation();
    pose.theta = normalizeAngle(std::atan2(matrix[3], matrix[0]));
    pose.fx = std::cos(pose.theta);
    pose.fy = std::sin(pose.theta);
    return pose;
  }

  std::array<double, 3> computeMagnetPoint() const {
    const MobilePose pose = getMobilePose();
    return {
      pose.translation[0] + pose.fx * kMagnetPointDistance,
      pose.translation[1] + pose.fy * kMagnetPointDistance,
      kBoxZ
    };
  }

  void tryStartProcessing(MachineBay &bay) {
    if (bay.inputBox < 0 || bay.outputBox >= 0 || bay.processing)
      return;

    BoxInfo &box = boxes_[bay.inputBox];
    const double now = robot_.getTime();
    const double delay = randomProcessingTime();

    bay.processing = true;
    box.processingUntil = now + delay;
    box.targetState = (bay.machine == 'A') ? PartState::Green : PartState::Blue;
    box.targetPose = bay.outputBoxPose;

    std::printf("%s started processing in Machine %c bay %d; delay %.2f s.\n",
                box.boxDef.c_str(), bay.machine, bay.bayIndex, delay);
    std::fflush(stdout);
  }

  bool acceptBoxAtMachine(int index, char machine, MachineBay &bay) {
    BoxInfo &box = boxes_[index];

    if (bay.inputBox >= 0) {
      std::printf("%s not accepted by Machine %c bay %d: input already occupied by %s.\n",
                  box.boxDef.c_str(), machine, bay.bayIndex, boxes_[bay.inputBox].boxDef.c_str());
      std::fflush(stdout);
      return false;
    }

    if (machine == 'A') {
      if (box.scoredA)
        return false;
      box.scoredA = true;
      ++score_;
      std::printf("%s accepted by Machine A bay %d. +1. red -> green.\n", box.boxDef.c_str(), bay.bayIndex);
    } else if (machine == 'B') {
      if (box.scoredB)
        return false;
      box.scoredB = true;
      ++score_;
      std::printf("%s accepted by Machine B bay %d. +1. green -> blue.\n", box.boxDef.c_str(), bay.bayIndex);
    }

    bay.inputBox = index;
    box.inMachineInput = true;
    box.inputMachine = machine;
    box.inputBay = bay.bayIndex;

    if (bay.outputBox >= 0) {
      std::printf("Machine %c bay %d output occupied; %s waits in the input.\n",
                  machine, bay.bayIndex, box.boxDef.c_str());
      std::fflush(stdout);
    }

    tryStartProcessing(bay);
    std::fflush(stdout);
    return true;
  }

  MachineBay *findInputBay(char machine, const double *position) {
    auto &bays = machineBaysFor(machine);
    for (auto &bay : bays) {
      if (bay.inputZone.contains(position))
        return &bay;
    }
    return nullptr;
  }

  void clearReadyOutputIfNeeded(int index) {
    BoxInfo &box = boxes_[index];

    if (box.readyA && box.readyABay >= 0 && box.readyABay < kMachineBayCount) {
      MachineBay &bay = machineABays_[box.readyABay];
      if (bay.outputBox == index)
        bay.outputBox = -1;
      box.readyA = false;
      box.readyABay = -1;
      sendClearMessage('A', index);
      std::printf("%s removed from Machine A output; bay is free.\n", box.boxDef.c_str());
      tryStartProcessing(bay);
    }

    if (box.readyB && box.readyBBay >= 0 && box.readyBBay < kMachineBayCount) {
      MachineBay &bay = machineBBays_[box.readyBBay];
      if (bay.outputBox == index)
        bay.outputBox = -1;
      box.readyB = false;
      box.readyBBay = -1;
      sendClearMessage('B', index);
      std::printf("%s removed from Machine B output; bay is free.\n", box.boxDef.c_str());
      tryStartProcessing(bay);
    }
    std::fflush(stdout);
  }

  void attachNearestBox() {
    if (attachedIndex_ >= 0)
      return;

    const std::array<double, 3> magnetPoint = computeMagnetPoint();

    int best = -1;
    double bestDistance = kAttachDistance;
    for (int i = 0; i < kBoxCount; ++i) {
      if (boxes_[i].processingUntil >= 0.0 || boxes_[i].delivered || boxes_[i].inMachineInput)
        continue;

      const double *position = boxes_[i].translation->getSFVec3f();
      const double distance = distanceXY(magnetPoint, position);
      if (distance < bestDistance) {
        best = i;
        bestDistance = distance;
      }
    }

    if (best >= 0) {
      clearReadyOutputIfNeeded(best);
      attachedIndex_ = best;
      std::printf("Attached %s as %s.\n", boxes_[best].boxDef.c_str(), stateName(boxes_[best].state));
      std::fflush(stdout);
    }
  }

  void evaluateDrop(int index) {
    BoxInfo &box = boxes_[index];
    const double *position = box.translation->getSFVec3f();

    if (box.state == PartState::Red) {
      MachineBay *bay = findInputBay('A', position);
      if (bay && acceptBoxAtMachine(index, 'A', *bay))
        return;
    } else if (box.state == PartState::Green) {
      MachineBay *bay = findInputBay('B', position);
      if (bay && acceptBoxAtMachine(index, 'B', *bay))
        return;
    } else if (box.state == PartState::Blue && kOutgoingZone.contains(position) && !box.delivered) {
      box.delivered = true;
      ++score_;
      std::printf("%s delivered to outgoing warehouse. +1.\n", box.boxDef.c_str());
      std::fflush(stdout);
      return;
    }

    const MobilePose robotPose = getMobilePose();
    std::printf("%s dropped outside a valid destination for %s at x=%.3f y=%.3f; robot x=%.3f y=%.3f theta=%.3f.\n",
                box.boxDef.c_str(), stateName(box.state), position[0], position[1],
                robotPose.translation[0], robotPose.translation[1], robotPose.theta);
    std::fflush(stdout);
  }

  void finishProcessingIfNeeded() {
    const double now = robot_.getTime();
    for (int i = 0; i < kBoxCount; ++i) {
      BoxInfo &box = boxes_[i];
      if (box.processingUntil < 0.0 || now < box.processingUntil)
        continue;

      MachineBay *bay = bayForBoxInput(box);
      if (!bay)
        continue;

      box.state = box.targetState;
      setBoxColor(box, box.state);
      setBoxPose(box, box.targetPose.x, box.targetPose.y, box.targetPose.z, box.targetPose.yaw);
      box.processingUntil = -1.0;
      box.inMachineInput = false;
      box.inputMachine = '?';
      box.inputBay = -1;

      bay->inputBox = -1;
      bay->outputBox = i;
      bay->processing = false;

      if (bay->machine == 'A') {
        box.readyA = true;
        box.readyABay = bay->bayIndex;
        sendReadyMessage('A', i, bay->bayIndex, bay->outputRobotPose);
      } else if (bay->machine == 'B') {
        box.readyB = true;
        box.readyBBay = bay->bayIndex;
        sendReadyMessage('B', i, bay->bayIndex, bay->outputRobotPose);
      }

      std::printf("%s ready as %s at Machine %c bay %d output.\n",
                  box.boxDef.c_str(), stateName(box.state), bay->machine, bay->bayIndex);
      std::fflush(stdout);
    }
  }

  void handleMessages() {
    while (receiver_->getQueueLength() > 0) {
      const auto *raw = static_cast<const char *>(receiver_->getData());
      int size = receiver_->getDataSize();
      if (size < 0)
        size = 0;

      std::string msg(raw, raw + size);

      if (msg == "MAGNET_ON") {
        magnetOn_ = true;
        attachNearestBox();
      } else if (msg == "MAGNET_OFF") {
        magnetOn_ = false;
        if (attachedIndex_ >= 0) {
          const int dropped = attachedIndex_;
          attachedIndex_ = -1;
          evaluateDrop(dropped);
        }
      }

      receiver_->nextPacket();
    }
  }

  void sendStartAndOrderOnce() {
    if (sentStartOrder_ || robot_.getTime() < 0.5)
      return;

    sendText("START");
    sendText("ORDER " + order_);
    sentStartOrder_ = true;
  }

  void broadcastPose() {
    const MobilePose pose = getMobilePose();
    const std::string attached = (attachedIndex_ >= 0) ? boxes_[attachedIndex_].boxDef : "none";

    char msg[kMessageSize];
    std::snprintf(msg, sizeof(msg), "POSE %.5f %.5f %.5f %d %s %d %d",
                  pose.translation[0], pose.translation[1], pose.theta, score_, attached.c_str(),
                  machineReadyMask('A'), machineReadyMask('B'));
    sendText(msg);
  }

  void updateOverlay() {
    const std::string attached = (attachedIndex_ >= 0) ? boxes_[attachedIndex_].boxDef : "none";
    const char *mode = (kTaskOrderMode == kOrderModeRandom) ? "random" : "manual";

    char line0[180];
    char line1[180];
    char line2[180];

    std::snprintf(line0, sizeof(line0), "ORDER %s   mode=%s   score=%d   magnet=%s",
                  order_.c_str(), mode, score_, magnetOn_ ? "ON" : "OFF");
    std::snprintf(line1, sizeof(line1), "attached=%s   boxes: 0:%c  1:%c  2:%c  3:%c",
                  attached.c_str(),
                  letterForState(boxes_[0].state), letterForState(boxes_[1].state),
                  letterForState(boxes_[2].state), letterForState(boxes_[3].state));
    std::snprintf(line2, sizeof(line2),
                  "readyMask A=%d B=%d   bay values are box index; -1=empty\nA in/out: %d/%d %d/%d   B in/out: %d/%d %d/%d",
                  machineReadyMask('A'), machineReadyMask('B'),
                  machineABays_[0].inputBox, machineABays_[0].outputBox,
                  machineABays_[1].inputBox, machineABays_[1].outputBox,
                  machineBBays_[0].inputBox, machineBBays_[0].outputBox,
                  machineBBays_[1].inputBox, machineBBays_[1].outputBox);

    robot_.setLabel(0, line0, 0.015, 0.015, 0.07, 0x000000, 0.0, "Verdana");
    robot_.setLabel(1, line1, 0.015, 0.055, 0.07, 0x000000, 0.0, "Verdana");
    robot_.setLabel(2, line2, 0.015, 0.095, 0.055, 0x000000, 0.0, "Verdana");
  }

  std::array<char, kBoxCount> makeRandomOrder() {
    std::array<char, kBoxCount> letters = {'R', 'R', 'G', 'B'};
    std::shuffle(letters.begin(), letters.end(), rng_);
    return letters;
  }

  std::array<char, kBoxCount> makeManualOrder() const {
    std::array<char, kBoxCount> letters = {};
    for (int i = 0; i < kBoxCount; ++i) {
      char c = kManualOrder[i];
      if (!validOrderLetter(c))
        c = 'B';
      letters[i] = c;
    }
    return letters;
  }

  void initializeMachineBays() {
    machineABays_ = {{
      {'A', 0, {-0.515, -0.380, -0.105,  0.055}, {-0.260,  0.000, kBoxZ, M_PI / 2.0}, {-0.155,  0.000, M_PI}, -1, -1, false},
      {'A', 1, {-0.515, -0.380, -0.255, -0.095}, {-0.260, -0.150, kBoxZ, M_PI / 2.0}, {-0.155, -0.150, M_PI}, -1, -1, false}
    }};

    machineBBays_ = {{
      {'B', 0, { 0.200,  0.335, -0.105,  0.055}, { 0.430, -0.010, kBoxZ, M_PI / 2.0}, { 0.535, -0.010, M_PI}, -1, -1, false},
      {'B', 1, { 0.200,  0.335,  0.045,  0.205}, { 0.430,  0.150, kBoxZ, M_PI / 2.0}, { 0.535,  0.150, M_PI}, -1, -1, false}
    }};
  }

  void initializeOrderAndBoxes() {
    initializeMachineBays();

    const std::array<char, kBoxCount> letters =
      (kTaskOrderMode == kOrderModeManual) ? makeManualOrder() : makeRandomOrder();

    score_ = 0;
    magnetOn_ = false;
    attachedIndex_ = -1;
    sentStartOrder_ = false;
    order_.assign(letters.begin(), letters.end());

    for (int i = 0; i < kBoxCount; ++i) {
      BoxInfo &box = boxes_[i];

      box.state = stateForLetter(letters[i]);
      box.processingUntil = -1.0;
      box.targetState = box.state;
      box.targetPose = {0.0, 0.0, 0.0, 0.0};
      box.scoredA = false;
      box.scoredB = false;
      box.readyA = false;
      box.readyB = false;
      box.readyABay = -1;
      box.readyBBay = -1;
      box.inMachineInput = false;
      box.inputMachine = '?';
      box.inputBay = -1;
      box.delivered = false;

      setBoxColor(box, box.state);
      setBoxPose(box, kIncomingSlots[i][0], kIncomingSlots[i][1], kBoxZ, 0.0);
    }

    std::printf("Warehouse order left-to-right: %s (%s mode)\n", order_.c_str(),
                (kTaskOrderMode == kOrderModeRandom) ? "random" : "manual");
    std::fflush(stdout);
  }

  webots::Supervisor robot_;
  webots::Receiver *receiver_ = nullptr;
  webots::Emitter *emitter_ = nullptr;
  webots::Node *mobile_ = nullptr;
  webots::Field *mobileTranslation_ = nullptr;
  std::array<BoxInfo, kBoxCount> boxes_;
  std::array<MachineBay, kMachineBayCount> machineABays_;
  std::array<MachineBay, kMachineBayCount> machineBBays_;
  std::string order_ = "RRGB";
  bool magnetOn_ = false;
  int attachedIndex_ = -1;
  int score_ = 0;
  bool sentStartOrder_ = false;
  std::mt19937 rng_;
};

int main() {
  LogisticsSupervisor app;
  if (!app.init())
    return EXIT_FAILURE;
  app.run();
  return EXIT_SUCCESS;
}
