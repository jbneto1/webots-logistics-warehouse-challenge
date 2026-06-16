#include "robot_navigation.hpp"

#include "debug_config.hpp"

#include <webots/Connector.hpp>
#include <webots/Emitter.hpp>
#include <webots/Motor.hpp>
#include <webots/Receiver.hpp>
#include <webots/Robot.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

// Physical robot dimensions used to convert chassis speeds into wheel speeds.
// These should match the wheel radius, axle spacing, and motor limits in the
// Webots world; wrong values make every controller gain feel misleading.
constexpr double kWheelRadiusM = 0.022;
constexpr double kAxleLengthM = 0.128;
constexpr double kMaxWheelSpeedRadS = 18.5;

// Navigation tuning. If the robot slows down too much at intermediate route
// points, first check whether that point is being reached with goThrough()
// (uses kThrough*) or goToPose() (uses final slowdown and angle alignment).
constexpr double kPositionToleranceM = 0.015;      // Final stop radius; smaller is more precise but can creep.
constexpr double kBackPositionToleranceM = 0.055;  // Reverse-clear radius; larger exits pockets sooner.
constexpr double kThroughToleranceM = 0.100;       // Pass-through waypoint radius; larger reduces intermediate slowdowns.
constexpr double kAngleToleranceRad = 0.055;       // Final heading tolerance; smaller aligns longer.
constexpr double kFinalSlowdownRadiusM = 0.045;    // Final approach slowdown starts inside this distance.
constexpr double kFinalMinLinearMS = 0.035;        // Minimum forward speed during final approach.
constexpr double kFinalMaxLinearMS = 0.170;        // Maximum forward speed for stop-at-goal moves.
constexpr double kThroughMinLinearMS = 0.085;      // Minimum speed through non-stopping waypoints.
constexpr double kThroughMaxLinearMS = 0.225;      // Maximum speed through non-stopping waypoints.
constexpr double kArcLinearSpeedMS = 0.110;        // Constant open-loop arc speed.
constexpr double kMovingInnerWheelRatio = 0.25;    // Keep both wheels driving during translated turns.
constexpr double kTightTurnDistanceM = 0.110;      // Near service poses, pivot before the short final drive.
constexpr double kTightTurnHeadingErrorRad = 0.20;

double clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double limitAngularWhileMoving(double linearMS, double angularRadS) {
  if (std::fabs(linearMS) < 1e-6)
    return angularRadS;

  const double halfAxle = kAxleLengthM * 0.5;
  const double maxAngular = (1.0 - kMovingInnerWheelRatio) * std::fabs(linearMS) / halfAxle;
  return clamp(angularRadS, -maxAngular, maxAngular);
}

}  // namespace

class Navigation::Impl {
public:
  webots::Robot robot;
  webots::Motor *leftMotor = nullptr;
  webots::Motor *rightMotor = nullptr;
  webots::Emitter *magnetEmitter = nullptr;
  webots::Receiver *taskReceiver = nullptr;
  webots::Connector *electromagnetConnector = nullptr;

  Pose2D currentPose = {0.0, 0.0, 0.0};
  bool havePose = false;
  std::string lastOrder;
  int latestScore = 0;
  std::string attachedBox = "none";
  int machineAReadyMask = 0;
  int machineBReadyMask = 0;
  std::array<int, 8> machineAReadyBay;
  std::array<int, 8> machineBReadyBay;
  std::array<Pose2D, 8> machineAReadyPose;
  std::array<Pose2D, 8> machineBReadyPose;
  std::array<bool, 8> machineAHaveReadyPose;
  std::array<bool, 8> machineBHaveReadyPose;
  bool magnetOn = false;
  double commandedLinearMS = 0.0;
  double commandedAngularRadS = 0.0;
  double leftWheelRadS = 0.0;
  double rightWheelRadS = 0.0;
  double lastConnectorStatusTime = -1.0;
  bool pickupDockingActive = false;
  double pickupDockingStartTime = 0.0;

  bool waitActive = false;
  double waitStartTime = 0.0;

  bool backUpActive = false;
  double backUpStartTime = 0.0;

  bool arcActive = false;
  double arcRadius = 0.0;
  double arcTargetAngle = 0.0;
  bool arcClockwise = false;
  double arcPreviousTheta = 0.0;
  double arcProgress = 0.0;

  Impl() {
    machineAReadyBay.fill(-1);
    machineBReadyBay.fill(-1);
    machineAHaveReadyPose.fill(false);
    machineBHaveReadyPose.fill(false);
  }
};

Navigation::Navigation() : impl_(new Impl()) {}

Navigation::~Navigation() {
  delete impl_;
}

double Navigation::normalizeAngle(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

void Navigation::setWheelSpeeds(double linearMS, double angularRadS) {
  double left = (linearMS - angularRadS * kAxleLengthM * 0.5) / kWheelRadiusM;
  double right = (linearMS + angularRadS * kAxleLengthM * 0.5) / kWheelRadiusM;

  left = clamp(left, -kMaxWheelSpeedRadS, kMaxWheelSpeedRadS);
  right = clamp(right, -kMaxWheelSpeedRadS, kMaxWheelSpeedRadS);

  impl_->commandedLinearMS = linearMS;
  impl_->commandedAngularRadS = angularRadS;
  impl_->leftWheelRadS = left;
  impl_->rightWheelRadS = right;

  impl_->leftMotor->setVelocity(left);
  impl_->rightMotor->setVelocity(right);
}

void Navigation::stop() {
  setWheelSpeeds(0.0, 0.0);
}

void Navigation::resetActions() {
  impl_->waitActive = false;
  impl_->backUpActive = false;
  impl_->arcActive = false;
  impl_->pickupDockingActive = false;
}

void Navigation::sendText(const std::string &message) {
  if (impl_->magnetEmitter)
    impl_->magnetEmitter->send(message.c_str(), static_cast<int>(message.size()));
}

void Navigation::readTaskMessages() {
  while (impl_->taskReceiver->getQueueLength() > 0) {
    const auto *raw = static_cast<const char *>(impl_->taskReceiver->getData());
    int size = impl_->taskReceiver->getDataSize();
    if (size >= 127)
      size = 127;

    char msg[128];
    std::memcpy(msg, raw, size);
    msg[size] = '\0';

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    int score = 0;
    int readyA = 0;
    int readyB = 0;
    char attached[64] = "none";

    const int poseFields = std::sscanf(msg, "POSE %lf %lf %lf %d %63s %d %d",
                                       &x, &y, &theta, &score, attached, &readyA, &readyB);
    if (poseFields == 7 || poseFields == 5) {
      impl_->currentPose = {x, y, normalizeAngle(theta)};
      impl_->latestScore = score;
      impl_->attachedBox = attached;
      if (poseFields == 7) {
        impl_->machineAReadyMask = readyA;
        impl_->machineBReadyMask = readyB;
        for (int i = 0; i < 8; ++i) {
          if ((impl_->machineAReadyMask & (1 << i)) == 0) {
            impl_->machineAReadyBay[i] = -1;
            impl_->machineAHaveReadyPose[i] = false;
          }
          if ((impl_->machineBReadyMask & (1 << i)) == 0) {
            impl_->machineBReadyBay[i] = -1;
            impl_->machineBHaveReadyPose[i] = false;
          }
        }
      }
      impl_->havePose = true;
    } else if (std::strncmp(msg, "READY ", 6) == 0) {
      char machine = '?';
      int boxIndex = -1;
      int readyBay = -1;
      double rx = 0.0;
      double ry = 0.0;
      double rtheta = 0.0;
      const int fields = std::sscanf(msg, "READY %c %d %d %lf %lf %lf",
                                     &machine, &boxIndex, &readyBay, &rx, &ry, &rtheta);
      if (boxIndex >= 0 && boxIndex < 8) {
        if (machine == 'A') {
          impl_->machineAReadyMask |= (1 << boxIndex);
          if (fields == 6) {
            impl_->machineAReadyBay[boxIndex] = readyBay;
            impl_->machineAReadyPose[boxIndex] = {rx, ry, normalizeAngle(rtheta)};
            impl_->machineAHaveReadyPose[boxIndex] = true;
          }
        } else if (machine == 'B') {
          impl_->machineBReadyMask |= (1 << boxIndex);
          if (fields == 6) {
            impl_->machineBReadyBay[boxIndex] = readyBay;
            impl_->machineBReadyPose[boxIndex] = {rx, ry, normalizeAngle(rtheta)};
            impl_->machineBHaveReadyPose[boxIndex] = true;
          }
        }
      }
    } else if (std::strncmp(msg, "CLEAR ", 6) == 0) {
      char machine = '?';
      int boxIndex = -1;
      if (std::sscanf(msg, "CLEAR %c %d", &machine, &boxIndex) == 2 && boxIndex >= 0 && boxIndex < 8) {
        if (machine == 'A') {
          impl_->machineAReadyMask &= ~(1 << boxIndex);
          impl_->machineAReadyBay[boxIndex] = -1;
          impl_->machineAHaveReadyPose[boxIndex] = false;
        } else if (machine == 'B') {
          impl_->machineBReadyMask &= ~(1 << boxIndex);
          impl_->machineBReadyBay[boxIndex] = -1;
          impl_->machineBHaveReadyPose[boxIndex] = false;
        }
      }
    } else if (std::strncmp(msg, "ORDER ", 6) == 0) {
      impl_->lastOrder = msg + 6;
      if (debugEnabled(DEBUG_STATE))
        std::printf("Received ORDER %s\n", impl_->lastOrder.c_str());
    } else if (std::strcmp(msg, "START") == 0) {
      if (debugEnabled(DEBUG_STATE))
        std::printf("Received START\n");
    }

    impl_->taskReceiver->nextPacket();
  }
}

void Navigation::init() {
  impl_->leftMotor = impl_->robot.getMotor("left wheel motor");
  impl_->rightMotor = impl_->robot.getMotor("right wheel motor");
  impl_->magnetEmitter = impl_->robot.getEmitter("magnet_emitter");
  impl_->taskReceiver = impl_->robot.getReceiver("task_receiver");
  impl_->electromagnetConnector = impl_->robot.getConnector("electromagnet_connector");

  if (!impl_->leftMotor || !impl_->rightMotor || !impl_->magnetEmitter || !impl_->taskReceiver ||
      !impl_->electromagnetConnector) {
    std::fprintf(stderr, "Missing a required Webots device. Check names in logistics_pbl_enu.wbt.\n");
    std::exit(EXIT_FAILURE);
  }

  impl_->leftMotor->setPosition(INFINITY);
  impl_->rightMotor->setPosition(INFINITY);
  impl_->leftMotor->setVelocity(0.0);
  impl_->rightMotor->setVelocity(0.0);
  impl_->taskReceiver->enable(TIME_STEP);
  impl_->electromagnetConnector->enablePresence(TIME_STEP);

  if (debugEnabled(DEBUG_STATE))
    std::printf("C++ navigation API ready.\n");
}

bool Navigation::step() {
  if (impl_->robot.step(TIME_STEP) == -1)
    return false;
  readTaskMessages();
  return true;
}

bool Navigation::poseValid() const {
  return impl_->havePose;
}

Pose2D Navigation::pose() const {
  return impl_->currentPose;
}

const std::string &Navigation::lastOrder() const {
  return impl_->lastOrder;
}

int Navigation::score() const {
  return impl_->latestScore;
}

const std::string &Navigation::attachedBox() const {
  return impl_->attachedBox;
}

double Navigation::time() const {
  return impl_->robot.getTime();
}

double Navigation::commandedLinearSpeed() const {
  return impl_->commandedLinearMS;
}

double Navigation::commandedAngularSpeed() const {
  return impl_->commandedAngularRadS;
}

double Navigation::leftWheelSpeedRadS() const {
  return impl_->leftWheelRadS;
}

double Navigation::rightWheelSpeedRadS() const {
  return impl_->rightWheelRadS;
}

int Navigation::machineAReadyMask() const {
  return impl_->machineAReadyMask;
}

int Navigation::machineBReadyMask() const {
  return impl_->machineBReadyMask;
}

bool Navigation::machineAReady(int boxIndex) const {
  if (boxIndex < 0 || boxIndex >= 8)
    return false;
  return (impl_->machineAReadyMask & (1 << boxIndex)) != 0;
}

bool Navigation::machineBReady(int boxIndex) const {
  if (boxIndex < 0 || boxIndex >= 8)
    return false;
  return (impl_->machineBReadyMask & (1 << boxIndex)) != 0;
}

int Navigation::machineAReadyBay(int boxIndex) const {
  if (!machineAReady(boxIndex))
    return -1;
  return impl_->machineAReadyBay[boxIndex];
}

int Navigation::machineBReadyBay(int boxIndex) const {
  if (!machineBReady(boxIndex))
    return -1;
  return impl_->machineBReadyBay[boxIndex];
}

bool Navigation::machineAReadyPose(int boxIndex, Pose2D &poseOut) const {
  if (!machineAReady(boxIndex) || !impl_->machineAHaveReadyPose[boxIndex])
    return false;
  poseOut = impl_->machineAReadyPose[boxIndex];
  return true;
}

bool Navigation::machineBReadyPose(int boxIndex, Pose2D &poseOut) const {
  if (!machineBReady(boxIndex) || !impl_->machineBHaveReadyPose[boxIndex])
    return false;
  poseOut = impl_->machineBReadyPose[boxIndex];
  return true;
}

bool Navigation::rotateTo(double theta) {
  if (!impl_->havePose) {
    stop();
    return false;
  }

  const double error = normalizeAngle(theta - impl_->currentPose.theta);

  if (std::fabs(error) < kAngleToleranceRad) {
    stop();
    return true;
  }

  double angular = clamp(2.5 * error, -1.25, 1.25);
  if (std::fabs(angular) < 0.22)
    angular = (angular < 0.0) ? -0.22 : 0.22;

  setWheelSpeeds(0.0, angular);
  return false;
}

bool Navigation::rotateClockwiseTo(double theta) {
  if (!impl_->havePose) {
    stop();
    return false;
  }

  const double shortestError = normalizeAngle(theta - impl_->currentPose.theta);
  if (std::fabs(shortestError) < kAngleToleranceRad) {
    stop();
    return true;
  }

  double clockwiseRemaining = normalizeAngle(impl_->currentPose.theta - theta);
  if (clockwiseRemaining < 0.0)
    clockwiseRemaining += 2.0 * M_PI;

  double angular = -clamp(2.5 * clockwiseRemaining, 0.22, 1.25);
  setWheelSpeeds(0.0, angular);
  return false;
}

bool Navigation::driveToXY(double x, double y, bool stopAtGoal, bool throughWaypoint) {
  if (!impl_->havePose) {
    stop();
    return false;
  }

  const double dx = x - impl_->currentPose.x;
  const double dy = y - impl_->currentPose.y;
  const double distance = std::sqrt(dx * dx + dy * dy);
  const double tolerance = throughWaypoint ? kThroughToleranceM : kPositionToleranceM;

  if (distance < tolerance) {
    if (stopAtGoal)
      stop();
    return true;
  }

  const double targetHeading = std::atan2(dy, dx);
  const double headingError = normalizeAngle(targetHeading - impl_->currentPose.theta);

  if (!throughWaypoint && distance < kTightTurnDistanceM &&
      std::fabs(headingError) > kTightTurnHeadingErrorRad) {
    double angular = clamp(2.7 * headingError, -1.25, 1.25);
    if (std::fabs(angular) < 0.24)
      angular = (angular < 0.0) ? -0.24 : 0.24;
    setWheelSpeeds(0.0, angular);
    return false;
  }

  // Through waypoints keep a higher minimum speed and a wider tolerance. Final
  // goals slow down near the target so service poses remain accurate.
  const double minLinear = throughWaypoint ? kThroughMinLinearMS : kFinalMinLinearMS;
  const double maxLinear = throughWaypoint ? kThroughMaxLinearMS : kFinalMaxLinearMS;
  double linear = clamp(1.35 * distance, minLinear, maxLinear);

  if (!throughWaypoint && distance < kFinalSlowdownRadiusM)
    linear = clamp(0.95 * distance, 0.030, 0.115);

  // Large heading error means rotate in place; moderate error scales linear
  // speed down so the robot does not swing wide into shelves or machines.
  if (std::fabs(headingError) > 1.05)
    linear = 0.0;
  else
    linear *= clamp(1.0 - std::fabs(headingError) / 1.20, 0.35, 1.0);

  double angular = clamp(3.2 * headingError, -1.70, 1.70);
  angular = limitAngularWhileMoving(linear, angular);
  setWheelSpeeds(linear, angular);
  return false;
}

bool Navigation::goTo(double x, double y) {
  return driveToXY(x, y, true, false);
}

bool Navigation::goThrough(double x, double y) {
  return driveToXY(x, y, false, true);
}

bool Navigation::goToPose(double x, double y, double theta) {
  if (!goTo(x, y))
    return false;
  return rotateTo(theta);
}

bool Navigation::wait(double seconds) {
  if (!impl_->waitActive) {
    impl_->waitActive = true;
    impl_->waitStartTime = impl_->robot.getTime();
    stop();
  }

  if (impl_->robot.getTime() - impl_->waitStartTime >= seconds) {
    impl_->waitActive = false;
    return true;
  }

  stop();
  return false;
}

bool Navigation::backUp(double seconds) {
  if (!impl_->backUpActive) {
    impl_->backUpActive = true;
    impl_->backUpStartTime = impl_->robot.getTime();
  }

  if (impl_->robot.getTime() - impl_->backUpStartTime >= seconds) {
    impl_->backUpActive = false;
    stop();
    return true;
  }

  setWheelSpeeds(-0.120, 0.0);
  return false;
}

bool Navigation::backTo(double x, double y) {
  if (!impl_->havePose) {
    stop();
    return false;
  }

  const double dx = x - impl_->currentPose.x;
  const double dy = y - impl_->currentPose.y;
  const double distance = std::sqrt(dx * dx + dy * dy);

  if (distance < kBackPositionToleranceM) {
    stop();
    return true;
  }

  const double desiredFrontHeading = normalizeAngle(std::atan2(dy, dx) + M_PI);
  const double headingError = normalizeAngle(desiredFrontHeading - impl_->currentPose.theta);

  double linear = -clamp(1.85 * distance, 0.090, 0.220);
  if (distance < 0.090)
    linear = -clamp(1.55 * distance, 0.075, 0.140);

  if (std::fabs(headingError) > 1.05)
    linear = 0.0;
  else
    linear *= clamp(1.0 - std::fabs(headingError) / 1.20, 0.55, 1.0);

  double angular = clamp(3.4 * headingError, -1.75, 1.75);
  angular = limitAngularWhileMoving(linear, angular);
  setWheelSpeeds(linear, angular);
  return false;
}

bool Navigation::moveArc(double radiusM, double angleRad, bool clockwise) {
  if (!impl_->havePose) {
    stop();
    return false;
  }

  if (radiusM < 0.06)
    radiusM = 0.06;

  const double targetAngle = std::fabs(angleRad);

  if (!impl_->arcActive || std::fabs(impl_->arcRadius - radiusM) > 1e-6 ||
      std::fabs(impl_->arcTargetAngle - targetAngle) > 1e-6 || impl_->arcClockwise != clockwise) {
    impl_->arcActive = true;
    impl_->arcRadius = radiusM;
    impl_->arcTargetAngle = targetAngle;
    impl_->arcClockwise = clockwise;
    impl_->arcPreviousTheta = impl_->currentPose.theta;
    impl_->arcProgress = 0.0;
  }

  const double delta = normalizeAngle(impl_->currentPose.theta - impl_->arcPreviousTheta);
  impl_->arcProgress += std::fabs(delta);
  impl_->arcPreviousTheta = impl_->currentPose.theta;

  if (impl_->arcProgress >= impl_->arcTargetAngle) {
    impl_->arcActive = false;
    stop();
    return true;
  }

  double angular = kArcLinearSpeedMS / radiusM;
  if (clockwise)
    angular = -angular;

  setWheelSpeeds(kArcLinearSpeedMS, angular);
  return false;
}

bool Navigation::moveCircle(double radiusM, double angleRad, bool clockwise) {
  return moveArc(radiusM, angleRad, clockwise);
}

void Navigation::magnetPick() {
  if (impl_->magnetOn)
    return;

  const int presence = impl_->electromagnetConnector->getPresence();

  if (presence) {
    stop();
    impl_->electromagnetConnector->lock();
  } else {
    if (!impl_->pickupDockingActive) {
      impl_->pickupDockingActive = true;
      impl_->pickupDockingStartTime = impl_->robot.getTime();
    }

    const double elapsed = impl_->robot.getTime() - impl_->pickupDockingStartTime;
    const double phase = std::fmod(elapsed, 1.40);
    if (phase < 0.55)
      setWheelSpeeds(0.025, 0.0);
    else if (phase < 0.90)
      stop();
    else if (phase < 1.20)
      setWheelSpeeds(-0.018, 0.0);
    else
      stop();
  }

  if (impl_->electromagnetConnector->isLocked()) {
    stop();
    sendText("MAGNET_ON");
    impl_->magnetOn = true;
    if (debugEnabled(DEBUG_STATE))
      std::printf("Connector electromagnet locked\n");
    impl_->lastConnectorStatusTime = -1.0;
    impl_->pickupDockingActive = false;
  } else {
    const double now = impl_->robot.getTime();
    if (debugEnabled(DEBUG_DETAIL) &&
        (impl_->lastConnectorStatusTime < 0.0 || now - impl_->lastConnectorStatusTime > 0.75)) {
      std::printf("Waiting for connector lock: presence=%d pose=(%.3f, %.3f, %.3f)\n",
                  presence, impl_->currentPose.x, impl_->currentPose.y, impl_->currentPose.theta);
      impl_->lastConnectorStatusTime = now;
    }
  }
}

void Navigation::magnetDrop() {
  if (impl_->magnetOn) {
    impl_->electromagnetConnector->unlock();
    sendText("MAGNET_OFF");
    impl_->magnetOn = false;
    impl_->lastConnectorStatusTime = -1.0;
    impl_->pickupDockingActive = false;
    if (debugEnabled(DEBUG_STATE))
      std::printf("Connector electromagnet unlocked\n");
  }
}

bool Navigation::magnetIsOn() const {
  return impl_->magnetOn;
}
