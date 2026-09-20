#ifndef ROBOT_NAVIGATION_HPP
#define ROBOT_NAVIGATION_HPP

#include <array>
#include <string>

// Webots controller tick in milliseconds. Keep this aligned with receiver and
// connector enable periods so every control step sees fresh supervisor data.
constexpr int TIME_STEP = 32;

// Robot-center pose on the ENU floor plane: x/y in meters, theta in radians.
struct Pose2D {
  double x;
  double y;
  double theta;
};

class Navigation {
public:
  Navigation();
  ~Navigation();

  void init();
  bool step();
  void stop();
  void resetActions();

  bool poseValid() const;
  Pose2D pose() const;
  const std::string &lastOrder() const;
  int score() const;
  const std::string &attachedBox() const;
  double time() const;
  double commandedLinearSpeed() const;
  double commandedAngularSpeed() const;
  double leftWheelSpeedRadS() const;
  double rightWheelSpeedRadS() const;
  int machineAReadyMask() const;
  int machineBReadyMask() const;

  bool machineAReady(int boxIndex) const;
  bool machineBReady(int boxIndex) const;
  int machineAReadyBay(int boxIndex) const;
  int machineBReadyBay(int boxIndex) const;
  bool machineAReadyPose(int boxIndex, Pose2D &poseOut) const;
  bool machineBReadyPose(int boxIndex, Pose2D &poseOut) const;

  static double normalizeAngle(double angle);

  bool goTo(double x, double y);
  bool goThrough(double x, double y);
  bool goToPose(double x, double y, double theta);
  // Track the line through the goal in the given robot heading. Reverse keeps
  // that heading while backing out of a bay. Transit segments need not stop.
  bool goToLine(double x, double y, double theta, bool stopAtGoal = true, bool reverse = false);
  bool rotateTo(double theta);
  bool rotateClockwiseTo(double theta);
  bool wait(double seconds);
  bool backUp(double seconds);
  bool backTo(double x, double y);

  // Track a planned circle with position and tangent-heading feedback. The
  // start pose is the tangent entry, not the robot's drifting measured pose.
  bool goToArc(Pose2D start, double radiusM, double angleRad, bool clockwise, bool stopAtGoal = true);
  // Relative arcs use the measured entry pose; both helpers share goToArc.
  bool moveArc(double radiusM, double angleRad, bool clockwise);
  bool moveCircle(double radiusM, double angleRad, bool clockwise);

  void magnetPick();
  void magnetDrop();
  bool magnetIsOn() const;

private:
  bool driveToXY(double x, double y, bool stopAtGoal, bool throughWaypoint);
  void setWheelSpeeds(double linearMS, double angularRadS);
  void sendText(const std::string &message);
  void readTaskMessages();

  class Impl;
  Impl *impl_ = nullptr;
};

#endif
