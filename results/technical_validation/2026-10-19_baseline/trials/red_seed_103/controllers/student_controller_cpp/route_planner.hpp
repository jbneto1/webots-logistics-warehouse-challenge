#ifndef ROUTE_PLANNER_HPP
#define ROUTE_PLANNER_HPP

#include "robot_navigation.hpp"
#include "warehouse_clearance.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

enum class RouteMotion { Line, Arc, Rotate, RotateClockwise };

struct RouteStep {
  RouteMotion motion;
  Pose2D start;
  Pose2D goal;
  double radius = 0.0;
  double sweep = 0.0;
  bool clockwise = false;
  bool stopAtGoal = true;
};

// The map supplies obstacle-clearing aisle corners. Replace each corner with
// a tangent circular fillet, retaining a straight section between adjacent
// curves. This is a fixed warehouse route planner, not an obstacle detector:
// new aisle corners must leave room for the chassis AND the carried box.
inline std::vector<RouteStep> planCornerRoute(Pose2D start, const Pose2D waypoints[], int count,
                                            bool docking, bool clockwiseDeparture) {
  std::vector<RouteStep> result;
  if (count <= 0)
    return result;

  std::vector<Pose2D> points{start};
  for (int i = 0; i < count; ++i) {
    // A reverse-clear endpoint can differ from its map pose by a few mm. Do
    // not introduce a tiny return segment and a U-turn just to revisit it.
    if (std::hypot(waypoints[i].x - points.back().x, waypoints[i].y - points.back().y) > 0.020)
      points.push_back(waypoints[i]);
  }

  Pose2D cursor = start;
  auto rotate = [&](double heading, bool force = false) {
    if (!force && std::fabs(Navigation::normalizeAngle(heading - cursor.theta)) < 0.005)
      return;
    const Pose2D goal = {cursor.x, cursor.y, heading};
    const bool clockwise = result.empty() && clockwiseDeparture;
    result.push_back({clockwise ? RouteMotion::RotateClockwise : RouteMotion::Rotate, cursor, goal});
    cursor = goal;
  };
  auto line = [&](Pose2D goal, bool service = false) {
    if (std::hypot(goal.x - cursor.x, goal.y - cursor.y) < 1e-6)
      return;
    const double heading = service ? goal.theta : std::atan2(goal.y - cursor.y, goal.x - cursor.x);
    rotate(heading, service);  // Explicit alignment before the straight bay entry.
    goal.theta = heading;
    result.push_back({RouteMotion::Line, cursor, goal});
    cursor = goal;
  };

  for (size_t i = 1; i + 1 < points.size(); ++i) {
    const Pose2D a = points[i - 1], b = points[i], c = points[i + 1];
    const double incomingLength = std::hypot(b.x - a.x, b.y - a.y);
    const double outgoingLength = std::hypot(c.x - b.x, c.y - b.y);
    const double incomingHeading = std::atan2(b.y - a.y, b.x - a.x);
    const double outgoingHeading = std::atan2(c.y - b.y, c.x - b.x);
    const double turn = Navigation::normalizeAngle(outgoingHeading - incomingHeading);
    const double tangent = std::tan(std::fabs(turn) * 0.5);

    // Nearly straight nodes need no maneuver. A reversal or a short corner
    // gets a stopped alignment rather than an infeasibly tight moving turn.
    if (std::fabs(turn) < 0.03) {
      continue;
    }
    const double trim = std::min(0.120 * tangent, 0.40 * std::min(incomingLength, outgoingLength));
    const double radius = trim / tangent;
    if (std::fabs(turn) > 2.8 || radius < 0.060) {
      line(b);
      continue;
    }

    const Pose2D entry = {b.x - trim * std::cos(incomingHeading),
                          b.y - trim * std::sin(incomingHeading), incomingHeading};
    const Pose2D exit = {b.x + trim * std::cos(outgoingHeading),
                         b.y + trim * std::sin(outgoingHeading), outgoingHeading};
    line(entry);
    result.push_back({RouteMotion::Arc, entry, exit, radius, std::fabs(turn), turn < 0.0});
    cursor = exit;
  }
  line(waypoints[count - 1], docking);
  if (!docking)
    rotate(waypoints[count - 1].theta);

  for (size_t i = 0; i + 1 < result.size(); ++i) {
    const auto next = result[i + 1].motion;
    result[i].stopAtGoal = next == RouteMotion::Rotate || next == RouteMotion::RotateClockwise;
  }
  return result;
}

inline double routeRotation(Pose2D start, Pose2D goal, bool clockwise = false) {
  double angle = Navigation::normalizeAngle(goal.theta - start.theta);
  if (clockwise && angle > 0.0)
    angle -= 2.0 * std::acos(-1.0);
  return angle;
}

inline bool routeStepClear(const RouteStep &step, bool carrying, double margin = 0.015) {
  double angle = 0.0;
  double length = 0.0;
  if (step.motion == RouteMotion::Line)
    length = std::hypot(step.goal.x - step.start.x, step.goal.y - step.start.y);
  else if (step.motion == RouteMotion::Arc) {
    angle = step.clockwise ? -step.sweep : step.sweep;
    length = step.radius * step.sweep;
  } else
    angle = routeRotation(step.start, step.goal, step.motion == RouteMotion::RotateClockwise);
  const int samples = std::max(1, static_cast<int>(std::ceil(std::max(length / 0.006, std::fabs(angle) / 0.030))));
  for (int i = 0; i <= samples; ++i) {
    const double t = static_cast<double>(i) / samples;
    Pose2D pose = step.start;
    pose.theta += t * angle;
    if (step.motion == RouteMotion::Line) {
      pose.x += t * (step.goal.x - step.start.x);
      pose.y += t * (step.goal.y - step.start.y);
    } else if (step.motion == RouteMotion::Arc) {
      const double radius = step.clockwise ? -step.radius : step.radius;
      pose.x += radius * (std::sin(pose.theta) - std::sin(step.start.theta));
      pose.y -= radius * (std::cos(pose.theta) - std::cos(step.start.theta));
    }
    if (!warehousePoseClear(pose, carrying, margin))
      return false;
  }
  return true;
}

// A distance-equivalent score favors fewer stops and controller changes while
// accounting for extra pivoting. A gratuitous curve cannot beat a straight bay
// transfer; a continuous broad arc can replace several small corner fillets.
inline double routeCost(const std::vector<RouteStep> &steps) {
  double cost = 0.0;
  for (size_t i = 0; i < steps.size(); ++i) {
    const auto &step = steps[i];
    if (step.motion == RouteMotion::Arc)
      cost += step.radius * step.sweep + 0.025;
    else if (step.motion == RouteMotion::Line)
      cost += std::hypot(step.goal.x - step.start.x, step.goal.y - step.start.y) + 0.200;
    else {
      const double angle = routeRotation(step.start, step.goal, step.motion == RouteMotion::RotateClockwise);
      const bool afterArc = i > 0 && steps[i - 1].motion == RouteMotion::Arc;
      // Do more of the heading change while moving, leaving less pivoting at
      // the bay. Initial alignment happens in the already-cleared departure.
      cost += (afterArc ? 0.110 : 0.070) * std::fabs(angle);
      if (std::fabs(angle) > 0.055)
        cost += 0.060;
      // Prefer continuing the curve into the final alignment over undoing its
      // steering direction, when the clearance permits either solution.
      if (afterArc && std::fabs(angle) > 0.10 &&
          angle * (steps[i - 1].clockwise ? -1.0 : 1.0) < 0.0)
        cost += 0.060;
    }
    if (step.stopAtGoal && (step.motion == RouteMotion::Line || step.motion == RouteMotion::Arc))
      cost += 0.060;
  }
  return cost;
}

inline std::vector<RouteStep> planRoute(Pose2D start, const Pose2D waypoints[], int count,
                                      bool docking = false, bool clockwiseDeparture = false, bool carrying = true) {
  const auto fallback = planCornerRoute(start, waypoints, count, docking, clockwiseDeparture);
  auto best = fallback;
  // Preserve simple straight transfers and the initial warehouse approach.
  if (count <= 0 || std::none_of(best.begin(), best.end(), [](const RouteStep &s) { return s.motion == RouteMotion::Arc; }))
    return best;

  const Pose2D goal = waypoints[count - 1];
  Pose2D arrival = goal;
  if (docking) {
    // The last 10 cm stay straight and begin with a stopped bay alignment.
    // CLEAR poses remain the reverse targets, not constraints on arc size.
    arrival.x -= 0.100 * std::cos(goal.theta);
    arrival.y -= 0.100 * std::sin(goal.theta);
  }
  const double chord = std::hypot(arrival.x - start.x, arrival.y - start.y);
  if (chord < 0.060)
    return best;
  const double bearing = std::atan2(arrival.y - start.y, arrival.x - start.x);
  const double pi = std::acos(-1.0);
  const double originalCost = routeCost(best);
  double bestCost = originalCost;

  // Sweep both ways through broad circles joining the actual departure and
  // the safe arrival point. No line is needed to reach an artificial entry.
  for (int degrees = -180; degrees <= 180; degrees += 5) {
    if (std::abs(degrees) < 10)
      continue;
    const double angle = degrees * pi / 180.0;
    const double radius = chord / (2.0 * std::sin(std::fabs(angle) * 0.5));
    if (radius < 0.060)
      continue;
    const Pose2D entry = {start.x, start.y, Navigation::normalizeAngle(bearing - angle * 0.5)};
    const Pose2D exit = {arrival.x, arrival.y, Navigation::normalizeAngle(entry.theta + angle)};
    std::vector<RouteStep> candidate = {
      {clockwiseDeparture ? RouteMotion::RotateClockwise : RouteMotion::Rotate, start, entry},
      {RouteMotion::Arc, entry, exit, radius, std::fabs(angle), angle < 0.0},
      {RouteMotion::Rotate, exit, arrival}
    };
    if (docking)
      candidate.push_back({RouteMotion::Line, arrival, goal});
    const double cost = routeCost(candidate);
    if (cost >= bestCost - 1e-6)
      continue;
    bool clear = true;
    for (const auto &step : candidate) {
      // Bay endpoints deliberately approach the walls more closely than aisle
      // curves. Only the aligned, slow final line uses the docking allowance.
      const double margin = docking && step.motion == RouteMotion::Line ? 0.003 : 0.015;
      clear = clear && routeStepClear(step, carrying, margin);
    }
    if (!clear)
      continue;
    best = candidate;
    bestCost = cost;
  }
  return bestCost < originalCost - 0.010 ? best : fallback;
}

#endif
