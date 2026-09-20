#ifndef WAREHOUSE_CLEARANCE_HPP
#define WAREHOUSE_CLEARANCE_HPP

#include "robot_navigation.hpp"
#include <cmath>

// Static collision boxes from logistics_pbl_enu.wbt, in floor coordinates.
// Keep these in sync when moving walls. The planner uses them to approve arc
// shortcuts; the original aisle route remains the fallback.
struct WarehouseWall {
  double x, y, halfX, halfY;
};

constexpr WarehouseWall WAREHOUSE_WALLS[] = {
  {0.0, 0.595, 0.841, 0.009}, {0.0, -0.595, 0.841, 0.009},
  {-0.841, 0.0, 0.009, 0.595}, {0.841, 0.0, 0.009, 0.595},
  {-0.770, 0.540, 0.010, 0.040}, {-0.620, 0.540, 0.010, 0.040},
  {-0.470, 0.540, 0.010, 0.040}, {-0.320, 0.540, 0.010, 0.040},
  {-0.170, 0.540, 0.010, 0.040}, {-0.470, 0.580, 0.310, 0.010},
  {0.170, -0.540, 0.010, 0.040}, {0.320, -0.540, 0.010, 0.040},
  {0.470, -0.540, 0.010, 0.040}, {0.620, -0.540, 0.010, 0.040},
  {0.770, -0.540, 0.010, 0.040}, {0.470, -0.580, 0.310, 0.010},
  {-0.3475, -0.225, 0.090, 0.010}, {-0.3475, -0.075, 0.090, 0.010},
  {-0.3475, 0.075, 0.090, 0.010}, {-0.3475, -0.075, 0.010, 0.150},
  {0.3475, -0.075, 0.090, 0.010}, {0.3475, 0.075, 0.090, 0.010},
  {0.3475, 0.225, 0.090, 0.010}, {0.3475, 0.075, 0.010, 0.150}
};

// Separating-axis test: the robot/box rectangle can rotate, the walls do not.
inline bool overlapsWall(Pose2D pose, double halfLength, double halfWidth, const WarehouseWall &wall) {
  const double c = std::cos(pose.theta), s = std::sin(pose.theta);
  const double dx = wall.x - pose.x, dy = wall.y - pose.y;
  return std::fabs(dx) <= wall.halfX + halfLength * std::fabs(c) + halfWidth * std::fabs(s) &&
         std::fabs(dy) <= wall.halfY + halfLength * std::fabs(s) + halfWidth * std::fabs(c) &&
         std::fabs(dx * c + dy * s) <= halfLength + wall.halfX * std::fabs(c) + wall.halfY * std::fabs(s) &&
         std::fabs(-dx * s + dy * c) <= halfWidth + wall.halfX * std::fabs(s) + wall.halfY * std::fabs(c);
}

inline bool warehousePoseClear(Pose2D pose, bool carrying, double margin = 0.015) {
  const Pose2D box = {pose.x + 0.1175 * std::cos(pose.theta),
                     pose.y + 0.1175 * std::sin(pose.theta), pose.theta};
  const Pose2D magnet = {pose.x + 0.085 * std::cos(pose.theta),
                        pose.y + 0.085 * std::sin(pose.theta), pose.theta};
  for (const auto &wall : WAREHOUSE_WALLS) {
    // Keep the narrow projecting magnet separate from the body and wheels.
    if (overlapsWall(pose, 0.075 + margin, 0.071 + margin, wall) ||
        overlapsWall(magnet, 0.006 + margin, 0.014 + margin, wall) ||
        (carrying && overlapsWall(box, 0.0275 + margin, 0.0425 + margin, wall)))
      return false;
  }
  return true;
}

#endif
