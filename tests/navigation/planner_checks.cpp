#include "warehouse_map.hpp"
#include "route_planner.hpp"
#include <cstdio>
#include <cstdlib>

static void require(bool condition, int line) {
  if (!condition) {
    std::fprintf(stderr, "Planner check failed at line %d\n", line);
    std::exit(1);
  }
}
#define assert(condition) require(condition, __LINE__)

static void check(const char *name, Pose2D start, const std::vector<Pose2D> &points,
                  bool docking, bool clockwise, bool carrying, int expectedSteps) {
  const auto old = planCornerRoute(start, points.data(), static_cast<int>(points.size()), docking, clockwise);
  const auto route = planRoute(start, points.data(), static_cast<int>(points.size()), docking, clockwise, carrying);
  std::printf("%s: %d -> %d steps", name, static_cast<int>(old.size()), static_cast<int>(route.size()));
  for (const auto &s : route) {
    if (s.motion == RouteMotion::Arc) {
      std::printf(" arc r=%.3f sweep=%.1f deg", s.radius, s.sweep * 180.0 / M_PI);
      const double signedRadius = s.clockwise ? -s.radius : s.radius;
      const double endHeading = s.start.theta + (s.clockwise ? -s.sweep : s.sweep);
      const double x = s.start.x + signedRadius * (std::sin(endHeading) - std::sin(s.start.theta));
      const double y = s.start.y - signedRadius * (std::cos(endHeading) - std::cos(s.start.theta));
      assert(std::hypot(x - s.goal.x, y - s.goal.y) < 1e-8);
      assert(std::fabs(Navigation::normalizeAngle(endHeading - s.goal.theta)) < 1e-8);
    }
  }
  std::printf("\n");
  std::fflush(stdout);
  if (expectedSteps > 0) {
    assert(static_cast<int>(route.size()) == expectedSteps);
    assert(route[1].motion == RouteMotion::Arc);
    assert(route[1].radius > 0.12);
    for (const auto &s : route)
      assert(routeStepClear(s, carrying, docking && s.motion == RouteMotion::Line ? 0.003 : 0.015));
    if (docking) {
      assert(route.back().motion == RouteMotion::Line);
      assert(std::fabs(std::hypot(route.back().goal.x - route.back().start.x,
                                 route.back().goal.y - route.back().start.y) - 0.10) < 1e-8);
      assert(route[route.size() - 2].motion == RouteMotion::Rotate);
    }
  }
}

int main() {
  check("incoming -> A0", MAP_IN_FRONT[0], {MAP_MACHINE_A_INPUT_CLEAR_BAY[0], MAP_MACHINE_A_INPUT_BAY[0]}, true, true, true, 4);
  for (int bay = 0; bay < 2; ++bay) {
    check(bay ? "A1 around" : "A0 around", MAP_MACHINE_A_INPUT_CLEAR_BAY[bay],
          {MAP_IN_FRONT[0], MAP_P4_TOP_CENTER, MAP_MACHINE_A_OUTPUT_APPROACH_BAY[bay]}, false, false, false, 3);
    check(bay ? "B1 around" : "B0 around", MAP_MACHINE_B_INPUT_CLEAR_BAY[bay],
          {MAP_P4V_NORTH_CENTER, MAP_P5_NORTH_EAST, MAP_MACHINE_B_OUTPUT_APPROACH_BAY[bay]}, false, false, false, 3);
    check(bay ? "B1 -> outgoing" : "B0 -> outgoing", MAP_MACHINE_B_OUTPUT_CLEAR_BAY[bay],
          {MAP_OUT_FRONT[3], MAP_OUT_FRONT[0], MAP_OUT_DROP[0]}, true, false, true, 4);
    check(bay ? "incoming -> B1" : "incoming -> B0", MAP_IN_FRONT[0],
          {bay ? MAP_P4V_NORTH_CENTER : MAP_P4_TOP_CENTER, MAP_MACHINE_B_INPUT_CLEAR_BAY[bay], MAP_MACHINE_B_INPUT_BAY[bay]},
          true, true, true, 4);
  }
  check("incoming -> outgoing", MAP_IN_FRONT[0],
        {MAP_P10_WEST_CENTER, MAP_P21_WEST_SOUTH, MAP_P22V_SOUTH_CENTER, MAP_P22_CENTER_SOUTH, MAP_OUT_FRONT[0], MAP_OUT_DROP[0]},
        true, true, true, 0);
  // The payload extends beyond the chassis; a bare-robot check would miss this.
  assert(warehousePoseClear({-0.695, 0.244, FACE_WEST}, false));
  assert(!warehousePoseClear({-0.695, 0.244, FACE_WEST}, true));
  // A geometrically valid direct path must still be rejected through a wall.
  assert(!routeStepClear({RouteMotion::Line, {-0.60, 0.075, 0}, {0.0, 0.075, 0}}, false));
  const Pose2D straight[] = {MAP_MACHINE_B_INPUT_BAY[0]};
  const auto transfer = planRoute(MAP_MACHINE_A_OUTPUT_CLEAR_BAY[0], straight, 1, true, false, true);
  for (const auto &s : transfer)
    assert(s.motion != RouteMotion::Arc);
  std::puts("Planner geometry, payload clearance, straight docking and fallback checks passed.");
}
