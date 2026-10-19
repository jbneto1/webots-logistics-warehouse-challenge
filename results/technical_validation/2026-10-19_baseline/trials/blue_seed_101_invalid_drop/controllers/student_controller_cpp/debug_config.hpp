#ifndef DEBUG_CONFIG_HPP
#define DEBUG_CONFIG_HPP

// Controller console verbosity. Tune this while diagnosing route behavior:
// DEBUG_OFF    = only critical errors from Webots/controller startup.
// DEBUG_STATE  = state transitions and one-line task milestones.
// DEBUG_DETAIL = periodic pose, speed, magnet, machine, and path-index telemetry.
enum DebugLevel {
  DEBUG_OFF = 0,
  DEBUG_STATE = 1,
  DEBUG_DETAIL = 2
};

constexpr DebugLevel DEBUG_LEVEL = DEBUG_DETAIL;
constexpr double DEBUG_DETAIL_PERIOD_S = 0.35;

constexpr bool debugEnabled(DebugLevel level) {
  return DEBUG_LEVEL >= level;
}

#endif
