#ifndef SUPPORT_H
#define SUPPORT_H

#include <math.h>
#include <vector>
#include <tuple>
#include <functional>

const double MPS_TO_MPH = 2.236936; // 1 meter/sec equals this much mile/hour

const double SPEED_LIMIT = 49.5; // mph

const int NUM_LANES = 3;
const double LANE_WIDTH = 4; // meter
// To consider obstacles that are in process of changing lanes, use buffer.
const double LANE_DETECTION_BUFFER = LANE_WIDTH / 8;

// Min `s` direction distance required as a precondition for lane change.
// For safety, buffer behind should be much larger than that ahead.
// But mathematically this works.
const double LANE_CHANGE_BUFFER_S = 3; // meter

// Lane change is considered complete when the d settles within this distance from the new lane center.
const double LANE_CHANGE_COMPLETION_MARGIN = LANE_WIDTH / 40; // 10 cm margin => 20cm width

// The min/max bounds of time-to-collision, inside which it is considered to be dangerous.
const double MIN_TIME_TO_COLLISION = 0; // sec
const double MAX_TIME_TO_COLLISION = 1; // sec
// DEFAULT_ACCEL == `0.5 / 2.236936`. Then the `0.5` has a unit of `mile/hour/sec`?
// I don't know if this calculation is meaningful. But it works.
const double DEFAULT_ACCEL = .224; // meter/sec/sec

const double PATH_INTERVAL = 0.02; // sec
const double OUTPUT_PATH_DURATION = 1; // sec
// We'll generate this many plus one path points.
const int NUM_OUTPUT_PATH_POINTS = ceil(OUTPUT_PATH_DURATION / PATH_INTERVAL);


struct Telemetry {
  double now_x;
  double now_y;
  double now_s;
  double now_d;
  double now_yaw;
  double now_speed;

  std::vector<double> future_path_x;
  std::vector<double> future_path_y;
  size_t future_path_size;
  double future_path_duration;

  double future_s;
  double future_d;
  double future_speed;

  std::vector<std::vector<double> > now_obstacles;
};

enum FSM_State {KEEP_LANE, PLAN_LANE_CHANGE, CHANGING_LANE};


/**
  * Returns (FSM state, target lane, target speed).
  */
std::tuple<FSM_State, int, double> iterate_fsm(
  const FSM_State fsm_state, const int target_lane, const double target_speed,
  const Telemetry & telemetry,
  const bool debug);

/**
  * Assigns to `next_path_x` and `next_path_y`.
  */
std::tuple<std::vector<double>, std::vector<double> > generate_path(
  const Telemetry & telemetry,
  const double end_path_speed, const int target_lane,
  const std::function<std::vector<double>(double, double)> & sd_to_xy);

#endif
