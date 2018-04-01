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


enum FSM_State {KEEP_LANE, PLAN_LANE_CHANGE, CHANGING_LANE};


double lane_index_to_d(int lane_index);

int lane_index_away(int from_lane_index, double to_d);

/**
  * Returns obstacle indexes of {
  *   lane 0 ahead, lane 1 ahead, lane 2 ahead,
  *   lane 0 behind, lane 1 behind, lane 2 behind
  * }
  *
  * For each of the above, if not found, use -1.
  */
std::vector<int> find_closest_obstacles(const std::vector<std::vector<double> > & obstacles, double ref_s);

double extract_obstacle_speed(const std::vector<double> & obstacle);

double extract_obstacle_position(const std::vector<double> & obstacle);

/**
  * Returns whether setting the given lane as the target is .... TODO
  * 1) safe to do, and
  * 2) advantageous compared to reference time-to-collision.
  *
  * Returns (target speed, time-to-collision).
  */
// std::tuple<double> is_lane_optimal(
std::tuple<double, double> is_lane_feasible(
    const int lane,
    const std::vector<std::vector<double> > & obstacles, const std::vector<int> & closest_obstacle_inds,
    const double end_path_s, const double end_path_speed,
    const double prev_path_duration,
    const bool debug);

/**
  * Returns (whether lane change is safe to execute, target lane, target speed).
  */
// std::tuple<bool, int, double> prepare_lane_change(

/**
  * Returns (target lane, target speed, time-to-collision).
  */
std::tuple<int, double, double> prepare_lane_change(
    const int target_lane, const double target_speed,
    const std::vector<std::vector<double> > & obstacles, const std::vector<int> & closest_obstacle_inds,
    const double end_path_s, const double end_path_speed,
    const double prev_path_duration,
    const bool debug);

/**
  * Evaluates obstacle ahead in the provided target lane.
  *
  * Based on the obstacle's position and speed at the present _and_ at the end of
  * the previous path, determins optimal target speed if we are to stay in the
  * same lane.
  *
  * In other words, this fn calculates the cost time-to-collision of keeping lane.
  *
  * Also calculates the target speed if we ar eto stay in the lane.
  *
  * Returns (target speed, time-to-collision).
  */
std::tuple<double, double> keep_lane(
    const int target_lane,
    const std::vector<std::vector<double> > & obstacles, const std::vector<int> & closest_obstacle_inds,
    const double car_s, const double car_speed,
    const double end_path_s, const double end_path_speed,
    const double prev_path_duration,
    const bool debug);

/**
  * Returns (FSM state, target lane, target speed).
  */
std::tuple<FSM_State, int, double> iterate_fsm(
  const FSM_State fsm_state, const int target_lane, const double target_speed,
  const std::vector<std::vector<double> > & obstacles,
  const double car_s, const double car_d, const double car_speed,
  const double end_path_s, const double end_path_d, const double end_path_speed,
  const size_t previous_path_size,
  const bool debug);

/**
  * Assigns to `next_path_x` and `next_path_y`.
  */
std::tuple<std::vector<double>, std::vector<double> > generate_path(
    const double car_x, const double car_y, const double car_yaw,
    const std::vector<double> & previous_path_x, const std::vector<double> & previous_path_y, const size_t previous_path_size,
    const double end_path_s, const double end_path_speed, const int target_lane,
    const std::function<std::vector<double>(double, double)> & sd_to_xy);

#endif
