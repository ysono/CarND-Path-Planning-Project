#ifndef SUPPORT_H
#define SUPPORT_H

#include <math.h>
#include <vector>
#include <tuple>
#include <functional>

const double mps_to_mph = 2.236936; // 1 meter/sec equals this much mile/hour

const double speed_limit = 49.5; // mph

const int num_lanes = 3;
const double lane_width = 4; // meter
// To consider obstacles that are in process of changing lanes, use buffer.
const double lane_detection_buffer = lane_width / 8;

// Min front/back (s direction) distance required as a precondition for lane change.
// For safety, buffer behind should be much larger than that ahead.
// But mathematically this works. (And our car is an aggressive lane changer.)
const double lane_change_buffer_s = 3; // meter

// Lane change is considered complete when the d settles within this distance from target lane center.
const double lane_change_completion_margin = lane_width / 40; // 10 cm margin => 20cm width

// The min/max bound of time to collision, inside which it is considered to be dangerous.
const double min_time_to_collision = 0; // sec
const double max_time_to_collision = 1; // sec
// default_accel == `0.5 / 2.236936`. Then the `0.5` has a unit of `mile/hour/sec`?
// I don't know if this calculation is meaningful. But it works.
const double default_accel = .224; // meter/sec/sec

const double path_interval = 0.02; // sec
const double output_path_duration = 1; // sec
const int num_output_path_points = ceil(output_path_duration / path_interval);


enum Mode {KEEP_LANE, PLAN_LANE_CHANGE, CHANGING_LANE};


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
  * Returns whether setting the given lane as the target is
  * 1) safe to do, and
  * 2) advantageous compared to reference time-to-collision.
  *
  * Returns (whether safe, target speed)
  */
std::tuple<bool, double> is_lane_optimal(
    const int lane,
    const std::vector<std::vector<double> > & obstacles, const std::vector<int> & closest_obstacle_inds,
    const double end_path_s, const double end_path_speed, const double prev_path_duration,
    const double ref_time_to_collision,
    const bool debug);

/**
  * Returns (whether lane change is safe to execute, target lane, target speed)
  */
std::tuple<bool, int, double> prepare_lane_change(
    const int target_lane, const double target_speed,
    const std::vector<std::vector<double> > & obstacles, const std::vector<int> & closest_obstacle_inds,
    const double end_path_s, const double end_path_speed, const double prev_path_duration,
    const double time_to_collision_if_keep_lane,
    const bool debug);

/**
  * Returns (target speed, time to collision).
  */
std::tuple<double, double> keep_lane(
    const int target_lane,
    const std::vector<std::vector<double> > & obstacles, const std::vector<int> & closest_obstacle_inds,
    const double car_s, const double car_speed,
    const double end_path_s, const double end_path_speed,
    const double prev_path_duration,
    const bool debug);

/**
  * Assigns to `next_path_x` and `next_path_y`.
  */
std::tuple<std::vector<double>, std::vector<double> > generate_path(
    const double car_x, const double car_y, const double car_yaw,
    const std::vector<double> & previous_path_x, const std::vector<double> & previous_path_y, const int previous_path_size,
    const double end_path_s, const double end_path_speed, const int target_lane,
    const std::function<std::vector<double>(double, double)> & sd_to_xy);

#endif
