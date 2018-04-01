#ifndef SUPPORT_H
#define SUPPORT_H

#include <math.h>
#include <iostream>
#include <vector>
#include <tuple>
#include <functional>

const double MAX_DOUBLE = std::numeric_limits<double>::max();

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

struct Obstacle {
  double id;
  double now_x;
  double now_y;
  double now_speed;
  double now_s;
  double now_d;
};

enum FSM_State {KEEP_LANE, PLAN_LANE_CHANGE, CHANGING_LANE};

struct FSM {
  FSM_State state;
  int target_lane;
  double target_speed;
};

inline std::ostream & operator<<(std::ostream & stream, FSM const & fsm) { 
  std::string state_str;
  if (fsm.state == KEEP_LANE) { state_str = "KL "; }
  if (fsm.state == PLAN_LANE_CHANGE) { state_str = "PLC"; }
  if (fsm.state == CHANGING_LANE) { state_str = "CL "; }

  stream << "fsm_state " << state_str
    << " target_lane " << fsm.target_lane
    << " target_speed " << fsm.target_speed << std::endl;
  return stream;
}

/**
  * Describes constraints if the self vehicle is to follow some lane.
  *
  * `time_to_collision` should be used as the cost of such decision.
  * The larger the time, the better the cost.
  */
struct LaneConstraints {
  double target_speed;
  double time_to_collision;
};

inline std::ostream & operator<<(std::ostream & stream, LaneConstraints const & con) { 
  stream << "target_speed      " << con.target_speed << std::endl;
  stream << "time_to_collision " << con.time_to_collision << std::endl;
  return stream;
}

/**
  * Describes relationship between the self vehicle and an obstacle.
  */
struct ObstacleRelationship {
  double position_gap;
  bool is_position_gap_safe;
  double time_to_collision;
  bool is_time_to_collision_safe;
};

inline std::ostream & operator<<(std::ostream & stream, ObstacleRelationship const & rel) { 
  stream << "pos  " << (rel.is_position_gap_safe ? "" : "! ") << rel.position_gap << std::endl;
  stream << "time " << (rel.is_time_to_collision_safe ? "" : "! ") << rel.time_to_collision << std::endl;
  return stream;
}


/**
  * Returns (FSM state, target lane, target speed).
  */
FSM iterate_fsm(const FSM fsm, const Telemetry & telemetry, const bool debug);

/**
  * Assigns to `next_path_x` and `next_path_y`.
  */
std::tuple<std::vector<double>, std::vector<double> > generate_path(
  const int target_lane, const double end_path_speed, const Telemetry & telemetry,
  const std::function<std::vector<double>(double, double)> & sd_to_xy);

#endif
