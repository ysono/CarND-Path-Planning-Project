#ifndef SUPPORT_H
#define SUPPORT_H

#include <math.h>
#include <iostream>
#include <vector>
#include <tuple>
#include <functional>

constexpr double MAX_DOUBLE = std::numeric_limits<double>::max();

constexpr double MPS_TO_MPH = 2.236936; // 1 meter/sec equals this much mile/hour

constexpr double SPEED_LIMIT = 49.5; // mph

constexpr int NUM_LANES = 3;
constexpr double LANE_WIDTH = 4; // meter
// Detection of obstacles in a given lane includes obstacles outside this lane
// by this much margin.
constexpr double LANE_DETECTION_MARGIN_D = LANE_WIDTH / 4;
// Lane change is considered complete when the d settles within this distance
// from the destination lane center.
// The whole bidirectional width of this zone is twice this margin.
constexpr double LANE_CHANGE_COMPLETION_MARGIN_D = LANE_WIDTH / 8;

constexpr double MAX_SAFE_POSITION_GAP_FOLLOWING = 2; // meter
constexpr double MAX_SAFE_POSITION_GAP_PLC = 4; // meter

constexpr double MIN_UNSAFE_TIME_TO_COLLISION = 0; // sec
constexpr double MAX_UNSAFE_TIME_TO_COLLISION_FOLLOWING = 1; // sec
constexpr double MAX_UNSAFE_TIME_TO_COLLISION_PLC = 2; // sec

// DEFAULT_ACCEL == `0.5 / 2.236936`. Then the `0.5` has a unit of `mile/hour/sec`?
// I don't know if this calculation is meaningful. But it works.
constexpr double DEFAULT_ACCEL = .224; // meter/sec/sec

constexpr double PATH_INTERVAL = 0.02; // sec
constexpr double OUTPUT_PATH_DURATION = 0.4; // sec
const int NUM_OUTPUT_PATH_POINTS = ceil(OUTPUT_PATH_DURATION / PATH_INTERVAL);

// Gap between markers to add beyond the existing path.
constexpr double PATHGEN_FUTURE_MARKERS_NUM = 3;
// 30 meters is far ahead enough for a smooth lane change at speed of 49.5 mph.
constexpr double PATHGEN_FUTURE_MARKERS_GAP = 30; // meter
// At low speeds, including when starting out, use the gap at 15mph.
constexpr double PATHGEN_FUTURE_MARKERS_GAP_MIN = PATHGEN_FUTURE_MARKERS_GAP / SPEED_LIMIT * 15;

extern bool PP_DEBUG;


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

enum FSM_State {KEEP_LANE, PLAN_LANE_CHANGE, INITIATE_LANE_CHANGE, ABORT_LANE_CHANGE};

struct FSM {
  FSM_State state;
  int target_lane;
  double target_speed;
};

inline std::ostream & operator<<(std::ostream & stream, FSM const & fsm) { 
  std::string state_str;
  if (fsm.state == KEEP_LANE) { state_str = "KL "; }
  if (fsm.state == PLAN_LANE_CHANGE) { state_str = "PLC"; }
  if (fsm.state == INITIATE_LANE_CHANGE) { state_str = "ILC"; }
  if (fsm.state == ABORT_LANE_CHANGE) { state_str = "ALC"; }

  stream << "fsm_state " << state_str
    << " target_lane " << fsm.target_lane
    << " target_speed " << fsm.target_speed << std::endl;
  return stream;
}

/**
  * Describes constraints the self vehicle would encounter if it is to follow some lane.
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
  double time_to_collision;
};

inline std::ostream & operator<<(std::ostream & stream, ObstacleRelationship const & rel) { 
  stream << "pos  " << rel.position_gap << std::endl;
  stream << "time " << rel.time_to_collision << std::endl;
  return stream;
}


/**
  * Returns next FSM state.
  */
FSM iterate_fsm(const FSM fsm, const Telemetry & telemetry);

/**
  * Returns (`next_path_x`, `next_path_y`).
  */
std::tuple<std::vector<double>, std::vector<double> > generate_path(
  const int target_lane, const double end_path_speed, const Telemetry & telemetry,
  const std::function<std::vector<double>(double, double)> & sd_to_xy);

#endif
