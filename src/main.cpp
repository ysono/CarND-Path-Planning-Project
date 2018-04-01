#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <tuple>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;


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


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

double lane_index_to_d(int lane_index) {
  return lane_width / 2 + lane_width * lane_index;
}

int lane_index_away(int from_lane_index, double to_d) {
  double from_d = lane_index_to_d(from_lane_index);
  if (from_d == to_d) {
    return from_lane_index;
  } else if (from_d < to_d) {
    return min(num_lanes - 1, from_lane_index + 1);
  } else {
    return max(0, from_lane_index - 1);
  }
}

/**
  * Returns obstacle indexes of {
  *   lane 0 ahead, lane 1 ahead, lane 2 ahead,
  *   lane 0 behind, lane 1 behind, lane 2 behind
  * }
  *
  * For each of the above, if not found, use -1.
  */
vector<int> find_closest_obstacles(const vector<vector<double> > & obstacles, double ref_s) {

  vector<int> obstacle_inds(num_lanes * 2, -1);

  vector<double> closest_s_ahead(num_lanes, std::numeric_limits<double>::max());
  vector<double> closest_s_behind(num_lanes, - std::numeric_limits<double>::max());

  for (int obstacle_ind = 0; obstacle_ind < obstacles.size(); obstacle_ind++) {
    vector<double> obstacle = obstacles[obstacle_ind];
    double d = obstacle[6];
    double s = obstacle[5];

    vector<int> relevant_lanes;
    if (d < lane_width + lane_detection_buffer) {
      relevant_lanes.push_back(0);
    }
    if (lane_width - lane_detection_buffer < d && d < lane_width * 2 + lane_detection_buffer) {
      relevant_lanes.push_back(1);
    }
    if (lane_width * 2 - lane_detection_buffer < d) {
      relevant_lanes.push_back(2);
    }

    for(unsigned int i = 0; i < relevant_lanes.size(); i++) {
      int lane = relevant_lanes[i];

      if (s > ref_s && s < closest_s_ahead[lane]) {
        closest_s_ahead[lane] = s;
        obstacle_inds[lane] = obstacle_ind;
      } else if (s < ref_s && s > closest_s_behind[lane]) {
        closest_s_behind[lane] = s;
        obstacle_inds[lane + num_lanes] = obstacle_ind;
      }
    }
  }

  return obstacle_inds;
}

double extract_obstacle_speed(const vector<double> & obstacle) {
  double vx = obstacle[3];
  double vy = obstacle[4];
  return sqrt(pow(vx, 2) + pow(vy, 2));
}

double extract_obstacle_position(const vector<double> & obstacle) {
  return obstacle[5];
}

/**
  * Returns whether setting the given lane as the target is
  * 1) safe to do, and
  * 2) advantageous compared to reference time-to-collision.
  *
  * Returns (whether safe, target speed)
  */
std::tuple<bool, double> is_lane_optimal(
    const int lane,
    const vector<vector<double> > & obstacles, const vector<int> & closest_obstacle_inds,
    const double end_path_s, const double end_path_speed, const double prev_path_duration,
    const double ref_time_to_collision,
    const bool debug) {

  int obstacle_ind_ahead = closest_obstacle_inds[lane];
  int obstacle_ind_behind = closest_obstacle_inds[lane + num_lanes];

  if (obstacle_ind_ahead == -1 && obstacle_ind_behind == -1) {
    // No one ahead or behind in the adjacent lane.
    // Execute lane change. Go at speed limit.
    return std::make_tuple(true, speed_limit);
  }

  // Just extracting data. No decision to change FSM is made in this block.
  bool behind_is_ok = true;
  if (obstacle_ind_behind != -1) {
    vector<double> obstacle_behind = obstacles[obstacle_ind_behind];
    double obst_speed = extract_obstacle_speed(obstacle_behind);
    double obst_pos_endpath = extract_obstacle_position(obstacle_behind) + obst_speed * prev_path_duration;

    double dist_gap = end_path_s - obst_pos_endpath;
    double speed_gap = end_path_speed - obst_speed;
    double time_to_collision = -dist_gap / speed_gap;

    behind_is_ok =
      dist_gap > lane_change_buffer_s &&
      (time_to_collision < min_time_to_collision ||
        time_to_collision > max_time_to_collision);

    if (debug) {
      cout << "v dist_gap "
        << (dist_gap > lane_change_buffer_s ? "" : "! ")
        << dist_gap << endl;
      cout << "v time_to_collision "
        << (time_to_collision < min_time_to_collision ||
            time_to_collision > max_time_to_collision ? "" : "! ")
        << time_to_collision << endl;
    }
  }

  if (obstacle_ind_ahead == -1) {
    // No one ahead. By logic, there is someone behind.

    if (behind_is_ok) {
      // Execute lane change. Go at speed limit.
      return std::make_tuple(true, speed_limit);
    }

    // Evaluate other lanes.
    return std::make_tuple(false, -1);
  }

  // By logic, there is someone ahead.
  // Just extracting data. No decision to change FSM is made in this block.
  bool ahead_is_ok;
  double obst_ahead_speed;
  {
    // (Lines in this block can be reordered to be more performant.
    // But I prefer readability.)

    vector<double> obstacle_ahead = obstacles[obstacle_ind_ahead];
    double obst_speed = extract_obstacle_speed(obstacle_ahead);
    double obst_pos_endpath = extract_obstacle_position(obstacle_ahead) + obst_speed * prev_path_duration;

    double dist_gap = obst_pos_endpath - end_path_s;
    double speed_gap = obst_speed - end_path_speed;
    double time_to_collision = -dist_gap / speed_gap;

    ahead_is_ok = 
      dist_gap > lane_change_buffer_s &&
      time_to_collision > max_time_to_collision &&
      time_to_collision > ref_time_to_collision;

    obst_ahead_speed = obst_speed;

    if (debug) {
      cout << "^ dist_gap "
        << (dist_gap > lane_change_buffer_s ? "" : "! ")
        << dist_gap << endl;
      cout << "^ time_to_collision "
        << (time_to_collision < min_time_to_collision ||
            time_to_collision > max_time_to_collision ? "" : "! ")
        << time_to_collision << endl;
      cout << "^ improvement in time_to_collision "
        << (time_to_collision > ref_time_to_collision ? "" : "! ")
        << (time_to_collision - ref_time_to_collision) << endl;
    }
  }

  if (ahead_is_ok && behind_is_ok) {
    // Execute lane change. Go at speed of obstacle ahead.
    return std::make_tuple(true, obst_ahead_speed);
  }

  return std::make_tuple(false, -1);
}

/**
  * Returns (whether lane change is safe to execute, target lane, target speed)
  */
std::tuple<bool, int, double> prepare_lane_change(
    const int target_lane, const double target_speed,
    const vector<vector<double> > & obstacles, const vector<int> & closest_obstacle_inds,
    const double end_path_s, const double end_path_speed, const double prev_path_duration,
    const double time_to_collision_if_keep_lane,
    const bool debug) {

  vector<int> adjacent_lanes; // in the order of preference
  if (target_lane == 0 || target_lane == 2) {
    adjacent_lanes = {1};
  } else {
    adjacent_lanes = {0, 2};
  }

  for (unsigned int temp = 0; temp < adjacent_lanes.size(); temp++) {
    int adj_lane = adjacent_lanes[temp];
    if (debug) {
      cout << "adj_lane " << adj_lane << endl;
    }

    bool is_lane_change_ready;
    double adj_lane_speed;
    std::tie(is_lane_change_ready, adj_lane_speed) = is_lane_optimal(
      adj_lane,
      obstacles, closest_obstacle_inds,
      end_path_s, end_path_speed, prev_path_duration,
      time_to_collision_if_keep_lane,
      debug);

    if (is_lane_change_ready) {
      return std::make_tuple(is_lane_change_ready, adj_lane, adj_lane_speed);
    }

  }

  // Return the provided target lane and target speed unchanged.
  return std::make_tuple(false, target_lane, target_speed);
}

/**
  * Returns (target speed, time to collision).
  */
tuple<double, double> keep_lane(
    const int target_lane,
    const vector<vector<double> > & obstacles, const vector<int> & closest_obstacle_inds,
    const double car_s, const double car_speed,
    const double end_path_s, const double end_path_speed,
    const double prev_path_duration,
    const bool debug) {

  int obstacle_ind_ahead = closest_obstacle_inds[target_lane];
  
  if (obstacle_ind_ahead == -1) {
    return std::make_tuple(speed_limit, -1);
  }

  vector<double> obstacle_ahead = obstacles[obstacle_ind_ahead];
  double obst_pos_now = extract_obstacle_position(obstacle_ahead);
  double obst_speed = extract_obstacle_speed(obstacle_ahead);

  double dist_gap_now = obst_pos_now - car_s;
  double speed_gap_now = obst_speed - car_speed;
  double time_to_collision_now = -dist_gap_now / speed_gap_now;

  // We need to consider obstacles that cut in front of us at a distance between
  // current s and `end_path_s`.
  bool too_close_now =
    time_to_collision_now > min_time_to_collision &&
    time_to_collision_now < max_time_to_collision;

  if (debug) {
    cout << "^ time_to_collision_now "
      << (too_close_now ? "! " : "")
      << time_to_collision_now << endl;
    if (too_close_now) {
      // Keep the num of rows output by this fn the same.
      cout << "^ time_to_collision_endpath -----" << endl;
    }
  }

  if (too_close_now) {
    return std::make_tuple(obst_speed, time_to_collision_now);
  }

  double obst_pos_endpath = obst_pos_now + obst_speed * prev_path_duration;
  double dist_gap_endpath = obst_pos_endpath - end_path_s;
  double speed_gap_endpath = obst_speed - end_path_speed;
  double time_to_collision_endpath = -dist_gap_endpath / speed_gap_endpath;

  bool too_close_endpath =
    time_to_collision_endpath > min_time_to_collision &&
    time_to_collision_endpath < max_time_to_collision;

  if (debug) {
    cout << "^ time_to_collision_endpath "
      << (too_close_endpath ? "! " : "")
      << time_to_collision_endpath << endl;
  }

  if (too_close_endpath) {
    return std::make_tuple(obst_speed, time_to_collision_endpath);
  }

  return std::make_tuple(speed_limit, -1);
}


int main(int argc, char* argv[]) {
  bool debug = argc >= 2 && strcmp(argv[1], "--debug") == 0;

  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  Mode mode = KEEP_LANE;
  double target_speed = speed_limit;
  int target_lane = 1;
  double end_path_speed = 0; // Telemetry doesn't provide this, so remember.

  h.onMessage(
    [&mode, &target_speed, &target_lane,      // three states of FSM
      &end_path_speed,                        // non-provided telemetry value
      &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
      &debug]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {

    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
        	double car_x = j[1]["x"];
        	double car_y = j[1]["y"];
        	double car_s = j[1]["s"];
        	double car_d = j[1]["d"];
        	double car_yaw = j[1]["yaw"]; car_yaw = deg2rad(car_yaw); // deg
        	double car_speed = j[1]["speed"];

        	// Previous path data given to the Planner
        	auto previous_path_x = j[1]["previous_path_x"];
        	auto previous_path_y = j[1]["previous_path_y"];
          size_t previous_path_size = previous_path_x.size();

        	// Previous path's end s and d values 
        	double end_path_s = j[1]["end_path_s"];
        	double end_path_d = j[1]["end_path_d"];

        	// Sensor Fusion Data, a list of all other cars on the same side of the road.
        	vector<vector<double> > obstacles = j[1]["sensor_fusion"];

          ////// Begin finite state machine logic //////

          if (previous_path_size == 0) {
            end_path_s = car_s;
          }

          double prev_path_duration = path_interval * previous_path_size;

          // Use the self's current position as the reference when finding closest obstacles.
          vector<int> closest_obstacle_inds = find_closest_obstacles(obstacles, car_s);

          // For all modes.
          double time_to_collision;
          std::tie(target_speed, time_to_collision) = keep_lane(
            target_lane,
            obstacles, closest_obstacle_inds,
            car_s, car_speed,
            end_path_s, end_path_speed,
            prev_path_duration,
            debug);

          if (mode == KEEP_LANE) {
            bool recommend_lane_change = target_speed < speed_limit;
            if (recommend_lane_change) {
              mode = PLAN_LANE_CHANGE;
            }
          } else if (mode == CHANGING_LANE) {

            double d_error = fabs(lane_index_to_d(target_lane) - end_path_d);
            if (d_error < lane_change_completion_margin) {
              mode = KEEP_LANE;
            } else {

              // Use a non-optimal reference time-to-collision of `-1`.
              bool is_lane_change_safe;
              double _;
              std::tie(is_lane_change_safe, _) = is_lane_optimal(
                target_lane,
                obstacles, closest_obstacle_inds,
                end_path_s, end_path_speed, prev_path_duration,
                -1,
                debug);
              if (! is_lane_change_safe) {
                // I believe this happens when the obstacle in the lane to change
                // to behind the self either
                // - came from the same lane and accelerated
                // - came from another lane
                // Either way, the presence of this obstacle behind must not
                // have been accounted for when originally planning the current
                // lane change.

                // Pass the current position in d, rather than d at the pathend.
                target_lane = lane_index_away(target_lane, car_d);

                // Not assigning `target_speed`. Let the next iteration handle it.
              }
            }

          } // end `if mode == CHANGING_LANE`

          if (mode == PLAN_LANE_CHANGE) {
            // Time to collision is the cost function for choosing between
            // staying with PLAN_LANE_CHANGE and changing to CHANGING_LANE.

            bool is_lane_change_ready;
            std::tie(is_lane_change_ready, target_lane, target_speed) = prepare_lane_change(
              target_lane, target_speed,
              obstacles, closest_obstacle_inds,
              end_path_s, end_path_speed, prev_path_duration,
              time_to_collision,
              debug);

            if (is_lane_change_ready) {
              mode = CHANGING_LANE;
            }
          }

          // For all modes, adjust acceleration only.
          // We're using the same speed for all path points to be added.
          // Although it would be more effective to use different speeds for
          // points to be added, in practice it should make little difference
          // becuase we rarely have to generate more than 1 to 3 points.
          if (end_path_speed < target_speed) {
            end_path_speed += default_accel;
          } else {
            end_path_speed -= default_accel;
          }

          if (debug) {
            string mode_str;
            if (mode == KEEP_LANE) { mode_str = "KL "; }
            if (mode == PLAN_LANE_CHANGE) { mode_str = "PLC"; }
            if (mode == CHANGING_LANE) { mode_str = "CL "; }
            cout << "mode " << mode_str
              << " target_lane " << target_lane
              << " target_speed " << target_speed << endl;
          }

          ////// Begin path generation //////

          // Markers for spline. Spline will go through all points defined here.
          vector<double> markers_x, markers_y;

          double end_path_x, end_path_y, end_path_yaw;

          // add 2 previous points to markers
          if (previous_path_size < 2) {
            end_path_x = car_x;
            end_path_y = car_y;
            end_path_yaw = car_yaw;

            markers_x.push_back(car_x - cos(car_yaw));
            markers_y.push_back(car_y - sin(car_yaw));

            markers_x.push_back(end_path_x);
            markers_y.push_back(end_path_y);
          } else {
            end_path_x = previous_path_x[previous_path_size - 1];
            end_path_y = previous_path_y[previous_path_size - 1];
            double penultimate_x = previous_path_x[previous_path_size - 2];
            double penultimate_y = previous_path_y[previous_path_size - 2];
            end_path_yaw = atan2(end_path_y - penultimate_y, end_path_x - penultimate_x);

            markers_x.push_back(penultimate_x);
            markers_y.push_back(penultimate_y);

            markers_x.push_back(end_path_x);
            markers_y.push_back(end_path_y);
          }

          // Add a few more sparse markers in far distance
          // Pick the markers in s,d and convert them to x,y
          // 30 meters is far ahead enough for a smooth lane change
          // Add more points further away to make the tail end of the curve straight
          vector<double> future_s_offsets{30, 60, 90};
          for (unsigned int i = 0; i < future_s_offsets.size(); i++) {
            double future_s_offset = future_s_offsets[i];

            vector<double> marker_xy = getXY(
              end_path_s + future_s_offset,
              lane_index_to_d(target_lane),
              map_waypoints_s, map_waypoints_x, map_waypoints_y);

            markers_x.push_back(marker_xy[0]);
            markers_y.push_back(marker_xy[1]);
          }

          // Transform markers to coordinate system - which I'll call p,q - such that
          // origin is at previous path's end
          // and p is pointing to car's forward direction at previous path's end
          // Reuse the same vectors
          for (int i = 0; i < markers_x.size(); i++) {
            double translated_x = markers_x[i] - end_path_x;
            double translated_y = markers_y[i] - end_path_y;
            markers_x[i] = translated_x * cos(-end_path_yaw) - translated_y * sin(-end_path_yaw);
            markers_y[i] = translated_x * sin(-end_path_yaw) + translated_y * cos(-end_path_yaw);
          }

          // Spline in p,q
          tk::spline spliner_pq;
          spliner_pq.set_points(markers_x, markers_y);

          // Next path x,y. This is what we will output.
          // The vector length will be greater than `previous_path_size`.
        	vector<double> next_path_x(previous_path_size), next_path_y(previous_path_size);

          // Add back all previous path points
          std::copy(previous_path_x.begin(), previous_path_x.end(), next_path_x.begin());
          std::copy(previous_path_y.begin(), previous_path_y.end(), next_path_y.begin());

          // Add newly generated path points.
          // First determine at what interval.
          double p_interval;
          {
            // Pick an arbitrary p in the future. It should be both far enough
            // and close enough to linearize the curvature.
            // Because s and p coordinates are approximately equal in the short
            // distance, simply pick the closest future s offset.
            double target_p = future_s_offsets[0];
            double target_q = spliner_pq(target_p);
            double target_dist = sqrt(pow(target_p, 2) + pow(target_q, 2));
            double dist_per_interval = path_interval * end_path_speed / mps_to_mph; // meter
            double N = target_dist / dist_per_interval;
            p_interval = target_p / N;
          }

          // Add as many points as needed to refill the count of output path points.
          // Generate points in p,q; then transform to x,y
          for (int i = 0; i <= num_output_path_points - previous_path_size; i++) {
            
            // To avoid repeating end_path_x and end_path_y, must use `i+1`. Makes a big difference!
            double p = p_interval * (i + 1);
            double q = spliner_pq(p);

            double x = p * cos(end_path_yaw) - q * sin(end_path_yaw) + end_path_x;
            double y = p * sin(end_path_yaw) + q * cos(end_path_yaw) + end_path_y;

            next_path_x.push_back(x);
            next_path_y.push_back(y);
          }

          ////// End of path generation //////

          json msgJson;
        	msgJson["next_x"] = next_path_x;
        	msgJson["next_y"] = next_path_y;

        	auto msg = "42[\"control\","+ msgJson.dump()+"]";
        	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          //this_thread::sleep_for(chrono::milliseconds(1000));
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
