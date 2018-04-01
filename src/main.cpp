// TODO car should change lane if it's not much faster but is much farther ahead.

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;


const double speed_limit = 49.5; // mph

const int num_lanes = 3;
const double lane_width = 4; // meter

const double max_time_to_collision = 1.0;

// const double keep_lane_obstacle_buffer = 30; // meter
// const double lane_change_obstacle_buffer_ahead = 10; // meter
// const double lane_change_obstacle_buffer_behind = 8; // meter

// This value == `0.5 / 2.236936`. I don't think this calculation is meaningful. But it works.
const double default_accel = .224; // meter/sec/sec

const double mps_to_mph = 2.236936; // 1 meter/sec equals this much mile/hour

const double path_interval = 0.02; // sec
const double output_path_duration = 1; // sec
const int num_output_path_points = ceil(output_path_duration / path_interval);


enum Mode {KEEP_LANE, PLAN_LANE_CHANGE};


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

/**
  * Returns obstacle indexes of {
  *   lane 0 ahead, lane 1 ahead, lane 2 ahead,
  *   lane 0 behind, lane 1 behind, lane 2 behind
  * }
  */
vector<int> find_closest_obstacles(const vector<vector<double> > & obstacles, double ref_s) {

  vector<int> obstacle_inds(num_lanes * 2, -1);

  vector<double> closest_s_ahead(num_lanes, std::numeric_limits<double>::max());
  vector<double> closest_s_behind(num_lanes, - std::numeric_limits<double>::max());

  double d_buffer = lane_width / 4;

  for (int obstacle_ind = 0; obstacle_ind < obstacles.size(); obstacle_ind++) {
    vector<double> obstacle = obstacles[obstacle_ind];
    double d = obstacle[6];
    double s = obstacle[5];

    vector<int> relevant_lanes;
    if (d < lane_width + d_buffer) {
      relevant_lanes.push_back(0);
    }
    if (lane_width - d_buffer < d && d < lane_width * 2 + d_buffer) {
      relevant_lanes.push_back(1);
    }
    if (lane_width * 2 - d_buffer < d) {
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
  * Assigns target_speed and mode.
  */
void keep_lane(
    const int target_lane,
    const vector<vector<double> > obstacles, const vector<int> closest_obstacle_inds,
    const double car_s, const double car_speed,
    const double end_path_s, const double end_path_speed,
    const double prev_path_duration,
    double & target_speed, Mode & mode,
    const bool debug) {

  int obstacle_ind_ahead = closest_obstacle_inds[target_lane];
  
  if (obstacle_ind_ahead == -1) {
    target_speed = speed_limit;
    mode = KEEP_LANE;
    return;
  }

  vector<double> obstacle_ahead = obstacles[obstacle_ind_ahead];
  double obst_pos_now = extract_obstacle_position(obstacle_ahead);
  double obst_speed = extract_obstacle_speed(obstacle_ahead);

  double dist_gap_now = obst_pos_now - car_s;
  double speed_gap_now = obst_speed - car_speed;
  double time_to_collision_now = -dist_gap_now / speed_gap_now;

  bool too_close_now = /*time_to_collision_now > 0 && */time_to_collision_now < max_time_to_collision;

  if (debug) {
    cout << "^ time_to_collision_now " << (too_close_now ? "! " : "") << time_to_collision_now << endl;
    if (too_close_now) {
      cout << "^ time_to_collision_endpath -----" << endl;
    }
  }

  if (too_close_now) {
    target_speed = obst_speed;
    // mode = PLAN_LANE_CHANGE;
    return;
  }

  double obst_pos_endpath = obst_pos_now + obst_speed * prev_path_duration;
  double dist_gap_endpath = obst_pos_endpath - end_path_s;
  double speed_gap_endpath = obst_speed - end_path_speed;
  double time_to_collision_endpath = -dist_gap_endpath / speed_gap_endpath;

  bool too_close_endpath = /*time_to_collision_now > 0 &&*/ time_to_collision_endpath < max_time_to_collision;

  if (debug) {
    cout << "^ time_to_collision_endpath " << (too_close_endpath ? "! " : "") << time_to_collision_endpath << endl;
  }

  if (too_close_endpath) {
    target_speed = obst_speed;
    // mode = PLAN_LANE_CHANGE;
    return;
  }

  target_speed = speed_limit;
  mode = KEEP_LANE;
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
        	// double car_d = j[1]["d"]; // not used
        	double car_yaw = j[1]["yaw"]; car_yaw = deg2rad(car_yaw); // deg
        	double car_speed = j[1]["speed"];

        	// Previous path data given to the Planner
        	auto previous_path_x = j[1]["previous_path_x"];
        	auto previous_path_y = j[1]["previous_path_y"];
          size_t previous_path_size = previous_path_x.size();

        	// Previous path's end s and d values 
        	double end_path_s = j[1]["end_path_s"];
        	// double end_path_d = j[1]["end_path_d"]; // not used

        	// Sensor Fusion Data, a list of all other cars on the same side of the road.
        	vector<vector<double> > obstacles = j[1]["sensor_fusion"];

          ////// Begin finite state machine logic //////

          if (previous_path_size == 0) {
            end_path_s = car_s;
          }

          double prev_path_duration = path_interval * previous_path_size;

          // Use the self's current position as the reference when finding closest obstacles.
          vector<int> closest_obstacle_inds = find_closest_obstacles(obstacles, car_s);

          // This `mode` block can assign `target_speed`, `target_lane` and `mode`.
          if (mode == PLAN_LANE_CHANGE) {
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

              int obstacle_ind_ahead = closest_obstacle_inds[adj_lane];
              int obstacle_ind_behind = closest_obstacle_inds[adj_lane + num_lanes];

              // No one ahead or behind in the adjacent lane.
              // Execute lane change. Go at speed limit.
              if (obstacle_ind_ahead == -1 && obstacle_ind_behind == -1) {
                target_speed = speed_limit;
                target_lane = adj_lane;
                mode = KEEP_LANE;

                // Don't evaluate other lanes.
                break;
              }

              // Just extracting data. No FSM change is made in this `if` block.
              bool behind_is_ok = true;
              if (obstacle_ind_behind != -1) {
                vector<double> obstacle_behind = obstacles[obstacle_ind_behind];
                double obst_speed = extract_obstacle_speed(obstacle_behind);
                double obst_pos_endpath = extract_obstacle_position(obstacle_behind) + obst_speed * prev_path_duration;

                double speed_gap = target_speed - obst_speed;
                double dist_gap = end_path_s - obst_pos_endpath;
                double time_to_collision = dist_gap / speed_gap;

                behind_is_ok = time_to_collision > 1.0;

                // bool dist_gap_ok = dist_gap > lane_change_obstacle_buffer_behind;
                // bool speed_gap_ok = speed_gap > 0;

                // // If there is enough distance gap, disregard speed gap.
                // behind_is_ok = dist_gap_ok || speed_gap_ok;

                if (debug) {
                  // cout << "v speed gap " << (speed_gap_ok ? "" : "! ") << speed_gap << endl;
                  // cout << "v dist gap " << (dist_gap_ok ? "" : "! ") << dist_gap << endl;
                  cout << "v time_to_collision " << (behind_is_ok ? "" : "! ") << time_to_collision << endl;
                }
              }

              if (obstacle_ind_ahead == -1) {
                // No one ahead. By logic, there is someone behind.

                if (behind_is_ok) {
                  // Execute lane change. Go at speed limit.
                  target_speed = speed_limit;
                  target_lane = adj_lane;
                  mode = KEEP_LANE;

                  // Don't evaluate other lanes.
                  break;
                } else {
                  // Evaluate other lanes.
                  continue;
                }
              }

              // By logic, there is someone ahead.
              bool ahead_is_ok;
              double speed_ahead;
              {
                vector<double> obstacle_ahead = obstacles[obstacle_ind_ahead];
                double obst_speed = extract_obstacle_speed(obstacle_ahead);
                double obst_pos_endpath = extract_obstacle_position(obstacle_ahead) + obst_speed * prev_path_duration;

                double speed_gap = obst_speed - target_speed;
                double dist_gap = obst_pos_endpath - end_path_s;
                double time_to_collision = dist_gap / speed_gap;

                ahead_is_ok = time_to_collision > 1.0;

                // speed_gap_ahead_is_ok = speed_gap > 0;
                // dist_gap_ahead_is_ok = dist_gap > lane_change_obstacle_buffer_ahead;

                // ahead_is_ok =
                //   dist_gap > lane_change_obstacle_buffer_ahead * 1.5 ||
                //   (
                //     dist_gap > lane_change_obstacle_buffer_ahead &&
                //     speed_gap > 0
                //   );

                speed_ahead = obst_speed;

                if (debug) {
                  // cout << "^ speed gap " << (speed_gap_ahead_is_ok ? "" : "! ") << speed_gap << endl;
                  // cout << "^ dist gap " << (dist_gap_ahead_is_ok ? "" : "! ") << dist_gap << endl;
                  cout << "^ time_to_collision " << (ahead_is_ok ? "" : "! ") << time_to_collision << endl;
                }
              }

              if (ahead_is_ok && behind_is_ok) {
                // Execute lane change. Go at speed of obstacle ahead.
                target_speed = speed_ahead;
                target_lane = adj_lane;
                mode = KEEP_LANE;

                // Don't evaluate other lanes.
                break;
              }

              // Try other lanes. Do not `break` here.

            } // end of `for each lane in adjacent_lanes`
          } // end of `if (mode == PLAN_LANE_CHANGE)`

          // TODO restore the if condition? If keeping, move up.
          // else if (mode == KEEP_LANE) {
            // Set target_speed. Do not set target_lane.
            // if (closest_obstacle_inds[target_lane] == -1) {
            //   target_speed = speed_limit;
            // } else {


            keep_lane(
              target_lane,
              obstacles, closest_obstacle_inds,
              car_s, car_speed,
              end_path_s, end_path_speed,
              prev_path_duration,
              target_speed, mode,
              debug);
            {

            }
            // }
          // }

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
            cout << "mode " << mode
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
