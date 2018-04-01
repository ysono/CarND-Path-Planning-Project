#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <tuple>
#include <vector>
#include <functional>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "support.h"

using std::cout;
using std::endl;
using std::string;
using std::vector;
using nlohmann::json;

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

int main(int argc, char* argv[]) {
  PP_DEBUG = argc >= 2 && strcmp(argv[1], "--debug") == 0;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  std::function<vector<double>(double, double)> sd_to_xy =
  [&map_waypoints_s, &map_waypoints_x, &map_waypoints_y]
  (double s, double d) {
    return getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  };

  FSM fsm {KEEP_LANE, 1, SPEED_LIMIT};

  // A requried property of the previously planned path that's not retained and
  // provided by the simulator.
  double end_path_speed = 0;

  h.onMessage(
    [&fsm, &end_path_speed, &sd_to_xy]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

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
          
          Telemetry telemetry;

          // Main car's localization Data
          telemetry.now_x = j[1]["x"];
          telemetry.now_y = j[1]["y"];
          telemetry.now_s = j[1]["s"];
          telemetry.now_d = j[1]["d"];
          double now_yaw = j[1]["yaw"]; // deg
          telemetry.now_yaw = deg2rad(now_yaw); // rad
          telemetry.now_speed = j[1]["speed"]; // keep all speed vars as mph

          // Previous path data given to the Planner
          vector<double> future_path_x = j[1]["previous_path_x"];
          vector<double> future_path_y = j[1]["previous_path_y"];
          telemetry.future_path_x = future_path_x;
          telemetry.future_path_y = future_path_y;
          telemetry.future_path_size = future_path_x.size();
          telemetry.future_path_duration = PATH_INTERVAL * telemetry.future_path_size;

          // Previous path's end s and d values 
          telemetry.future_s = j[1]["end_path_s"];
          telemetry.future_d = j[1]["end_path_d"];
          telemetry.future_speed = end_path_speed;

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          vector<vector<double> > now_obstacles = j[1]["sensor_fusion"];
          telemetry.now_obstacles = now_obstacles;

          ////// End of unravelling telemtry data //////

          if (PP_DEBUG) {
            cout << "======" << endl << fsm;
          }

          vector<double> next_path_x, next_path_y;

          if (telemetry.future_path_size >= NUM_OUTPUT_PATH_POINTS) {
            if (PP_DEBUG) {
              cout << "no need to generate path points" << endl;
            }
          } else {
            if (telemetry.future_path_size == 0) {
              telemetry.future_s = telemetry.now_s;
              telemetry.future_d = telemetry.now_d;
            }

            fsm = iterate_fsm(fsm, telemetry);

            // For all modes, adjust acceleration only.
            // We're using the same speed for all path points to be added.
            // Although it would be more effective to use different speeds for
            // points to be added, in practice it should make little difference
            // becuase we rarely have to generate more than 1 to 3 points.
            if (end_path_speed < fsm.target_speed) {
              end_path_speed += DEFAULT_ACCEL;
            } else {
              end_path_speed -= DEFAULT_ACCEL;
            }

            std::tie(next_path_x, next_path_y) = generate_path(
              fsm.target_lane, end_path_speed, telemetry, sd_to_xy);
          }

          ////// Finished generating path //////

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
