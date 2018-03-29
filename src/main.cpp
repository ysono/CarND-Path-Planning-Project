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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

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

//////////////////
// TODO consts here
int num_lanes = 3;
double lane_width = 4; // meter
double obstacle_detection_delta_s = 30; // meter
//////////////////

int lane_d_to_index(double d) {
  return floor(d / lane_width);
}

/**
  * Returns indexes of {
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

    vector<int> relevant_lane_inds;
    if (d < lane_width + d_buffer) {
      relevant_lane_inds.push_back(0);
    }
    if (lane_width - d_buffer < d && d < lane_width * 2 + d_buffer) {
      relevant_lane_inds.push_back(1);
    }
    if (lane_width * 2 - d_buffer < d) {
      relevant_lane_inds.push_back(2);
    }

    for(unsigned int i = 0; i < relevant_lane_inds.size(); i++) {
      int lane_ind = relevant_lane_inds[i];

      if (s > ref_s && s < closest_s_ahead[lane_ind]) {
        closest_s_ahead[lane_ind] = s;
        obstacle_inds[lane_ind] = obstacle_ind;
      } else if (s < ref_s && s > closest_s_behind[lane_ind]) {
        closest_s_behind[lane_ind] = s;
        obstacle_inds[lane_ind + num_lanes] = obstacle_ind;
      }
    }
  }

  return obstacle_inds;
}

int main() {
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

  int target_lane_ind = 1;
  double end_path_speed = 0;

  h.onMessage(
    [&end_path_speed, &target_lane_ind,
    &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy
    ]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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
        	double car_yaw = j[1]["yaw"];
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

          ///////////////////////////

          // TODO consts here

          // double lane_width = 4; // meter
          // double obstacle_detection_delta_s = 30; // meter

          // == `0.5 / 2.236936`. I don't think this calculation is meaningful. But it works.
          double default_accel = .224; // meter/sec/sec

          double mps_to_mph = 2.236936; // 1 meter/sec equals this much mile/hour

          double path_interval = 0.02; // sec
          
          ///////////////////////////

          // set car_s and car_d to the end of the existing path
          if (previous_path_size > 0) {
            car_s = end_path_s;
            car_d = end_path_d;
          }

          bool too_close = false;


          vector<int> obstacle_inds = find_closest_obstacles(obstacles, car_s);

          if (obstacle_inds[target_lane_ind] != -1) {
            vector<double> obstacle = obstacles[obstacle_inds[target_lane_ind]];

            double vx = obstacle[3];
            double vy = obstacle[4];
            double obstacle_speed = sqrt(pow(vx, 2) + pow(vy, 2));
            double obstacle_pos_s = obstacle[5];

            // Assuming obstacle keeps the same speed, it will be here by the time
            // self get here.
            double obstacle_future_pos_s =
              obstacle_pos_s + (double) previous_path_size * path_interval * obstacle_speed;

            if (obstacle_future_pos_s - car_s < obstacle_detection_delta_s) {
              too_close = true;

              // target_lane_ind -= 1; // TODO remove
              if (end_path_speed > obstacle_speed) {
                end_path_speed -= default_accel; 
              }
            }
          }


          // change velocity for all points output in this iteration
          // further optimization todo would change velocity within points output in this iteration
          // another problem is when there is an obstacle, the target never stabliizes at obstacle's speed but instead fluctuates lte that speed.
          if (too_close) {
          } else if (end_path_speed < 49.5) {
            end_path_speed += default_accel;
          }

          ///---

          // what i call markers. first they're in x,d. these will be transformed to p,q; then fed into spline.
          vector<double> ptsx, ptsy;

          // these refs contain info for the starting point of additional path generation
          // ie they're based on 2 previous points.
          double ref_x, ref_y, ref_yaw;

          // add 2 previous points to markers
          if (previous_path_size < 2) {
            ref_x = car_x;
            ref_y = car_y;
            ref_yaw = deg2rad(car_yaw);

            ptsx.push_back(car_x - cos(car_yaw));
            ptsy.push_back(car_y - sin(car_yaw));
            ptsx.push_back(car_x);
            ptsy.push_back(car_y);
          } else {
            ref_x = previous_path_x[previous_path_size - 1];
            ref_y = previous_path_y[previous_path_size - 1];
            double ref_x_prev = previous_path_x[previous_path_size - 2];
            double ref_y_prev = previous_path_y[previous_path_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // add more markers. add sparse points. pick s,d and convert to x,y using getXY()
          // 30 meters is far ahead enough for lane change
          vector<double> next_wp0 = getXY(car_s + 30, 2 + 4 * target_lane_ind, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, 2 + 4 * target_lane_ind, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, 2 + 4 * target_lane_ind, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // transform markers to p,q in-place
          for (int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
            ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
          }

          tk::spline s;
          s.set_points(ptsx, ptsy);

          // these are the path points to be output
        	vector<double> next_x_vals;
        	vector<double> next_y_vals;

          // add back all previous points
          for (int i = 0; i < previous_path_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // add new path points
          // first pick x interval
          double target_x = 30; // arbitrary. this is NOT the limit of further x being added.
          double target_y = s(target_x);
          double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
          double N = target_dist / (path_interval * end_path_speed / mps_to_mph);
          double x_interval = target_x / N;

          // add as many points as needed to refill 50. this 50 count limits further x to be added.
          // generate in p,q; transform to x,y; add to next_x_vals and next_y_vals vectors.
          for (int i = 0; i <= 50 - previous_path_size; i++) {
            // to avoid repeating ref_x and ref_y, must use `+1`. makes a big difference!
            double x_point = x_interval * (i + 1);
            double y_point = s(x_point);

            double x_ref = x_point;
            double y_ref = y_point;

            // transform from p,q to x,y
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }






          ///////////////////////////

        	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          json msgJson;
        	msgJson["next_x"] = next_x_vals;
        	msgJson["next_y"] = next_y_vals;

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
