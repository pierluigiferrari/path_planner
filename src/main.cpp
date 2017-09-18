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

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
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

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
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

vector<double> get_frenet_velocity_vxvy(double x, double y, double vx, double vy, vector<double> maps_x, vector<double> maps_y)
{
  double car_heading  = atan2(vy, vx);

  int next_wp = NextWaypoint(x,y, car_heading, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }

  double lane_heading = atan2(maps_y[next_wp] - maps_y[prev_wp], maps_x[next_wp] - maps_x[prev_wp]);

  double delta_theta = car_heading - lane_heading;

  double norm_v = sqrt(pow(vx, 2) + pow(vy, 2));

  double v_s = norm_v * cos(delta_theta);
  double v_d = norm_v * sin(delta_theta);

  return {v_s, v_d};
}

vector<double> get_frenet_velocity_theta(double x, double y, double theta, double speed, vector<double> maps_x, vector<double> maps_y)
{
  double car_heading  = theta;

  int next_wp = NextWaypoint(x,y, car_heading, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }

  double lane_heading = atan2(maps_y[next_wp] - maps_y[prev_wp], maps_x[next_wp] - maps_x[prev_wp]);

  double delta_theta = car_heading - lane_heading;

  double norm_v = speed;

  double v_s = norm_v * cos(delta_theta);
  double v_d = norm_v * sin(delta_theta);

  return {v_s, v_d};
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

  // Define the lane we want to be in. The left-most lane is lane 0, the right-most lane is lane 2.
  int lane = 1;

  // Set the reference velocity that we want the car to keep.
  double ref_vel = 0.0;

  // Set the planning horizon in seconds.
  // This is the length of the planned path for the ego car and the length of the predicted paths for the other cars on the road.
  double planning_horizon = 1.0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &ref_vel, &lane, &planning_horizon](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	// A 2-dimensional vector of shape (N, 7) where each entry represents a car by seven coordinates:
          	// [car's unique ID,
          	//  car's x position in map coordinates,
          	//  car's y position in map coordinates,
          	//  car's x velocity in m/s,
          	//  car's y velocity in m/s,
          	//  car's s position in Frenet coordinates,
          	//  car's d position in Frenet coordinates]
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	int prev_size = previous_path_x.size();

          	json msgJson;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

          	if (prev_size > 0) {
          	  car_s = end_path_s;
          	}

          	bool too_close = false; // Indicator to register whether we need to deviate from our standard driving policy to just follow the current lane at constant speed

          	// Determine the reference velocity for the ego car
          	for (int i = 0; i < sensor_fusion.size(); i++)
          	{ // For each car around the ego car...

          	  double other_car_d = sensor_fusion[i][6]; // ...get the d-coordinate...

          	  if (other_car_d > (car_d - 3) && other_car_d < (car_d + 3))
          	  { // ...and if the other car is (nearly) in the same lane as the ego car,...

          	    // ...compute where the other car will be by the time we reach the end of the current path,
          	    // so that we can compute whether or not a collision would occur if we followed our current path.
          	    double other_car_vx = sensor_fusion[i][3];
          	    double other_car_vy = sensor_fusion[i][4];
          	    double other_car_v = sqrt(other_car_vx * other_car_vx + other_car_vy * other_car_vy); // Compute the other car's velocity from the x and y components
          	    double other_car_s = sensor_fusion[i][5];
          	    double other_car_s_future = other_car_s + other_car_v * 0.02 * (double) prev_size; // Predict the car's future position using its velocity

          	    if (other_car_s_future > car_s && (other_car_s_future - car_s) < 30)
          	    { // If the other car is in front of our car AND too close...

          	      too_close = true; // ...set the indicator to indicate that we need to act.

          	    }
          	  }
          	}

          	// For each non-ego car, generate a trajectory under the assumption that the car always keeps its lane, i.e. that it cannot change lanes
          	  // We need no `else` case, because cars cannot go off-road in this simulator, so no car will have a d value larger than 12.

          	  /* We need no spline as long as we only assume cars always keep their lane.
          	  // Generate a spline between the car's current and future position.
          	  vector<double> other_car_xy = getXY(other_car_s, other_car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	  vector<double> other_car_xy_aux0 = getXY(other_car_s_aux0, other_car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	  vector<double> other_car_xy_aux1 = getXY(other_car_s_aux1, other_car_d_future, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	  vector<double> other_car_xy_future = getXY(other_car_s_future, other_car_d_future, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          	  // Vectors to hold the spline end points
          	  vector<double> other_car_spline_pts_x;
          	  vector<double> other_car_spline_pts_y;
          	  other_car_spline_pts_x.push_back(other_car_xy[0]);
          	  other_car_spline_pts_x.push_back(other_car_xy_aux0[0]);
          	  other_car_spline_pts_x.push_back(other_car_xy_aux1[0]);
          	  other_car_spline_pts_x.push_back(other_car_xy_future[0]);
          	  other_car_spline_pts_y.push_back(other_car_xy[1]);
              other_car_spline_pts_y.push_back(other_car_xy_aux0[1]);
              other_car_spline_pts_y.push_back(other_car_xy_aux1[1]);
              other_car_spline_pts_y.push_back(other_car_xy_future[1]);

              // Transform the spline end points into the other car's local coordinate system.
              for (int i = 0; i < other_car_spline_pts_x.size(); i++)
              {
                double shifted_x = other_car_spline_pts_x[i] - other_car_x;
                double shifted_y = other_car_spline_pts_y[i] - other_car_y;

                other_car_spline_pts_x[i] = shifted_x * cos(0 - other_car_yaw) - shifted_y * sin(0 - other_car_yaw);
                other_car_spline_pts_y[i] = shifted_x * sin(0 - other_car_yaw) + shifted_y * cos(0 - other_car_yaw);
              }

              // Declare the spline
              tk::spline other_car_spline;

              // Set the end points for the spline
              other_car_spline.set_points(other_car_spline_pts_x, other_car_spline_pts_y);
              */

          	// Create lists to store the spline end points, i.e. the points between which we want the splines to interpolate
          	vector<double> spline_points_x;
          	vector<double> spline_points_y;

          	// Keep track of the reference position, which is the position from which we are currently viewing the world
          	double ref_x = car_x;
          	double ref_y = car_y;
          	double ref_yaw = deg2rad(car_yaw);

          	// Cover the initialization state, i.e. the state in which we have less than 2 path points in our previous path.
          	if (prev_size < 2) { // If we have fewer than 2 path points in our previous path...
          	  // ...generate a pseudo-previous position by just going backwards from where the car is now...
          	  double prev_car_x = car_x - cos(car_yaw);
          	  double prev_car_y = car_y - sin(car_yaw);
          	  // ...and use the car's current and the generated previous position to form a path that is tangential to the car's heading.
          	  spline_points_x.push_back(prev_car_x);
          	  spline_points_x.push_back(car_x);
          	  spline_points_y.push_back(prev_car_y);
          	  spline_points_y.push_back(car_y);
          	}
          	// Now cover the states in which we have a sufficiently long previous path.
          	else { // Or else, if we have enough previous path points...
          	  // ...make the last path point of the previous path the new reference position...
          	  ref_x = previous_path_x[prev_size - 1];
          	  ref_y = previous_path_y[prev_size - 1];
          	  // ...and use the second-to-last path point to get the tangent to the car's heading at that position...
          	  double prev_car_x = previous_path_x[prev_size - 2];
          	  double prev_car_y = previous_path_y[prev_size - 2];
          	  // ...and compute the reference yaw from these two points...
          	  ref_yaw = atan2(ref_y - prev_car_y, ref_x - prev_car_x);
          	  // ...and push the points onto the spline point list
          	  spline_points_x.push_back(prev_car_x);
              spline_points_x.push_back(ref_x);
              spline_points_y.push_back(prev_car_y);
              spline_points_y.push_back(ref_y);
          	}

          	// Add three additional spline end points that are spaced far apart
          	vector<double> spline_point_3 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> spline_point_4 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> spline_point_5 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          	spline_points_x.push_back(spline_point_3[0]);
          	spline_points_x.push_back(spline_point_4[0]);
          	spline_points_x.push_back(spline_point_5[0]);
          	spline_points_y.push_back(spline_point_3[1]);
            spline_points_y.push_back(spline_point_4[1]);
            spline_points_y.push_back(spline_point_5[1]);

            // Transform the spline points into the car's local coordinate system
            for (int i = 0; i < spline_points_x.size(); i++)
            {
              double shifted_x = spline_points_x[i] - ref_x;
              double shifted_y = spline_points_y[i] - ref_y;

              spline_points_x[i] = shifted_x * cos(0 - ref_yaw) - shifted_y * sin(0 - ref_yaw);
              spline_points_y[i] = shifted_x * sin(0 - ref_yaw) + shifted_y * cos(0 - ref_yaw);
            }

            // Declare the spline
            tk::spline spline;

            // Set the end points for the spline
            spline.set_points(spline_points_x, spline_points_y);

            // Make a list for the actual path points we will generate from this spline
            vector<double> next_x_vals;
            vector<double> next_y_vals;

            // First, add all remaining previous path points to the new path, if any
            for (int i = 0; i < prev_size; i++) {

              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);

            }

            // Now, fill up the missing path points up to 50 with points generated from
            // the spline.

            // In order to that, first compute how to subdivide the spline into points spaced
            // such that the car maintains its target velocity
            double target_x = 30; // Remember, we're in the car's perspective right now
            double target_y = spline(target_x);
            double target_dist = sqrt(target_x * target_x + target_y * target_y);

            double x_offset = 0; // An offset for the loop below

            // We're filling up as many missing path points such that we always have 50 path points for the new path
            for (int i = 1; i <= planning_horizon / 0.02 - prev_size; i++) {

              if (too_close) { // If we're getting too close to the car in front of us...
                ref_vel -= 0.224; // ...reduce the speed by 0.224 m/s per 20 ms,...
              }
              else if (ref_vel < 49.5) { // ...or else if we are slower than the speed limit, gradually speed up
                ref_vel += 0.224;
              }

              // Compute the number of segments to subdivide the next 30-meter stretch of the spline into
              double N = target_dist / (0.02 * ref_vel / 2.24); // Car visits a new path point every 20 ms and dividing by 2.24 converts the reference velocity from mph to m/s.
              double next_x = x_offset + target_x / N;
              double next_y = spline(next_x);

              x_offset = next_x;

              // Transform the next point back from the car's local to the global coordinate system.
              double temp_x = next_x;
              double temp_y = next_y;
              next_x = temp_x * cos(ref_yaw) - temp_y * sin(ref_yaw);
              next_y = temp_x * sin(ref_yaw) + temp_y * cos(ref_yaw);

              next_x += ref_x;
              next_y += ref_y;

              // Push this next path point onto the list
              next_x_vals.push_back(next_x);
              next_y_vals.push_back(next_y);

            }

          	// END

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
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
