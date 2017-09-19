/*
 * helper_functions.cpp
 *
 *  Created on: Sep 19, 2017
 *      Author: pierluigiferrari
 */

#include "helper_functions.h"

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double mph2mps(double x) { return x * 0.447; }
double mps2mph(double x) { return x / 0.447; }

double l2_distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int closest_waypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{
  double closest_len = 100000; // large number
  int closest_wp = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = l2_distance(x, y, map_x, map_y);
    if(dist < closest_len)
    {
      closest_len = dist;
      closest_wp = i;
    }
  }

  return closest_wp;
}

int next_waypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
  int closest_wp = closest_waypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closest_wp];
  double map_y = maps_y[closest_wp];

  double heading = atan2(map_y - y, map_x - x);

  double angle = fabs(theta - heading);

  if(angle > pi()/4)
  {
    closest_wp++;
  }

  return closest_wp;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> get_frenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
  int next_wp = next_waypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // Find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = l2_distance(x_x, x_y, proj_x, proj_y);

  // See if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double center2pos = l2_distance(center_x, center_y, x_x, x_y);
  double center2ref = l2_distance(center_x, center_y, proj_x, proj_y);

  if(center2pos <= center2ref)
  {
    frenet_d *= -1;
  }

  // Calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += l2_distance(maps_x[i], maps_y[i], maps_x[i+1], maps_y[i+1]);
  }

  frenet_s += l2_distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

// Transform from Frenet (s, d) coordinates to Cartesian (x, y)
vector<double> get_xy(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2(maps_y[wp2] - maps_y[prev_wp], maps_x[wp2] - maps_x[prev_wp]);
  // The x, y, s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

vector<double> get_frenet_velocity_vxvy(double x, double y, double vx, double vy, vector<double> maps_x, vector<double> maps_y)
{
  double car_heading = atan2(vy, vx);

  int next_wp = next_waypoint(x, y, car_heading, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if(next_wp == 0)
  {
    prev_wp = maps_x.size() - 1;
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
  double car_heading = theta;

  int next_wp = next_waypoint(x, y, car_heading, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if(next_wp == 0)
  {
    prev_wp = maps_x.size() - 1;
  }

  double lane_heading = atan2(maps_y[next_wp] - maps_y[prev_wp], maps_x[next_wp] - maps_x[prev_wp]);

  double delta_theta = car_heading - lane_heading;

  double norm_v = speed;

  double v_s = norm_v * cos(delta_theta);
  double v_d = norm_v * sin(delta_theta);

  return {v_s, v_d};
}

