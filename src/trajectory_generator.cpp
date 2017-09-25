/*
 * trajectory_generator.cpp
 *
 *  Created on: Sep 14, 2017
 *      Author: pierluigiferrari
 */

#include <math.h>
#include <vector>
#include <iostream>
#include "spline.h"
#include "trajectory_generator.h"
#include "helper_functions.h"

using namespace std;

TrajectoryGenerator::TrajectoryGenerator() {};

TrajectoryGenerator::TrajectoryGenerator(vector<double> map_waypoints_x,
                                         vector<double> map_waypoints_y,
                                         vector<double> map_waypoints_s)
{
  map_waypoints_x_ = map_waypoints_x;
  map_waypoints_y_ = map_waypoints_y;
  map_waypoints_s_ = map_waypoints_s;
}

TrajectoryGenerator::~TrajectoryGenerator() {}

vector<vector<double>> TrajectoryGenerator::generate_trajectory(int target_lane,
                                                                double target_velocity,
                                                                double time_to_reach_tl,
                                                                double time_to_reach_tv,
                                                                double planning_horizon,
                                                                bool full_path,
                                                                double car_x,
                                                                double car_y,
                                                                double car_s,
                                                                double car_d,
                                                                double car_yaw,
                                                                double end_path_v,
                                                                vector<double> previous_path_x,
                                                                vector<double> previous_path_y,
                                                                int num_prev_path_points_keep,
                                                                double ref_s,
                                                                double ref_d)
{
  // Create lists to store the spline end points, i.e. the points between which we want the splines to interpolate
  vector<double> spline_points_x;
  vector<double> spline_points_y;

  // Keep track of the reference position, which is the position from which we are currently viewing the world
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  // Cover the initialization state, i.e. the state in which we have less than 2 path points in our previous path.
  if (num_prev_path_points_keep < 2)
  { // If we have fewer than 2 path points in our previous path...
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
  else
  { // Or else, if we have enough previous path points...
    // ...make the last path point of the previous path the new reference position...
    ref_x = previous_path_x[num_prev_path_points_keep - 1];
    ref_y = previous_path_y[num_prev_path_points_keep - 1];
    // ...and use the second-to-last path point to get the tangent to the car's heading at that position...
    double prev_car_x = previous_path_x[num_prev_path_points_keep - 2];
    double prev_car_y = previous_path_y[num_prev_path_points_keep - 2];
    // ...and compute the reference yaw from these two points...
    ref_yaw = atan2(ref_y - prev_car_y, ref_x - prev_car_x);
    // ...and push the points onto the spline point list
    spline_points_x.push_back(prev_car_x);
    spline_points_x.push_back(ref_x);
    spline_points_y.push_back(prev_car_y);
    spline_points_y.push_back(ref_y);
  }

  // Add three additional spline end points that are spaced far apart.
  // Assume that each lane is 4 meters wide.

  // The average of the car's current speed and its target speed determines
  // the s coordinate by which the transition from the current lane to the target lane
  // must be completed.
  double avg_speed = 0.5 * (end_path_v + target_velocity);

  //vector<double> spline_point_3 = get_xy(ref_s + time_to_reach_tl * avg_speed, (2 + 4 * (double) target_lane), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  //vector<double> spline_point_4 = get_xy(ref_s + 2 * time_to_reach_tl * avg_speed, (2 + 4 * (double) target_lane), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  //vector<double> spline_point_5 = get_xy(ref_s + 3 * time_to_reach_tl * avg_speed, (2 + 4 * (double) target_lane), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);

  vector<double> spline_point_3 = get_xy(ref_s + 30, (2 + 4 * (double) target_lane), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  vector<double> spline_point_4 = get_xy(ref_s + 60, (2 + 4 * (double) target_lane), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  vector<double> spline_point_5 = get_xy(ref_s + 90, (2 + 4 * (double) target_lane), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);

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
  vector<double> next_v_vals; // Store velocity at each path point.
  vector<double> next_a_vals; // Store acceleration at each path point.
  vector<double> next_yaw_vals; // Store heading at each path point.

  // Determine how many path points to generate.
  int num_new_path_points;
  if (!full_path) // If we're just "filling up" the points of an existing previous path...
  {
    // ...add all remaining previous path points to the new path, if any,...
    for (int i = 0; i < num_prev_path_points_keep; i++)
    {
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
    }
    // ...and set the number of path points to be generated accordingly.
    num_new_path_points = planning_horizon / 0.02 - num_prev_path_points_keep;
  }
  // Otherwise, generate a full path.
  else num_new_path_points = planning_horizon / 0.02;

  // Now, fill up the missing path points up to 50 with points generated from
  // the spline.

  // In order to that, compute how to subdivide the spline into points spaced
  // such that the car maintains its reference velocity for a given stretch.
  // We do this by linearizing the curvature of the spline, which is an alright
  // approximation for short distances on a highway, but it's not an elegant solution.
  double target_x = 30; // Remember, we're in the car's perspective right now
  double target_y = spline(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  double x_offset = 0; // An offset for the loop below

  // Store the car's speed at the currently considered path planning point here
  double ref_speed = end_path_v;

  // We're filling up as many missing path points such that we always have the required number of path points for the new path
  // corresponding to the planning horizon at 50 path points per second.
  for (int i = 0; i < num_new_path_points; i++)
  {
    double acceleration = 0.0;

    // If necessary, accelerate or slow down between the last path point and this path point
    if (fabs(ref_speed - target_velocity) > 0.2) // Allow a 0.2 m/s tolerance.
    {
      // Compute the required acceleration to reach the target velocity in the required time,...
      acceleration = (target_velocity - ref_speed) / time_to_reach_tv;
      // ...with the acceleration being bounded within reasonable values.
      acceleration = max(-9.0, min(9.0, acceleration));
      ref_speed += acceleration * 0.02;
    }

    if (full_path)
    {
      next_v_vals.push_back(ref_speed);
      next_a_vals.push_back(acceleration);
    }

    // Compute the number of segments to subdivide the next 30-meter stretch of the spline into
    double N = target_dist / (0.02 * ref_speed); // Car visits a new path point every 20 ms
    double next_x = x_offset + target_x / N;
    double next_y = spline(next_x);

    //cout << "ref_speed: " << ref_speed << " N: " << N << " next_x: " << next_x << endl;

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

    if (full_path)
    {
      double next_yaw;
      if (i == 0) next_yaw = atan2(next_y_vals[i] - car_y, next_x_vals[i - 1] - car_x);
      else        next_yaw = atan2(next_y_vals[i] - next_y_vals[i - 1], next_x_vals[i] - next_x_vals[i - 1]);

      next_yaw_vals.push_back(next_yaw);
    }
  }

  return {next_x_vals, next_y_vals, next_v_vals, next_a_vals, next_yaw_vals};
}

