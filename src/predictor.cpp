/*
 * predictor.cpp
 *
 *  Created on: Sep 16, 2017
 *      Author: pierluigiferrari
 */

#include <iostream>
#include "predictor.h"
#include "helper_functions.h"

Predictor::Predictor() {}

Predictor::Predictor(const GNB& gnb,
                     const vector<double>& map_waypoints_x,
                     const vector<double>& map_waypoints_y,
                     const vector<double>& map_waypoints_s)
{
  gnb_ = gnb;
  tra_gen_ = TrajectoryGenerator(map_waypoints_x, map_waypoints_y, map_waypoints_s);
  map_waypoints_x_ = map_waypoints_x;
  map_waypoints_y_ = map_waypoints_y;
  map_waypoints_s_ = map_waypoints_s;
}

Predictor::~Predictor() {}

void Predictor::update(const vector<vector<double>>& sensor_fusion)
{
  sensor_fusion_ = sensor_fusion;
}

void Predictor::predict_trajectories(double prediction_horizon, int index)
{
  // Clear any previous trajectory predictions.
  pred_trajectories_ = vector<vector<vector<double>>>();

  int start_index;
  int end_index;
  if (index == -1)
  {
    start_index = 0;
    end_index = sensor_fusion_.size();
  }
  else
  {
    start_index = index;
    end_index = index + 1;
  }

  for (int i = start_index; i < end_index; i++) // Iterate over the objects of interest, either one or all.
  {
    double car_x   = sensor_fusion_[i][1];
    double car_y   = sensor_fusion_[i][2];
    double car_vx  = sensor_fusion_[i][3];
    double car_vy  = sensor_fusion_[i][4];
    double car_s   = sensor_fusion_[i][5];
    double car_d   = sensor_fusion_[i][6];
    double car_yaw = atan2(car_vy, car_vx);
    double car_v   = sqrt(car_vx * car_vx + car_vy * car_vy);

    // It is really annoying, but the sensor_fusion list always has 12 entries, some of which are dummy
    // entries with d<0 and must be excluded from these predictions.
    if (car_d < 0) continue;

    // Compute the velocity vector in Frenet coordinates, vs and vd (= s_dot and d_dot),
    // since this is what the GNB classifier needs.
    vector<double> car_v_frenet = get_frenet(car_vx, car_vy, car_yaw, map_waypoints_x_, map_waypoints_y_);
    double car_vs = car_v_frenet[0];
    double car_vd = car_v_frenet[1];

    // Predict the car's current state based on the 4-tuple of observations (s, d, vs, vd).
    // At this time, the Gaussian Naive Bayes classifier used here can predict one of three
    // states: "keep"  (car will keep it's current lane),
    //         "left"  (car will make a left lane change or is already in the course of doing so),
    //         "right" (car will make a right lane change or is already in the course of doing so).
    vector<double> observation = {car_s, car_d, car_vs, car_vd};
    string pred_state = gnb_.predict(observation);
    pred_state = "keep";

    if (pred_state == "keep")
    {
      // If the predicted state is "keep", simply assume that the car will continue driving in its lane
      // at constant d and constant velocity.

      // For the future d value, assume that the car will always steer towards the center of its current lane.
      int car_lane = (int) car_d / 4;
      double car_d_pred = 2 + 4 * (double) car_lane;

      vector<double> next_x_vals;
      vector<double> next_y_vals;
      vector<double> next_v_vals;
      vector<double> next_a_vals;
      vector<double> next_yaw_vals;

      // Instead of using the more complex trajectory generator,
      // generate the path points since the trajectory is so simple.
      int num_path_points = prediction_horizon / 0.02;
      for (int t = 1; t <= num_path_points; t++)
      {
        double car_s_pred = car_s + car_v * t * 0.02;
        vector<double> car_xy = get_xy(car_s_pred, car_d_pred, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);

        next_x_vals.push_back(car_xy[0]);
        next_y_vals.push_back(car_xy[1]);
        next_v_vals.push_back(car_v);
        next_a_vals.push_back(0);

        double car_yaw_pred;
        if (t == 1) car_yaw_pred = atan2(next_y_vals[t - 1] - car_y, next_x_vals[t - 1] - car_x);
        else        car_yaw_pred = atan2(next_y_vals[t - 1] - next_y_vals[t - 2], next_x_vals[t - 1] - next_x_vals[t - 2]);

        next_yaw_vals.push_back(car_yaw_pred);
      }

      vector<vector<double>> pred_trajectory = {next_x_vals, next_y_vals, next_v_vals, next_a_vals, next_yaw_vals};

      pred_trajectories_.push_back(pred_trajectory);
    }
    else
    {
      // If the predicted state is "left" or "right", compute its target lane and generate a trajectory.
      // Compute the target lane for this test state.
      int car_lane = (int) car_d / 4;
      int target_lane;
      if      (pred_state == "left")  target_lane = car_lane - 1;
      else if (pred_state == "right") target_lane = car_lane + 1;

      // The trajectory generator formally needs a list of previous path points for this car.
      // Just pass an empty list.
      vector<double> previous_path_x;
      vector<double> previous_path_y;

      std::cout << "Predicting for an other car." << std::endl;
      // Compute the predicted trajectory for this vehicle.
      vector<vector<double>> pred_trajectory = tra_gen_.generate_trajectory(target_lane,
                                                                            car_v,
                                                                            2.0,
                                                                            0.0,
                                                                            prediction_horizon,
                                                                            true,
                                                                            car_x,
                                                                            car_y,
                                                                            car_s,
                                                                            car_d,
                                                                            car_yaw,
                                                                            car_v,
                                                                            previous_path_x,
                                                                            previous_path_y,
                                                                            0.0,
                                                                            0.0);

      pred_trajectories_.push_back(pred_trajectory);
    }
  }

  std::cout << "Other car trajectories predicted: " << pred_trajectories_.size() << std::endl;
}

vector<double> Predictor::predict_location(int index, int k)
{
  double car_x_pred   = pred_trajectories_[index][0][k];
  double car_y_pred   = pred_trajectories_[index][1][k];
  double car_v_pred   = pred_trajectories_[index][2][k];
  double car_a_pred   = pred_trajectories_[index][3][k];
  double car_yaw_pred = pred_trajectories_[index][4][k];

  return {car_x_pred, car_y_pred, car_v_pred, car_a_pred, car_yaw_pred};
}
