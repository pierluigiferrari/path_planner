/*
 * predictor.cpp
 *
 *  Created on: Sep 16, 2017
 *      Author: pierluigiferrari
 */

#include "predictor.h"

predictor::predictor() {}

predictor::predictor(vector<vector<double>> sensor_fusion)
{
  sensor_fusion_ = sensor_fusion;
}

predictor::~predictor() {}

vector<double> predictor::position(int index, double t)
{
  int i = index;

  double car_vx  = sensor_fusion_[i][3];
  double car_vy  = sensor_fusion_[i][4];
  double car_s   = sensor_fusion_[i][5];
  double car_d   = sensor_fusion_[i][6];
  //double car_yaw = atan2(car_vy, car_vx);

  double car_v = sqrt(car_vx * car_vx + car_vy * car_vy); // Compute the other car's velocity from the x and y components.
  // Predict the car's future position (at the end of the planning horizon) using its velocity, assuming it will follow its current lane.
  double car_s_pred = car_s + car_v * t;
  // For the future d value, assume that the car will always steer towards the center of its current lane.
  int car_lane = (int) car_d / 4;
  double car_d_pred = 2 + 4 * (double) car_lane;

  return {car_s_pred, car_d_pred, (double) car_lane};
}

double predictor::velocity(int index)
{
  int i = index;

  double car_vx  = sensor_fusion_[i][3];
  double car_vy  = sensor_fusion_[i][4];

  return sqrt(car_vx * car_vx + car_vy * car_vy);
}
