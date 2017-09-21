/*
 * behavior_planner.h
 *
 *  Created on: Sep 16, 2017
 *      Author: pierluigiferrari
 */

#ifndef BEHAVIOR_PLANNER_H_
#define BEHAVIOR_PLANNER_H_

#include <vector>
#include <string>
#include <math.h>
#include "gaussian_naive_bayes.h"
#include "predictor.h"
#include "trajectory_generator.h"

using namespace std;

class behavior_planner
{
public:

  /*
   * Constructor
   *
   * @param planning_horizon How many seconds into the future the planner should plan. Defaults to 1 second.
   * @param map_waypoints_x The list of x coordinates of the waypoints that define the track.
   * @param map_waypoints_y The list of y coordinates of the waypoints that define the track.
   * @param map_waypoints_s The list of Frenet s coordinates of the waypoints that define the track.
   */
  behavior_planner(int num_lanes,
                   double speed_limit,
                   GNB gnb,
                   vector<double> map_waypoints_x,
                   vector<double> map_waypoints_y,
                   vector<double> map_waypoints_s,
                   double planning_horizon = 1.0,
                   double frontal_buffer = 30.0,
                   double lateral_buffer = 3.0,
                   double speed_tolerance = 0.2,
                   vector<double> cost_weights = {0.64, 0.33, 0.01, 0.02});

  /*
   * Destructor
   */
  virtual ~behavior_planner();

  vector<vector<double>> transition(double car_x,
                                    double car_y,
                                    double car_s,
                                    double car_d,
                                    double car_yaw,
                                    double car_speed,
                                    vector<double> previous_path_x,
                                    vector<double> previous_path_y,
                                    double end_path_s,
                                    double end_path_d,
                                    vector<vector<double>> sensor_fusion);

  vector<vector<double>> generate_trajectory(string state);

  // The ego car's localization data
  double car_x_ = 0;
  double car_y_ = 0;
  double car_s_ = 0;
  double car_d_ = 0;
  double car_yaw_ = 0;
  double car_speed_ = 0;
  // The portion of the previously generated path that hasn't been driven yet
  vector<double> previous_path_x_;
  vector<double> previous_path_y_;
  double end_path_s_ = 0;
  double end_path_d_ = 0;
  // All other cars' localization data
  vector<vector<double>> sensor_fusion_;

  int target_lane_ = 0; // The current target lane

private:

  vector<int> get_leading_trailing(int lane);

  double compute_target_velocity(vector<int> leading_trailing);

  vector<double> safety_brake();

  bool lane_change_safe(vector<vector<double>> trajectory, int target_lane);

  double get_ref_s();

  double get_ref_d();

  double get_ref_t();

  double cost_velocity(const vector<vector<double>> &trajectory, double stop_cost = 1.0);

  double cost_speed_limit(const vector<vector<double>> &trajectory);

  double cost_lane_change(int target_lane);

  double cost_outer_lane(int target_lane);

  int num_lanes_; // The number of lanes on the highway
  double speed_limit_; // The current speed limit
  Predictor pred_; // The predictor
  TrajectoryGenerator tra_gen_; // The trajectory generator
  double speed_tolerance_; // How much the ego car is allowed to deviate from the speed limit in meters per second
  double lateral_buffer_; // The minimum lateral space to a leading car in order to be allowed to pass it
  double frontal_buffer_; // The frontal safety buffer to be maintained to a leading car
  vector<double> cost_weights_; // The weights of the cost functions in the order of declaration
  string current_state_;
  double cycle_ref_speed_; // The reference speed from the last cycle to the next
  double planning_horizon_; // How many seconds into the future the planner should plan ahead
  vector<double> map_waypoints_x_;
  vector<double> map_waypoints_y_;
  vector<double> map_waypoints_s_;
};



#endif /* BEHAVIOR_PLANNER_H_ */
