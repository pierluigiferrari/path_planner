/*
 * trajectory_generator.h
 *
 *  Created on: Sep 14, 2017
 *      Author: pierluigiferrari
 */

#ifndef TRAJECTORY_GENERATOR_H_
#define TRAJECTORY_GENERATOR_H_

#include <vector>

using namespace std;

class TrajectoryGenerator {

public:

  /*
   * Default constructor
   */
  TrajectoryGenerator();

  /*
   * Constructor
   */
  TrajectoryGenerator(vector<double> map_waypoints_x,
                      vector<double> map_waypoints_y,
                      vector<double> map_waypoints_s);

  /*
   * Destructor
   */
  virtual ~TrajectoryGenerator();

  /*
   *
   */
  vector<vector<double>> generate_trajectory(int target_lane,
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
                                             double ref_d);

  // Returns the trajectory point at time t
  double operator() (double t) const;

private:
  // The waypoints that define the map. Necessary to convert between Cartesian and Frenet coordinates.
  vector<double> map_waypoints_x_;
  vector<double> map_waypoints_y_;
  vector<double> map_waypoints_s_;
};

#endif /* TRAJECTORY_GENERATOR_H_ */
