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

class BehaviorPlanner
{
public:

  /*
   * Constructor
   *
   * @param num_lanes The constant number of lanes on the same side of the road.
   * @param speed_limit The constant speed limit.
   * @param gnb A Gaussian Naive Bayes classifier object.
   * @param map_waypoints_x The list of Cartesian x coordinates of the waypoints that define the track.
   * @param map_waypoints_y The list of Cartesian y coordinates of the waypoints that define the track.
   * @param map_waypoints_s The list of Frenet s coordinates of the waypoints that define the track.
   * @param planning_horizon How many seconds into the future the planner should plan. Defaults to 1 second.
   * @param frontal_buffer The minimum distance in meters that the ego car tries to keep to any leading cars.
   * @param lateral_buffer The minimum distance in meters in every direction that the ego car tries to keep to
   *                       any objects around it.
   * @param speed_tolerance The tolerance around the target speed that the ego car must stay within.
   * @param cost_weights The weights of the cost functions used to determine the best successor state.
   *
   */
  BehaviorPlanner(int num_lanes,
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
  virtual ~BehaviorPlanner();

  /*
   * Computes the best next state and returns a trajectory for that state.
   *
   * @param car_x The ego car's last measured Cartesian x coordinate in meters.
   * @param car_y The ego car's last measured Cartesian y coordinate in meters.
   * @param car_s The ego car's last measured Frenet s coordinate in meters.
   * @param car_d The ego car's last measured Frenet d coordinate in meters.
   * @param car_yaw The ego car's last measured heading measured counter-clockwise
   *                against the x axis in degrees.
   * @param car_speed The ego car's scalar velocity, i.e. the L2 norm of the velocity vector.
   * @param previous_path_x A list of the x coordinates of the planned path points from
   *                        the last planning cycle minus those points that the car has already
   *                        passed.
   * @param previous_path_y A list of the y coordinates of the planned path points from
   *                        the last planning cycle minus those points that the car has already
   *                        passed.
   * @param end_path_s The Frenet s coordinate of the last path point of the previous planning cycle.
   * @param end_path_d The Frenet d coordinate of the last path point of the previous planning cycle.
   * @param sensor_fusion A list containing the noise-free state of all other sensed objects
   *                      on the same side of the road. For each object on the same side of the road,
   *                      its state is represented in the format `[id, x, y, vx, vy, s, d]`, with:
   *                       - `id` is the object's unique identifier.
   *                       - `(x, y)` are the Cartesian coordinates of the object's last measured location in meters.
   *                       - `(vx, vy)` are the components of the object's velocity vector in meters per second.
   *                       - `(s, d)` are the Frenet coordinates of the object's last measured location in meters.
   *
   * @returns A 2-dimensional vector of shape `(N, 5)` containing the planned path in the form of a list of
   *          the next `N` path points in the format `[x, y, v, a, yaw]`.
   *          The temporal distance between any two given path points and between the car's last measured state
   *          and the first path point is 20 milliseconds, i.e. the ego car must pass each subsequent path point
   *          in 20 millisecond increments.
   *           - `(x, y)` are the Cartesian coordinates of the path point in meters.
   *           - `v` is the L2 norm of the velocity vector at that path point in meters per second,
   *                 i.e. the car's scalar velocity.
   *           - `a` is the L2 norm of the acceleration vector at that path point in meters per second per second,
   *                 i.e. the car's scalar acceleration.
   *           -`yaw` is the angle of the car's heading at that path point measured counter-clockwise
   *                  against the x-axis in radians.
   */
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

  /*
   * Returns a 2-tuple with the indices of the leading and trailing vehicles in the target lane.
   * The indices refer to the indices of the `sensor_fusion_` vector. The leading vehicle is
   * the closest predicted vehicle in front of the ego car at the end of its current path in
   * the target lane, while the trailing vehicle is the closest predicted vehicle behind the
   * ego car at the end of its current path in the target lane.
   *
   * @param lane The lane number for which the leading and trailing vehicles are to be returned.
   */
  vector<int> get_leading_trailing(int lane);

  /*
   * Returns the maximum possible scalar velocity in meters per second depending on possible
   * traffic in front of the ego car and the speed limit.
   *
   * @param leading_trailing The output of `get_leading_trailing()`.
   */
  double compute_target_velocity(vector<int> leading_trailing);

  /*
   * Determines whether the ego car violates the frontal safety buffer to any car in front of it
   * and computes the required speed and time to reach that speed to re-establish the frontal
   * safety buffer quickly.
   *
   * @returns A 2-tuple with the first value being the ego car's required scalar velocity
   *          in meters per second and the second value being the time in seconds within
   *          which the ego car must reach that required velocity.
   */
  vector<double> safety_brake();

  /*
   * Determines whether or not a lane change is safe. A lane change is safe if the ego car
   * would not come closer than the lateral safety buffer to any other car's predicted position
   * at any point during the trajectory that would execute the lane change.
   *
   * @param trajectory The trajectory whose safety is to be determined. This is the output
   *                   of the trajectory generator, a 2-dimensional vector of shape `(N, 5)`
   *                   containing the planned path in the form of a list of the next `N` path
   *                   points in the format `[x, y, v, a, yaw]`. The temporal distance between
   *                   any two given path points and between the car's last measured state and
   *                   the first path point is 20 milliseconds, i.e. the ego car must pass each
   *                   subsequent path point in 20 millisecond increments.
   *                    - `(x, y)` are the Cartesian coordinates of the path point in meters.
   *                    - `v` is the L2 norm of the velocity vector at that path point in meters
   *                      per second, i.e. the car's scalar velocity.
   *                    - `a` is the L2 norm of the acceleration vector at that path point in
   *                      meters per second per second, i.e. the car's scalar acceleration.
   *                    - `yaw` is the angle of the car's heading at that path point measured
   *                      counter-clockwise against the x-axis in radians.
   * @param target_lane The target lane for the lane change.
   */
  bool lane_change_safe(vector<vector<double>>& trajectory, int target_lane);

  /*
   * Returns the Frenet s coordinate in meters of the last point of the ego car's current path.
   */
  double get_ref_s();

  /*
   * Returns the Frenet d coordinate in meters of the last point of the ego car's current path.
   */
  double get_ref_d();

  /*
   * Returns the time in seconds between the present and the last point of the ego car's current path.
   */
  double get_ref_t();

  /*
   * Returns a value within `[0, stop_cost]`, the cost of a trajectory regarding its average velocity.
   * The higher the average velocity of the trajectory, the lower the cost. The cost for driving at
   * or beyond the speed limit is 0, the cost for standing still is `stop_cost`, and the points in
   * between are linearly interpolated.
   *
   * @param stop_cost The cost for standing still.
   */
  double cost_velocity(const vector<vector<double>>& trajectory, double stop_cost = 1.0);

  /*
   * Returns 1 if the velocity at any point of the trajectory is higher than the speed limit
   *         and 0 otherwise.
   */
  double cost_speed_limit(const vector<vector<double>>& trajectory);

  /*
   * Returns 1 if the `target_lane` is not the ego car's current lane and 0 otherwise.
   */
  double cost_lane_change(int target_lane);

  /*
   * Returns 1 if the car is in the left-most or right-most lane and 0 otherwise.
   */
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
