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
   * Computes a low-jerk trajectory for that given boundary conditions.
   *
   * @param target_lane The target lane number for the trajectory, where 0 is
   *                    the left-most lane on the same side of the road.
   * @param target_velocity The target speed for the trajectory in meters per second.
   * @param time_to_reach_tl The time allowed to reach the target lane in seconds.
   *                         This determines how smoothly or abruptly a lane change
   *                         must be executed.
   * @param time_to_reach_tv The time allowed to reach the target velocity in seconds.
   *                         This determines the constant acceleration applied to attain
   *                         the target velocity.
   * @param planning_horizon The temporal length of the trajectory in seconds.
   * @param full_path If `true`, a trajectory of the full length of the planning horizon
   *                  will be generated on top of the previous path points to be kept in the
   *                  generated trajectory.
   *                  If `false`, only enough new trajectory points will be generated in order
   *                  to get an overall path of the length of the planning horizon with a
   *                  combination of the previous path points to be kept and the newly generated
   *                  path points.
   * @param car_x The car's last measured Cartesian x coordinate in meters.
   * @param car_y The car's last measured Cartesian y coordinate in meters.
   * @param car_s The car's last measured Frenet s coordinate in meters.
   * @param car_d The car's last measured Frenet d coordinate in meters.
   * @param car_yaw The ego car's last measured heading measured counter-clockwise
   *                against the x axis in degrees.
   * @param ref_speed The car's scalar velocity, i.e. the L2 norm of the velocity vector, at the
   *                  time step one before the first newly generated path point. This is the speed
   *                  at the cross-over between the previously kept path points and the newly
   *                  generated path points.
   * @param previous_path_x A list of the x coordinates of the planned path points from
   *                        the last planning cycle minus those points that the car has already
   *                        passed.
   * @param previous_path_y A list of the y coordinates of the planned path points from
   *                        the last planning cycle minus those points that the car has already
   *                        passed.
   * @param num_prev_path_points_keep The number of path points of the previous path that should
   *                                  be kept for the new trajectory.
   * @param ref_s The Frenet s coordinate of the path point of the previous planning cycle after
   *              which the newly generated trajectory shall begin.
   * @param ref_d The Frenet d coordinate of the path point of the previous planning cycle after
   *              which the newly generated trajectory shall begin.
   *
   * @returns A 2-dimensional vector of shape `(N, 5)` containing the planned path in the form of a list of
   *          the next `N` path points in the format `[x, y, v, a, yaw]`.
   *          The temporal distance between any two given path points and between the car's last measured state
   *          and the first path point is 20 milliseconds, i.e. the ego car must pass each subsequent path point
   *          in 20 millisecond increments.
   *           - `(x, y)` are the Cartesian coordinates of the path point in meters.
   *           - `v` is the L2 norm of the velocity vector at that location in meters per second,
   *                 i.e. the object's scalar velocity.
   *           - `a` is the L2 norm of the acceleration vector at that location in meters per second per second,
   *                 i.e. the object's scalar acceleration.
   *           -`yaw` is the angle of the object's heading at that location measured counter-clockwise
   *                  against the x-axis in radians.
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
                                             double ref_speed,
                                             vector<double> previous_path_x,
                                             vector<double> previous_path_y,
                                             int num_prev_path_points_keep,
                                             double ref_s,
                                             double ref_d);

private:
  // The waypoints that define the map. Necessary to convert between Cartesian and Frenet coordinates.
  vector<double> map_waypoints_x_;
  vector<double> map_waypoints_y_;
  vector<double> map_waypoints_s_;
};

#endif /* TRAJECTORY_GENERATOR_H_ */
