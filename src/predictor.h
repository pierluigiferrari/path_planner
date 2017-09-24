/*
 * predictor.h
 *
 *  Created on: Sep 16, 2017
 *      Author: pierluigiferrari
 */

#ifndef PREDICTOR_H_
#define PREDICTOR_H_

#include <vector>
#include <math.h>
#include "gaussian_naive_bayes.h"
#include "trajectory_generator.h"

using namespace std;

class Predictor
{
public:

  /*
   * Default constructor
   */
  Predictor();

  /*
   * Constructor
   *
   * @param gnb A Gaussian Naive Bayes classifier object.
   * @param map_waypoints_x The list of Cartesian x coordinates of the waypoints that define the track in meters.
   * @param map_waypoints_y The list of Cartesian y coordinates of the waypoints that define the track in meters.
   * @param map_waypoints_s The list of Frenet s coordinates of the waypoints that define the track in meters.
   */
  Predictor(const GNB& gnb,
            const vector<double>& map_waypoints_x,
            const vector<double>& map_waypoints_y,
            const vector<double>& map_waypoints_s);

  /*
   * Destructor
   */
  virtual ~Predictor();

  /*
   * Updates the sensor fusion data.
   *
   * @param sensor_fusion Sensor fusion data, a list of all other cars on the same side of the road.
   *                      A 2-dimensional vector of shape (N, 7) where each entry represents a car by seven coordinates:
   *                      [car's unique ID,
   *                       car's x position in map coordinates,
   *                       car's y position in map coordinates,
   *                       car's x velocity in m/s,
   *                       car's y velocity in m/s,
   *                       car's s position in Frenet coordinates,
   *                       car's d position in Frenet coordinates].
   */
  void update(const vector<vector<double>>& sensor_fusion);

  /*
   * Predicts trajectories that span the length of the prediction horizon for objects
   * in the latest sensor fusion update. By default, predicted trajectories for all objects
   * are being generated.
   *
   * @param prediction_horizon The temporal length of the predicted trajectories in seconds.
   * @param index The index of the object for which a trajectory is to be predicted or -1 if
   *              trajectories are to be predicted for all objects in the latest sensor fusion
   *              update. Defaults to -1.
   */
  void predict_trajectories(double prediction_horizon, int index = -1);

  /*
   * Returns the predicted position of a car at time t in Frenet coordinates.
   *
   * @param index The row index in the sensor fusion data table of the car for which the position is to be predicted.
   * @param k The number of 20 ms time steps from the present time for which the position is to be predicted.
   *
   * @returns A vector of length 5 containing the the object's predicted state at time step `k` in
   *          the format `[x, y, v, a, yaw]`:
   *           - `(x, y)` are the Cartesian coordinates of the location in meters.
   *           - `v` is the L2 norm of the velocity vector at that location in meters per second,
   *                 i.e. the object's scalar velocity.
   *           - `a` is the L2 norm of the acceleration vector at that location in meters per second per second,
   *                 i.e. the object's scalar acceleration.
   *           -`yaw` is the angle of the object's heading at that location measured counter-clockwise
   *                  against the x-axis in radians.
   */
  vector<double> predict_location(int index, int k);

private:
  GNB gnb_; // The Gaussian Naive Bayes classifier
  TrajectoryGenerator tra_gen_; // The trajectory generator
  vector<vector<double>> sensor_fusion_; // The sensor fusion vector containing the current observations of the other objects on the road
  vector<vector<vector<double>>> pred_trajectories_; // The predicted trajectories
  // The map; for conversion between Cartesian and Frenet coordinates.
  vector<double> map_waypoints_x_;
  vector<double> map_waypoints_y_;
  vector<double> map_waypoints_s_;
};

#endif /* PREDICTOR_H_ */
