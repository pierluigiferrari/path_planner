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
   */
  void update(const vector<vector<double>>& sensor_fusion);

  void predict_trajectories(double prediction_horizon, int index = -1);

  /*
   * Returns the predicted position of a car at time t in Frenet coordinates.
   *
   * @param index The row index in the sensor fusion data table of the car for which the position is to be predicted.
   * @param k The number of 20 ms time steps from the present time for which the position is to be predicted.
   *
   * @returns A 3-tuple containing the car's predicted position in Frenet coordinates, (s, d), and the car's lane number.
   *          The left-most lane is lane 0, counting up towards the right-most lane. If the car is predicted to be in neither
   *          of the lanes, the value will be -1.
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
