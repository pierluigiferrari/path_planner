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

using namespace std;

class predictor
{
public:

  /*
   * Default constructor
   */
  predictor();

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
  predictor(vector<vector<double>> sensor_fusion);

  /*
   * Destructor
   */
  virtual ~predictor();

  /*
   * Returns the predicted position of a car at time t in Frenet coordinates.
   *
   * @param index The row index in the sensor fusion data table of the car for which the position is to be predicted.
   * @param t The time in seconds from the present time for which the position is to be predicted.
   *
   * @returns A 3-tuple containing the car's predicted position in Frenet coordinates, (s, d), and the car's lane number.
   *          The left-most lane is lane 0, counting up towards the right-most lane. If the car is predicted to be in neither
   *          of the lanes, the value will be -1.
   */
  vector<double> position(int index, double t);

  /*
   * Returns the predicted absolute velocity of a car at time t.
   *
   * @param index The row index in the sensor fusion data table of the car for which the velocity is to be predicted.
   *
   * @returns The car's predicted absolute velocity.
   */
  double velocity(int index);

private:
  // The sensor fusion vector passed in the constructor.
  vector<vector<double>> sensor_fusion_;
};

#endif /* PREDICTOR_H_ */
