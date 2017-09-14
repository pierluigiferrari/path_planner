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

vector<double> jerk_minimizing_trajectory(vector<double> start, vector <double> end, double T);

#endif /* TRAJECTORY_GENERATOR_H_ */
