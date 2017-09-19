/*
 * helper_functions.h
 *
 *  Created on: Sep 17, 2017
 *      Author: pierluigiferrari
 */

#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <math.h>
#include <vector>

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

double mph2mps(double x);
double mps2mph(double x);

double l2_distance(double x1, double y1, double x2, double y2);

int closest_waypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);

int next_waypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> get_frenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

// Transform from Frenet (s, d) coordinates to Cartesian (x, y)
vector<double> get_xy(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

vector<double> get_frenet_velocity_vxvy(double x, double y, double vx, double vy, vector<double> maps_x, vector<double> maps_y);

vector<double> get_frenet_velocity_theta(double x, double y, double theta, double speed, vector<double> maps_x, vector<double> maps_y);

#endif /* HELPER_FUNCTIONS_H_ */
