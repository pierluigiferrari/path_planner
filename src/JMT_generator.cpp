/*
 * JMT_generator.cpp
 *
 *  Created on: Sep 17, 2017
 *      Author: pierluigiferrari
 */

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include "Dense"
#include "JMT_generator.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

JMT::JMT() {}

JMT::~JMT() {}

void JMT::compute_trajectory(vector<double> start, vector <double> end, double T) {
    /*
     * The jerk-minimal trajectory will be a fifth-degree polynomial
     * (understand why: http://www.shadmehrlab.org/book/minimum_jerk/minimumjerk.htm)
     *
     * x(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
     *
     * with first and second derivatives
     *
     * x'(t) = a_1 + 2 * a_2 * t + 3 * a_3 * t**2 + 4  * a_4 * t**3 +  5 * a_5 * t**4
     * x"(t) =       2 * a_2     + 6 * a_3 * t    + 12 * a_4 * t**2 + 20 * a_5 * t**3
     *
     * We are trying to find the polynomial's six coefficients [a_0, a_1, ..., a_5]
     * given six boundary conditions.
     *
     * Let x(0), x'(0), x"(0) be the boundary conditions at time t = 0 and
     * let x(T), x'(T), x"(T) be the boundary conditions at time t = T.
     *
     * Then plugging in t = 0 above immediately yields
     *
     * a_0 =       x (0)
     * a_1 =       x'(0)
     * a_2 = 0.5 * x"(0)
     *
     * In order to get the remaining three coefficients, we need to solve the
     * linear system
     *
     * A * x = b, where
     *
     *     /   T**3     T**4     T**5 \
     * A = | 3*T**2   4*T**3   5*T**4 |
     *     \ 6*T     12*T**2  20*T**3 /
     *
     *     / x (T) - (x (0) + x'(0) * T + 0.5 * x"(0) * T**2) \
     * b = | x'(T) - (x'(0) + x"(0) * T)                      |
     *     \ x"(T) -  x"(0)                                   /
     *
     *     / a_3 \
     * x = | a_4 |
     *     \ a_5 /
     */

    VectorXd x(3);
    VectorXd b(3);
    MatrixXd A(3,3);

    // Store powers of T to avoid unnecessary computation.
    double t2 = T * T;
    double t3 = t2 * T;
    double t4 = t3 * T;
    double t5 = t4 * T;

    // Initialize A and b
    b << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * t2), end[1] - (start[1] + start[2] * T), end[2] - start[2];
    A <<   t3,    t4,    t5,
         3*t2,  4*t3,  5*t4,
         6*T , 12*t2, 20*t3;

    // Solve for x
    x = A.colPivHouseholderQr().solve(b);

    a[0] = start[0];
    a[1] = start[1];
    a[2] = 0.5 * start[2];
    a[3] = x(0);
    a[4] = x(1);
    a[5] = x(2);
}

double JMT::operator() (double t) const {

  double t2 = t * t;
  double t3 = t2 * t;
  double t4 = t3 * t;
  double t5 = t4 * t;

  // Evaluate the polynomial at time t
  return a[0] + a[1] * t + a[2] * t2 + a[3] * t3 + a[4] * t4 + a[5] * t5;

}
