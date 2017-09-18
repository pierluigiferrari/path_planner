/*
 * JMT_generator.h
 *
 *  Created on: Sep 17, 2017
 *      Author: pierluigiferrari
 */

#ifndef JMT_GENERATOR_H_
#define JMT_GENERATOR_H_

#include <vector>

using namespace std;

class JMT {

public:

  /*
   * Constructor
   */
  JMT();

  /*
   * Destructor
   */
  virtual ~JMT();

  /*
   * Computes the six coefficients for a 5th-degree polynomial
   * that constitutes the jerk-minimizing trajectory from the start state
   * to the end state in time T.
   *
   * @param start The initial state, a 3-tuple consisting of a state-describing
   *              coordinate and its first and second time derivatives.
   * @param end The desired final state, a 3-tuple consisting of a state-describing
   *            coordinate and its first and second time derivatives.
   * @param T The time in seconds allowed for the transition from the start state
   *          to the end state.
   */
  void compute_trajectory(vector<double> start, vector <double> end, double T);

  // Returns the trajectory point at time t
  double operator() (double t) const;

private:
  // Vector to store the coefficients of the fifth-order polynomial that constitutes the jerk-minimal trajectory.
  vector<double> a = vector<double>(6);
};

#endif /* JMT_GENERATOR_H_ */
