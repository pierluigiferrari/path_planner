/*
 * gaussian_naive_bayes.h
 *
 *  Created on: Sep 13, 2017
 *      Author: pierluigiferrari
 */

#ifndef GAUSSIAN_NAIVE_BAYES_H_
#define GAUSSIAN_NAIVE_BAYES_H_

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>

using namespace std;

class GNB {

public:

  /**
    * Constructor
    */
  GNB();

  /**
  * Destructor
  */
  virtual ~GNB();

  /*
   * Loads the training and test datasets.
   *
   */
  void set_up_training(string train_data, string train_labels);

  void set_up_evaluation(string test_data, string test_labels);

  /*
   * Trains the classifier with N data points and labels. "Training" consists of
   * computing the prior distribution of the labels and estimating the parameters of
   * the normal distributions of a feature given a label.
   */
  void train();

  /*
   * Returns the accuracy of the classifier on the test dataset.
   *
   * @param verbose If `true`, prints the accuracy of the classifier to standard output. Defaults to `false`.
   */
  float evaluate(bool verbose = false);

  /*
   * Predicts a label given a sample. You need to train the classifier before
   * making predictions.
   *
   * @param sample A 4-tuple containing the features s, d, s_dot, and d_dot, in this order.
   *
   * @returns The predicted label, which will be one of "left", "keep" or "right".
   */
  string predict(vector<double>);

private:

  // Store data and labels for training and test datasets

  // A 2-dimensional vector of shape (N, 4) containing N samples, each consisting of
  // the four features s, d, s_dot, and d_dot. These features are the Frenet
  // coordinates s and d and their first derivatives.
  vector<vector<double>> x_train_;
  vector<vector<double>> x_test_;
  // A vector containing the N labels. Each label is one of "left", "keep", or "right".
  vector<string> y_train_;
  vector<string> y_test_;

  // The list of classes that this classifier can distinguish
  vector<string> classes_ = {"left","keep","right"};

  // Means and variances for the Gaussian feature distributions by class
  vector<double> s_means_ = vector<double>(classes_.size());
  vector<double> s_variances_ = vector<double>(classes_.size());
  vector<double> s_dot_means_ = vector<double>(classes_.size());
  vector<double> s_dot_variances_ = vector<double>(classes_.size());
  vector<double> d_rel_means_ = vector<double>(classes_.size());
  vector<double> d_rel_variances_ = vector<double>(classes_.size());
  vector<double> d_dot_means_ = vector<double>(classes_.size());
  vector<double> d_dot_variances_ = vector<double>(classes_.size());

  // Prior distribution
  vector<double> prior_dist_ = vector<double>(classes_.size());

  /*
   * Loads the data for training from `file_name`.
   */
  vector<vector<double>> load_data(string file_name);

  /*
   * Loads the labels for training from `file_name`.
   */
  vector<string> load_labels(string file_name);

};

#endif /* GAUSSIAN_NAIVE_BAYES_H_ */
