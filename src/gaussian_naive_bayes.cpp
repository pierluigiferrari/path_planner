/*
 * naive_bayes_predictor.cpp
 *
 *  Created on: Sep 13, 2017
 *      Author: pierluigiferrari
 */

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <algorithm>
#include "gaussian_naive_bayes.h"

using namespace std;

// Helper functions

/*
 * Computes the probability density function for the 1-dimensional normal distribution.
 *
 * @param sample The sample value for which the PDF is to be evaluated.
 */
double gaussian(double sample, double mean, double variance) {
  return (1 / sqrt(2 * M_PI * variance)) * exp(-0.5 * pow(sample - mean, 2) / variance);
}

/*
 * Computes the probability density function for the multivariate normal distribution with
 * independent features, i.e. all covariances are zero.
 *
 * This helper function is currently not being used.
 *
 * @param sample The N-dimensional sample vector for which the PDF is to be evaluated.
 * @param mean The N-dimensional mean vector of the normal distribution.
 * @param variance A vector containing the N variances of the normal distribution. This argument
 *                 replaces the covariance matrix for the general multivariate case since the
 *                 features are assumed to be independent here.
 */
double multivariate_gaussian(vector<double> sample, vector<double> mean, vector<double> variance) {
  int dim = sample.size(); // dimensionality
  double det_sigma = 1.0; // determinant of the covariance matrix
  double ex = 0.0; // exponent
  for (int i = 0; i < dim; i++) {
    det_sigma *= variance[i];
    ex += pow((sample[i] - mean[i]) / variance[i], 2);
  }
  double c = 1.0 / sqrt(pow(2 * M_PI, dim) * det_sigma); // the constant of the gaussian distribution

  return c * exp(-0.5 * ex);
}

// Class implementation

GNB::GNB() {}

GNB::~GNB() {}

vector<vector<double> > GNB::load_data(string file_name) {

  ifstream in_data_(file_name.c_str(), ifstream::in);
  vector< vector<double >> data_out;
  string line;

  while (getline(in_data_, line)) {

    istringstream iss(line);
    vector<double> x_coord;

    string token;
    while(getline(iss, token, ',')) {
      x_coord.push_back(stod(token));
    }
    data_out.push_back(x_coord);
  }

  return data_out;
}

vector<string> GNB::load_labels(string file_name) {

  ifstream in_labels_(file_name.c_str(), ifstream::in);
  vector< string > labels_out;
  string line;
  while (getline(in_labels_, line)) {

    istringstream iss(line);
    string label;
    iss >> label;

    labels_out.push_back(label);
  }

  return labels_out;
}

void GNB::set_up_training(string train_data, string train_labels, string test_data, string test_labels) {
  x_train = load_data(train_data);
  x_test = load_data(test_data);
  y_train  = load_labels(train_labels);
  y_test  = load_labels(test_labels);
}

void GNB::train() {

  // Store the number of samples of each class in the dataset
  vector<int> n_samples(classes.size());

  // Compute the mean of each feature per class
  for (int i = 0; i < x_train.size(); i++) {

    double s     =      x_train[i][0];
    double d_rel = fmod(x_train[i][1], 4.0); // Compute d relative to the lane that the object is in. Each lane is 4 meters wide. This is a more useful feature for the classifier than an objects absolute d value.
    double s_dot =      x_train[i][2];
    double d_dot =      x_train[i][3];

    if (y_train[i] == classes[0]) {
      s_means[0] += s;
      d_rel_means[0] += d_rel;
      s_dot_means[0] += s_dot;
      d_dot_means[0] += d_dot;
      n_samples[0] += 1;
    }
    else if (y_train[i] == classes[1]) {
      s_means[1] += s;
      d_rel_means[1] += d_rel;
      s_dot_means[1] += s_dot;
      d_dot_means[1] += d_dot;
      n_samples[1] += 1;
    }
    else if (y_train[i] == classes[2]) {
      s_means[2] += s;
      d_rel_means[2] += d_rel;
      s_dot_means[2] += s_dot;
      d_dot_means[2] += d_dot;
      n_samples[2] += 1;
    }
  }
  for (int i = 0; i < classes.size(); i++) {
    s_means[i]     /= n_samples[i];
    d_rel_means[i] /= n_samples[i];
    s_dot_means[i] /= n_samples[i];
    d_dot_means[i] /= n_samples[i];
  }

  // Compute the variances of each feature per class
  for (int i = 0; i < x_train.size(); i++) {

    double s     =      x_train[i][0];
    double d_rel = fmod(x_train[i][1], 4.0); // Compute d relative to the lane that the object is in. Each lane is 4 meters wide. This is a more useful feature for the classifier than an objects absolute d value.
    double s_dot =      x_train[i][2];
    double d_dot =      x_train[i][3];

    if (y_train[i] == classes[0]) {
      s_variances[0]     += pow(s     - s_means[0],     2);
      d_rel_variances[0] += pow(d_rel - d_rel_means[0], 2);
      s_dot_variances[0] += pow(s_dot - s_dot_means[0], 2);
      d_dot_variances[0] += pow(d_dot - d_dot_means[0], 2);
    }
    else if (y_train[i] == classes[1]) {
      s_variances[1]     += pow(s     - s_means[1],     2);
      d_rel_variances[1] += pow(d_rel - d_rel_means[1], 2);
      s_dot_variances[1] += pow(s_dot - s_dot_means[1], 2);
      d_dot_variances[1] += pow(d_dot - d_dot_means[1], 2);
    }
    else if (y_train[i] == classes[2]) {
      s_variances[2]     += pow(s     - s_means[2],     2);
      d_rel_variances[2] += pow(d_rel - d_rel_means[2], 2);
      s_dot_variances[2] += pow(s_dot - s_dot_means[2], 2);
      d_dot_variances[2] += pow(d_dot - d_dot_means[2], 2);
    }
  }
  for (int i = 0; i < classes.size(); i++) {
    s_variances[i]     /= n_samples[i];
    d_rel_variances[i] /= n_samples[i];
    s_dot_variances[i] /= n_samples[i];
    d_dot_variances[i] /= n_samples[i];
  }

  // Compute the prior distribution P(C) of the classes
  for (int i = 0; i < classes.size(); i++) {
    prior_dist[i] = (double) n_samples[i] / x_train.size();
  }
}

float GNB::test(bool verbose) {

  int score = 0;

  for(int i = 0; i < x_test.size(); i++) {

    vector<double> sample = x_test[i];
    string prediction = predict(sample);

    if(prediction.compare(y_test[i]) == 0) {
      score += 1;
    }
  }

  float fraction_correct = float(score) / y_test.size();

  if (verbose) cout << "Accuracy on the test dataset: " << (100*fraction_correct) << " %." << endl;

  return fraction_correct;
}

string GNB::predict(vector<double> sample) {

  // Convert the sample feature 'd' into 'd_rel' as described in `train()`.
  sample[1] = fmod(sample[1], 4.0);

  // Store the probabilities of the classes given the sample here: P(C | x)
  vector<double> probabilities(classes.size());

  for (int i = 0; i < classes.size(); i++) {
    // Compute P(x(i) | C) for this class C for each individual feature x(i)
    double s_likelihood     = gaussian(sample[0], s_means[i],     s_variances[i]);
    double d_rel_likelihood = gaussian(sample[1], d_rel_means[i], d_rel_variances[i]);
    double s_dot_likelihood = gaussian(sample[2], s_dot_means[i], s_dot_variances[i]);
    double d_dot_likelihood = gaussian(sample[3], d_dot_means[i], d_dot_variances[i]);
    // Compute the probability P(C | x) = P(C) * P(x(0) | C) * P(x(1) | C) * P(x(2) | C) * P(x(3) | C)
    probabilities[i] = prior_dist[i] * s_likelihood * d_rel_likelihood * s_dot_likelihood * d_dot_likelihood;
  }

  // Take the maximum probability P(C | x) and return it as the prediction
  int arg_max = distance(probabilities.begin(), max_element(probabilities.begin(), probabilities.end()));

  return classes[arg_max];
}
