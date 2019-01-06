//
// Created by Alex Tyshka on 12/25/18.
//

#include "KFilter.h"
#include <exception>
using namespace Eigen;


KFilter::KFilter(int state_dimension, int measurement_dimension) : 
    state_dimension_(state_dimension), 
    measurement_dimension_(measurement_dimension)
{
    // Initialize all matrices to correct size and default values
    x_ = VectorXd::Zero(state_dimension);
    F_ = MatrixXd::Identity(state_dimension, state_dimension);
    P_ = MatrixXd::Identity(state_dimension, state_dimension);
    Q_ = MatrixXd::Identity(state_dimension, state_dimension);
    H_ = MatrixXd::Identity(state_dimension, measurement_dimension);
    R_ = MatrixXd::Identity(measurement_dimension, measurement_dimension);
}

KFilter::KFilter(MatrixXd F,
                 VectorXd x,
                 MatrixXd P,
                 MatrixXd Q,
                 MatrixXd H,
                 MatrixXd R) : F_(F), x_(x), P_(P), Q_(Q), H_(H), R_(R)
{
}

void KFilter::predict() 
{
    // Predict future state and covariance from current state
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KFilter::update(VectorXd z) 
{
    // Projecting covariance from state space to measurement space
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    // Compute Kalman Gain, residual, and update state accordingly
    MatrixXd K = P_ * H_.transpose() * S.inverse();
    VectorXd y = z - H_ * x_;
    x_ += K * y;
    // Update Covariances
    P_ = P_ - (K * H_ * P_);
}

