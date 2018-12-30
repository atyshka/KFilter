//
// Created by Alex Tyshka on 12/25/18.
//

#include "KFilter.h"

KFilter::KFilter() {

}



void KFilter::predict() {
    // Predict future state and covariance from current state
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KFilter::update(Eigen::VectorXd z) {
    // Projecting covariance from state space to measurement space
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    // Compute Kalman Gain, residual, and update state accordingly
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    Eigen::VectorXd y = z - H_ * x_;
    x_ += K * y;
    // Update Covariances
    P_ = P_ - (K * H_ * P_);
}