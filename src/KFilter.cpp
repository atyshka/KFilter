//
// Created by Alex Tyshka on 12/25/18.
//

#include "KFilter.h"
#include "Exceptions.h"
using namespace Eigen;


KFilter::KFilter(int state_dimension, int measurement_dimension) : 
    state_dimension_(state_dimension), 
    measurement_dimension_(measurement_dimension)
{
    // Initialize all matrices to correct size and default values
    // We'll directly set since we know these dimensions are valid
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
                 MatrixXd R)
{
    state_dimension_ = x.rows();
    measurement_dimension_ = R.rows();
    // Use these functions instead of direct set so we can check dimensions
    setStateTransition(F);
    setInitialState(x);
    setCovariance(P);
    setProcessNoise(Q);
    setMeasurementFunction(H);
    setMeasurementNoise(R);
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

void KFilter::setInitialState(Eigen::VectorXd x)
{
    if (x.rows() == state_dimension_) {
        x_ = x;
    } else {
        throw(InvalidDimensionException());
    }
}

void KFilter::setStateTransition(Eigen::MatrixXd F)
{
    if (F.rows() == state_dimension_ && F.cols() == state_dimension_) {
        F_ = F;
    } else {
        throw(InvalidDimensionException());
    }
}
void KFilter::setCovariance(Eigen::MatrixXd P)
{
    if (P.rows() == state_dimension_ && P.cols() == state_dimension_) {
        P_ = P;
    } else {
        throw(InvalidDimensionException());
    }
}

void KFilter::setMeasurementFunction(Eigen::MatrixXd H)
{
    if (H.rows() == measurement_dimension_ && H.cols() == state_dimension_) {
        H_ = H;
    } else {
        throw(InvalidDimensionException());
    }
}

void KFilter::setMeasurementNoise(Eigen::MatrixXd R)
{
    if (R.rows() == measurement_dimension_ && R.cols() == measurement_dimension_) {
        R_ = R;
    } else {
        throw(InvalidDimensionException());
    }
}

void KFilter::setProcessNoise(Eigen::MatrixXd Q)
{
    if (Q.rows() == state_dimension_ && Q.cols() == state_dimension_) {
        Q_ = Q;
    } else {
        throw(InvalidDimensionException());
    }
}