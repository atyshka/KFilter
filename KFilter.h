//
// Created by Alex Tyshka on 12/25/18.
//

#ifndef KF_LIBRARY_KFILTER_H
#define KF_LIBRARY_KFILTER_H
// Eigen headers moved in v3.3
#ifdef USE_NEW_EIGEN_HEADERS
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

class KFilter {
public:
    
    KFilter(Eigen::MatrixXd F,
            Eigen::VectorXd x,
            Eigen::MatrixXd P,
            Eigen::MatrixXd Q,
            Eigen::MatrixXd H,
            Eigen::MatrixXd R);
    KFilter(int state_dimension, int measurement_dimension);

    void predict();
    void update(Eigen::VectorXd z);
    
    void setInitialState(Eigen::VectorXd x);
    void setStateTransition(Eigen::MatrixXd F);
    void setCovariance(Eigen::MatrixXd P);
    void setMeasurementFunction(Eigen::MatrixXd H);
    void setMeasurementNoise(Eigen::MatrixXd R);
    void setProcessNoise(Eigen::MatrixXd Q);

    Eigen::VectorXd x() { return x_; }
private:
    Eigen::MatrixXd F_;
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;

    int measurement_dimension_;
    int state_dimension_;
};


#endif //KF_LIBRARY_KFILTER_H
