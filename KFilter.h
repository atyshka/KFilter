//
// Created by Alex Tyshka on 12/25/18.
//

#ifndef KF_LIBRARY_KFILTER_H
#define KF_LIBRARY_KFILTER_H

#ifdef USE_NEW_EIGEN_HEADERS
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

class KFilter {
public:
    KFilter();
    KFilter(int state_dimension, int measurement_dimension);

    void predict();
    void update(Eigen::VectorXd z);

    Eigen::MatrixXd F_;
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;
private:
    
};


#endif //KF_LIBRARY_KFILTER_H
