#include "KFilter.h"
#include "iostream"
using namespace Eigen;

int main(int argc, char const *argv[])
{
    KFilter filter;
    Matrix2d F;
    F << 1, 0.1,
         0, 1;
    Vector2d x(10.0, 4.5);
    Matrix2d P;
    P << 500, 0,
         0,   49;
    RowVector2d H(1.0, 0.0);
    MatrixXd R(1, 1);
    R << 1;
    Matrix2d Q;
    Q << 0.0, 0.0,
         0.0, 0.1;

    filter.F_ = F;
    filter.x_ = x;
    filter.P_ = P;
    filter.H_ = H;
    filter.R_ = R;
    filter.Q_ = Q;
    filter.predict();
    std::cout << filter.x_ << std::endl;
    Eigen::VectorXd measurement(1, 1);
    measurement << 10.3;
    filter.update(measurement);
    std::cout << filter.x_ << std::endl;
    return 0;
}
