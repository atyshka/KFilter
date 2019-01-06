#include "KFilter.h"
#include "iostream"
using namespace Eigen;

int main(int argc, char const *argv[])
{
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

    KFilter filter(F, x, P, Q, H, R);
    std::cout << filter.x() << std::endl;
    filter.predict();
    std::cout << filter.x() << std::endl;
    Eigen::VectorXd measurement(1, 1);
    measurement << 10.3;
    filter.update(measurement);
    std::cout << filter.x() << std::endl;
    return 0;
}
