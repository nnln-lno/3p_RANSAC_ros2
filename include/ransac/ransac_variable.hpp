#ifndef RANSAC_VARIABLE_HPP
#define RANSAC_VARIABLE_HPP

#include <Eigen/Eigen>

namespace m_ransac{
    
    typedef Eigen::Matrix<double,3,3> Mat3d;
    typedef Eigen::Matrix<double,2,2> Mat2d;
    typedef Eigen::MatrixXd MatXd;
    typedef Eigen::VectorXd VecXd;
    
}
#endif