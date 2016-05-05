#ifndef INVDYNEIGENINTERFACE_HPP_
#define INVDYNEIGENINTERFACE_HPP_
#include "../IDConfig.hpp"
namespace btInverseDynamics {
#ifdef BT_USE_DOUBLE_PRECISION
typedef Eigen::VectorXd vecx;
typedef Eigen::Vector3d vec3;
typedef Eigen::Matrix3d mat33;
typedef Eigen::MatrixXd matxx;
#else
typedef Eigen::VectorXf vecx;
typedef Eigen::Vector3f vec3;
typedef Eigen::Matrix3f mat33;
typedef Eigen::MatrixXf matxx;
#endif  //
}
#endif  // INVDYNEIGENINTERFACE_HPP_
