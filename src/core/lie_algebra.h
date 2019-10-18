#ifndef INCLUDE_CORE_LIE_ALGEBRA_H_
#define INCLUDE_CORE_LIE_ALGEBRA_H_

#include <eigen3/Eigen/Dense>

/** \brief some utility functions for Lie groups.
 *
 *  \author behley
 **/

class SE3 {
 public:
  SE3() = delete;

  /** \brief get rotation matrix from angle-axis + translation **/
  static Eigen::Matrix4d exp(const Eigen::VectorXd& x);

  /** \brief get angle-axis + translation from rotation matrix **/
  static Eigen::VectorXd log(const Eigen::Matrix4d& x);
};

class SO3 {
 public:
  SO3() = delete;
  static Eigen::Matrix4d exp(const Eigen::VectorXd& x);
};

#endif /* INCLUDE_CORE_LIE_ALGEBRA_H_ */
