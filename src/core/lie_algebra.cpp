#include "lie_algebra.h"
#include <iostream>

Eigen::Matrix4d SE3::exp(const Eigen::VectorXd& x) {
  //  std::cout << "exponential of " << x.transpose() << std::endl;
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();

  // see  phd thesis of Hauke Strasdat, 2012 pp 47-53:
  Eigen::Vector3d v(x[0], x[1], x[2]);
  Eigen::Vector3d omega(x[3], x[4], x[5]);

  double theta = omega.norm();
  if (theta > 1e-10) {
    Eigen::Matrix3d omega_skew;
    omega_skew << 0, -omega[2], omega[1], omega[2], 0, -omega[0], -omega[1], omega[0], 0;
    Eigen::Matrix3d omega_skew_sqr = omega_skew * omega_skew;

    // Rotation:
    double alpha = sin(theta) / theta;
    double beta = (1 - cos(theta)) / (theta * theta);
    result.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity() + alpha * omega_skew + beta * omega_skew_sqr;

    // Translation:
    double gamma = (1.0f - cos(theta)) / (theta * theta);
    double delta = (theta - sin(theta)) / (theta * theta * theta);

    Eigen::Matrix3d V = Eigen::Matrix3d::Identity() + gamma * omega_skew + delta * omega_skew_sqr;
    result.block<3, 1>(0, 3) = V * v;
  } else {
    result.block<3, 1>(0, 3) = v;  // just translation.
  }

  return result;
}

Eigen::VectorXd SE3::log(const Eigen::Matrix4d& M) {
  // see  PhD thesis of Hauke Strasdat, 2012 pp. 47-53 and Sophus library:

  Eigen::VectorXd x = Eigen::VectorXd::Zero(6);

  Eigen::Matrix3d R = M.topLeftCorner(3, 3);
  Eigen::Vector3d t = M.topRightCorner(3, 1);

  double d = 0.5 * (R.trace() - 1.0);
  Eigen::Matrix3d omega_skew = Eigen::Matrix3d::Zero();

  if (d < 1 - 1e-10) {
    double theta = std::acos(d);
    omega_skew = theta / (2 * std::sin(theta)) * (R - R.transpose());

    x[3] = omega_skew(2, 1);
    x[4] = omega_skew(0, 2);
    x[5] = omega_skew(1, 0);
  }

  double theta = x.tail(3).norm();

  x.head(3) = t;

  if (std::abs(theta) > 1e-10) {
    double half_theta = 0.5 * theta;

    double alpha = -0.5;
    double beta = 1 / (theta * theta) * (1 - theta * std::cos(half_theta) / (2 * std::sin(half_theta)));
    Eigen::Matrix3d V_inv = Eigen::Matrix3d::Identity() + alpha * omega_skew + beta * (omega_skew * omega_skew);

    x.head(3) = V_inv * t;
  }

  return x;
}

Eigen::Matrix4d SO3::exp(const Eigen::VectorXd& x) {
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();

  Eigen::Vector3d omega(x[0], x[1], x[2]);

  double theta = omega.norm();
  if (theta > 1e-10) {
    Eigen::Matrix3d omega_skew;
    omega_skew << 0, -omega[2], omega[1], omega[2], 0, -omega[0], -omega[1], omega[0], 0;
    Eigen::Matrix3d omega_skew_sqr = omega_skew * omega_skew;
    result.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity() + sin(theta) / theta * omega_skew +
                                 (1 - cos(theta)) / (theta * theta) * omega_skew_sqr;
  }

  return result;
}
