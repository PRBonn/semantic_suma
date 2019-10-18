#ifndef INCLUDE_CORE_PROJECTIVEICP_H_
#define INCLUDE_CORE_PROJECTIVEICP_H_

#include "Frame.h"
#include "lie_algebra.h"

#include <rv/Parameter.h>

#include <memory>

/**
 *  \author behley
 **/
class Objective {
 public:
  virtual ~Objective() {}

  virtual uint32_t num_parameters() const = 0;

  /** \brief set single parameter to specific value. **/
  virtual void setParameter(const rv::Parameter& param) {}

  /** \brief Current and last frame.
   *
   *  \param current data from current point cloud (the so-called "live" frame)
   *  \param last date from previous point cloud or data generated from model. (the so-called "model" frame)
   *
   *  At least one is implemented by the derived classes.
   **/
  virtual void setData(const std::shared_ptr<Frame>& current, const std::shared_ptr<Frame>& last) {
    throw std::runtime_error("ProjectiveICP::setData not implemented.");
  }

  /** \brief compute residual with given increment.
   *
   *  Beware:  naive implementation using jacobianProducts; should be overwritten by derived
   *  classes for maximal performance
   */
  virtual double residual(const Eigen::VectorXd& delta) = 0;

  /** \brief compute JtJ and Jtf exploiting intermediate computations at current pose, return F(x).  */
  virtual double jacobianProducts(Eigen::MatrixXd& JtJ, Eigen::MatrixXd& Jtf) = 0;

  /** \brief increment by given delta. **/
  void increment(const Eigen::VectorXd& delta) {
    pose_ = SE3::exp(delta) * pose_;
    iteration_ += 1;
  }

  uint32_t inlier() const { return inlier_; }
  uint32_t outlier() const { return outlier_; }
  uint32_t valid() const { return (inlier_ + outlier_); }
  uint32_t invalid() const { return invalid_; }

  float inlier_residual() const { return inlier_residual_; }

  /** \brief initialize the pose estimate. **/
  void initialize(const Eigen::Matrix4d& T0) { pose_ = T0; }

  const Eigen::Matrix4d& pose() const { return pose_; }

  /** \brief specify at which level the redisual/jacobian is computed. **/
  virtual void setLevel(uint32_t lvl) {  // noop
  }

  /** \brief get maximum level (coarse-to-fine). **/
  virtual uint32_t getMaxLevel() const { return 0; }

  virtual void reset() { /* noop */
  }

 protected:
  // can only be instanced by derived classes.
  Objective() {}

  Eigen::Matrix4d pose_{Eigen::Matrix4d::Identity()};
  uint32_t iteration_{0};
  uint32_t inlier_{0};
  float inlier_residual_{0.0f};
  uint32_t outlier_{0};
  uint32_t invalid_{0};
};

#endif /* INCLUDE_CORE_PROJECTIVEICP_H_ */
