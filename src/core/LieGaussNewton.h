#ifndef INCLUDE_CORE_LIEGAUSSNEWTON_H_
#define INCLUDE_CORE_LIEGAUSSNEWTON_H_

#include <rv/ParameterList.h>
#include "Objective.h"

#include <memory>

/** \brief Gauss-Newton method on Lie Groups, which performs the update using matrix exponentials.
 *
 *  Based on Gauss Newton method, but using poses and pose increments for representation of state.
 *  The actual implementation of the pose increments in handled by the objective. Therefore the
 *  optimizer only produces increments, which are then integrated by the objective.
 *
 *  \author behley
 **/

class OptimizerCallback {
 public:
  virtual ~OptimizerCallback() {}

  virtual void operator()(float residual, const Eigen::VectorXd& x, const Eigen::Matrix4d& pose) = 0;
};

class LieGaussNewton {
 public:
  LieGaussNewton();

  void setParameters(const rv::ParameterList& params);

  /** \brief optimize F starting with given pose T0. **/
  int32_t minimize(Objective& F, const Eigen::Matrix4d& T0);

  /** \brief initialize given objective and pose... **/
  void initialize(Objective& F, const Eigen::Matrix4d& T0);

  /** \brief perform a single step with the initialized objective. **/
  int32_t step();

  /** \brief get last residual. **/
  double residual() const;

  /** \brief get best pose. **/
  const Eigen::Matrix4d& pose() const;

  std::string reason(int32_t errorno) const;

  const Eigen::MatrixXd& covariance();

  const Eigen::MatrixXd& information();

  uint32_t iterationCount() const;

  // some constants.
  static const int32_t CONVERGED{0};

  const std::vector<Eigen::Matrix4d>& history() const { return history_; }

 protected:
  rv::ParameterList params_;
  double last_error{134567.00};
  uint32_t k_{0};
  Eigen::Matrix4d Tk_;
  Eigen::MatrixXd information_;
  Eigen::MatrixXd covariance_;
  bool covDirty_{true};
  Objective* objective_{nullptr};

  Eigen::MatrixXd JtJ, Jtf;

  uint32_t maxIter{200};
  double epsilon{1e-10}, delta{1e-10};

  std::vector<Eigen::Matrix4d> history_;
  std::shared_ptr<OptimizerCallback> callback_;
};

#endif /* INCLUDE_CORE_LIEGAUSSNEWTON_H_ */
