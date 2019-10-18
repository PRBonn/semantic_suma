#include "core/LieGaussNewton.h"
#include <rv/PrimitiveParameters.h>
#include <rv/Stopwatch.h>

using namespace rv;

LieGaussNewton::LieGaussNewton() {
  params_.insert(IntegerParameter("max iterations", maxIter));
  params_.insert(FloatParameter("stopping threshold", epsilon));
  params_.insert(FloatParameter("delta", delta));
}

int32_t LieGaussNewton::minimize(Objective& F, const Eigen::Matrix4d& T0) {
  history_.clear();

  initialize(F, T0);

  std::vector<uint32_t> maxIterations({maxIter, maxIter, maxIter, 3, 3, 3});

  for (uint32_t i = 0; i <= F.getMaxLevel(); ++i) {
    F.setLevel(i);
    k_ = 0;
    for (;;) {
      history_.push_back(Tk_);
      /** check if we can stop here. **/

      if (maxIterations[i] > 0 && k_ >= maxIterations[i]) break;  // max iterations reached. :/

      int32_t result = step();
      if (result == 0) break;  // converged.

      ++k_;
    }
  }

  return 0;  // converged, hurray!
}

void LieGaussNewton::initialize(Objective& F, const Eigen::Matrix4d& T0) {
  const uint32_t N = F.num_parameters();

  JtJ = Eigen::MatrixXd(N, N);
  Jtf = Eigen::MatrixXd(N, 1);

  Tk_ = T0;
  objective_ = &F;
  objective_->initialize(T0);
  last_error = std::numeric_limits<float>::max();  // objective_->jacobianProducts(JtJ, Jtf);  // get the jacobians.

  if (callback_ != 0) (*callback_)(last_error, Eigen::VectorXd::Zero(N), Tk_);  // t = 0
}

int32_t LieGaussNewton::step() {
  assert(objective_ != nullptr);

  int32_t result = 1;

  double current_error = objective_->jacobianProducts(JtJ, Jtf);

  Eigen::VectorXd deltax = JtJ.ldlt().solve(-Jtf);  // new direction.

  assert(!std::isnan(deltax[0]));

  if (deltax.lpNorm<Eigen::Infinity>() < delta) result = 0;                                      // minimal change.
  if (std::abs(Jtf.maxCoeff()) < epsilon) result = 0;                                            // gradient.
  if (current_error < last_error && std::abs(current_error - last_error) < epsilon) result = 0;  // converged.

  objective_->increment(deltax);

  Tk_ = objective_->pose();
  last_error = current_error;
  covariance_ = JtJ;
  information_ = JtJ;
  covDirty_ = true;

  if (callback_ != nullptr) (*callback_)(current_error, deltax, Tk_);

  return result;
}

void LieGaussNewton::setParameters(const ParameterList& params) {
  for (ParameterList::const_iterator it = params.begin(); it != params.end(); ++it) {
    if (params_.hasParam(it->name())) {
      params_.insert(*it);
    }
  }

  maxIter = params_["max iterations"];
  epsilon = params_["stopping threshold"];
  delta = params_["delta"];
}

const Eigen::MatrixXd& LieGaussNewton::covariance() {
  if (covDirty_) {
    covariance_ = covariance_.lu().inverse();
    covDirty_ = false;
  }

  return covariance_;
}

const Eigen::MatrixXd& LieGaussNewton::information() {
  return information_;
}

double LieGaussNewton::residual() const {
  return last_error;
}

std::string LieGaussNewton::reason(int32_t errorno) const {
  if (errorno == -1) return "Maximum number of iterations reached.";
  if (errorno == -2) return "Diverging.";

  return "no error";
}

const Eigen::Matrix4d& LieGaussNewton::pose() const {
  return Tk_;
}

uint32_t LieGaussNewton::iterationCount() const {
  return k_;
}
