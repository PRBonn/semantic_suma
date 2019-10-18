#include "Posegraph.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using namespace gtsam;

Posegraph::Posegraph() {}

void Posegraph::clear() {
  graph_ = NonlinearFactorGraph();
  initial_.clear();
  edges_.clear();
  result_.clear();
}

Posegraph::Ptr Posegraph::clone() const {
  return Posegraph::Ptr(new Posegraph(*this));
}

void Posegraph::setInitial(int32_t id, const Eigen::Matrix4d& initial_estimate) {
  bool addPrior = initial_.empty();

  //  std::cout << "Initializing " << id << std::endl;

  if (!initial_.exists(id))
    initial_.insert(id, Pose3(initial_estimate));
  else
    initial_.update(id, Pose3(initial_estimate));

  // also update intermediate result.
  if (result_.exists(id))
    result_.update(id, Pose3(initial_estimate));
  else
    result_.insert(id, Pose3(initial_estimate));

  if (addPrior) {
    Vector6 diagonal;
    diagonal << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
    noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Variances(diagonal);
    graph_.add(PriorFactor<Pose3>(id, Pose3(), priorModel));
  }
}

void Posegraph::addEdge(int32_t from, int32_t to, const Eigen::Matrix4d& measurement,
                             const Matrix6d& information) {
  SharedNoiseModel model = noiseModel::Gaussian::Information(information);
  if (robustify_) {
    model = noiseModel::Robust::Create(robustifier_, model);
  }

  NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(from, to, Pose3(measurement), model));
  graph_.add(factor);

  edges_.push_back({from, to, measurement});
}

Eigen::Matrix4d Posegraph::pose(int32_t id) const {
  return result_.at<Pose3>(id).matrix();
}

std::vector<Eigen::Matrix4d> Posegraph::poses() const {
  std::vector<Eigen::Matrix4d> poses(result_.size());

  for (const auto& pair : result_) {
    if (pair.key >= poses.size()) continue;

    poses[pair.key] = pair.value.cast<Pose3>().matrix();
  }

  return poses;
}

int32_t Posegraph::size() const {
  return result_.size();
}

void Posegraph::reinitialize() {
  result_ = initial_;
}

double Posegraph::error() const {
  return graph_.error(result_);
}

bool Posegraph::optimize(uint32_t num_iters) {
  // todo: check if every value has been initialized.

  LevenbergMarquardtParams params;
  params.maxIterations = num_iters;
  params.setVerbosity("TERMINATION");  // this will show info about stopping conditions
  params.setVerbosity("SILENT");
  LevenbergMarquardtOptimizer optimizer(graph_, result_, params);

  //  std::cout << ">>  initial error = " << graph_.error(result_) << std::endl;
  result_ = optimizer.optimize();
  //  std::cout << ">>  final error = " << graph_.error(result_) << std::endl;

  return true;
}

const gtsam::NonlinearFactorGraph& Posegraph::graph() const {
  return graph_;
}

const gtsam::Values& Posegraph::initial() const {
  return initial_;
}

const gtsam::Values& Posegraph::result() const {
  return result_;
}

void Posegraph::save(const std::string& filename) const {}
void Posegraph::load(const std::string& filename) {}

void Posegraph::setMEstimator(const gtsam::noiseModel::mEstimator::Base::shared_ptr& m_estimator) {
  robustify_ = (m_estimator != nullptr);
  robustifier_ = m_estimator;
}
