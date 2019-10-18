#include <core/SurfelMapping.h>
#include "core/Frame2Model.h"

#include <rv/PrimitiveParameters.h>

#include <fstream>
#include <sstream>

#include <glow/GlState.h>

#include <rv/Stopwatch.h>
#include <memory>

#include <gtsam/linear/NoiseModel.h>

using namespace rv;
using namespace glow;

SurfelMapping::SurfelMapping(const rv::ParameterList& params)
    : preprocessor_(params),
      width_(params["data_width"]),
      height_(params["data_height"]),
      lastFrame_(new Frame(width_, height_)),
      currentFrame_(new Frame(width_, height_)),
      intermediateFrame_(new Frame(width_, height_)),
      map_(new SurfelMap(params)),
      currentModelFrame_(new Frame(params["model_width"], params["model_height"])),
      lastModelFrame_(new Frame(params["model_width"], params["model_height"])),
      lastLoopClosureFrame_(new Frame(params["model_width"], params["model_height"])),
      confidence_threshold_(10.0f)
{
  currentPose_ = Eigen::Matrix4d::Identity();
  lastPose_ = Eigen::Matrix4d::Identity();

  lastFrame_->map = map_;
  currentModelFrame_->map = map_;
  lastModelFrame_->map = map_;
  currentFrame_->map = map_;

  setParameters(params);

  posegraph_ = std::shared_ptr<Posegraph>(new Posegraph());
  posegraph_->setInitial(0, Eigen::Matrix4d::Identity());  // dummy key for first frame.
  trajectory_distances_.push_back(0);
  currentlyLoopClosing_ = false;
  currentPose_new_ = currentPose_old_ = Eigen::Matrix4d::Identity();
  loopCount_ = 0;

  auto& diag = info_.diagonal();
  // translational noise is smaller than rotational noise.
  double transNoise = 1.0;
  double rotNoise = 1.0;
  diag[0] = (transNoise * transNoise);
  diag[1] = (transNoise * transNoise);
  diag[2] = (transNoise * transNoise);

  diag[3] = (rotNoise * rotNoise);
  diag[4] = (rotNoise * rotNoise);
  diag[5] = (rotNoise * rotNoise);

  statistics_["additional-icp-time"] = 0.0;
}

SurfelMapping::~SurfelMapping() {
  if (optimizeFuture_.valid()) optimizeFuture_.wait();  // wait for finishing.
}

void SurfelMapping::setParameters(const rv::ParameterList& const_params) {
  rv::ParameterList params = const_params;

  gn_ = std::shared_ptr<LieGaussNewton>(new LieGaussNewton);

  fallbackMode_ = params["fallback_mode"];
  std::string approach = std::string(params["approach"]);

  preprocessor_.setParameters(params);

  objective_.reset(new Frame2Model(params));

  if (approach == "frame-to-frame") {
    std::cout << "Performing frame-to-frame matching." << std::endl;
    performMapping_ = false;

  } else if (approach == "frame-to-model") {
    std::cout << "Performing frame-to-model matching." << std::endl;
    performMapping_ = true;
  }

  if (fallbackMode_) {
    ParameterList fallback_params = params;

    fallback_params.insert(FloatParameter("icp-max-distance", double(params["fallback-max-distance"])));
    fallback_params.insert(FloatParameter("icp-max-angle", double(params["fallback-max-angle"])));

    recovery_.reset(new Frame2Model(fallback_params));
  }

  if (objective_ == nullptr) throw std::runtime_error("unknown projective ICP implementation.");

  initialize_identity_ = params["initialize_identity"];

  init_factor_ = 1.0f;
  if (params.hasParam("init_factor")) init_factor_ = params["init_factor"];

  gn_->setParameters(params);
  map_->setParameters(params);

  float p_unstable = 0.1f;
  log_unstable_ = std::log(p_unstable / (1.0f - p_unstable));

  if (params.hasParam("confidence_threshold")) {
    confidence_threshold_ = params["confidence_threshold"];
  }

  if (params.hasParam("loop-residual-threshold")) loopResidualThres_ = params["loop-residual-threshold"];
  if (params.hasParam("loop-outlier-threshold")) loopOutlierThres_ = params["loop-outlier-threshold"];
  if (params.hasParam("loop-valid-threshold")) loopValidThres_ = params["loop-valid-threshold"];
  if (params.hasParam("close-loops")) makeLoopClosures_ = params["close-loops"];
  if (params.hasParam("loop-search-distance")) loopClosureSearchDist_ = params["loop-search-distance"];
  if (params.hasParam("perform-mapping")) performMapping_ = params["perform-mapping"];
  if (params.hasParam("loop-min-verifications")) loopMinNumberVerifications_ = params["loop-min-verifications"];
  if (params.hasParam("loop-min-trajectory-distance")) loopClosureMinTrajDist_ = params["loop-min-trajectory-distance"];

  statistics_["loopOutlierThres"] = loopOutlierThres_;
  statistics_["loopResidualThres"] = loopResidualThres_;
  statistics_["validThres"] = loopValidThres_;

  params_ = params;
}

void SurfelMapping::reset() {
  timestamp_ = 0;
  lastFrame_ = std::make_shared<Frame>(width_, height_);
  currentFrame_ = std::make_shared<Frame>(width_, height_);
  intermediateFrame_ = std::make_shared<Frame>(width_, height_);

  uint32_t mwidth = currentModelFrame_->width;
  uint32_t mheight = currentModelFrame_->height;

  currentModelFrame_ = std::make_shared<Frame>(mwidth, mheight);
  lastModelFrame_ = std::make_shared<Frame>(mwidth, mheight);

  lastPose_ = Eigen::Matrix4d::Identity();
  currentPose_ = Eigen::Matrix4d::Identity();

  map_->reset();
  T0 = Eigen::Matrix4d::Identity();

  lastFrame_->map = map_;
  currentFrame_->map = map_;
  currentModelFrame_->map = map_;
  lastModelFrame_->map = map_;

  trackLoss_ = 0;
  firstTime_ = true;
  lastIncrement_ = Eigen::Matrix4d::Identity();

  posegraph_->clear();
  posegraph_->setInitial(0, Eigen::Matrix4d::Identity());
  trajectory_distances_.clear();
  trajectory_distances_.push_back(0);
  currentlyLoopClosing_ = false;
  currentPose_new_ = currentPose_old_ = Eigen::Matrix4d::Identity();
  loopCount_ = 0;

  statistics_["additional-icp-time"] = 0.0;

  objective_->reset();  // FIXME: not needed.
}

uint32_t SurfelMapping::timestamp() const {
  return timestamp_;
}

void SurfelMapping::processScan(const rv::Laserscan& scan) {
  Stopwatch::tic();

  // check if optimization ready, copy poses, reinitialize loop closure count.
  if (makeLoopClosures_ && performMapping_) integrateLoopClosures();

  Stopwatch::tic();
  initialize(scan);
  statistics_["initialize-time"] = Stopwatch::toc();

  Stopwatch::tic();
  preprocess();
  statistics_["preprocessing-time"] = Stopwatch::toc();

  //  double icpTime = 0.0;
  if (timestamp_ > 0) {
    Stopwatch::tic();
    updatePose();
    statistics_["icp-time"] = Stopwatch::toc();

    Stopwatch::tic();
    if (makeLoopClosures_ && performMapping_) checkLoopClosure();
    statistics_["loop-time"] = Stopwatch::toc();
  }

  Stopwatch::tic();
  if (performMapping_) updateMap();
  statistics_["mapping-time"] = Stopwatch::toc();

  float completeTime = Stopwatch::toc();

  statistics_["complete-time"] = completeTime;
  statistics_["icp_percentage"] = statistics_["opt-time"] / completeTime;

  timestamp_ += 1;
}

void SurfelMapping::integrateLoopClosures() {
  if (currentlyOptimizing_) {
    if (optimizeFuture_.wait_for(std::chrono::milliseconds(5)) == std::future_status::ready) {
      std::vector<Eigen::Matrix4f> casted_poses;
      const std::vector<Eigen::Matrix4d>& poses_opt = optimizedPosegraph_->poses();
      std::vector<Eigen::Matrix4d> poses_before = posegraph_->poses();

      for (uint32_t i = 0; i < poses_opt.size(); ++i) {
        casted_poses.push_back(poses_opt[i].cast<float>());
        posegraph_->setInitial(i, poses_opt[i]);
      }

      loopCount_ -= beforeLoopCount_;
      Eigen::Matrix4d difference = poses_opt[beforeID_] * beforeOptimizationPose_.inverse();

      for (uint32_t i = poses_opt.size(); i < poses_before.size(); ++i) {
        poses_before[i] = difference * poses_before[i];
        casted_poses.push_back(poses_before[i].cast<float>());
        posegraph_->setInitial(i, poses_before[i]);
      }

      map_->updatePoses(casted_poses);

      currentlyOptimizing_ = false;
      currentPose_ = difference * currentPose_;

      // reset.
      currentlyLoopClosing_ = false;
      currentPose_new_ = currentPose_old_ = currentPose_;

      trajectory_distances_.resize(posegraph_->size());
      // update trajectory distances.
      Eigen::Vector4d last = posegraph_->pose(0).col(3);
      float distance = 0;
      for (int32_t t = 0; t < posegraph_->size(); ++t) {
        distance += (last - posegraph_->pose(t).col(3)).norm();
        trajectory_distances_[t] = distance;
        last = posegraph_->pose(t).col(3);
      }
    }
  }
}

void SurfelMapping::genResidualPlot() {
  std::cout << "called genResidualPlot" << std::endl;
  if (useLoopClosureCandidate_) {
    std::cout << "Generating plot: " << std::endl;

    int32_t to = lastAddedLoopClosureCandidate_;
    int32_t from = timestamp_ - 1;

    Eigen::MatrixXd JtJ = Eigen::MatrixXd(6, 6);
    Eigen::MatrixXd Jtr = Eigen::MatrixXd(6, 1);

    Eigen::Matrix4d new_pose = posegraph_->pose(from);
    Eigen::Matrix4d old_pose = posegraph_->pose(to);
    Eigen::Matrix4d diff_pose = old_pose.inverse() * new_pose;

    Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
    R.topLeftCorner(3, 3) = diff_pose.topLeftCorner(3, 3);
    Eigen::Vector4d trans = new_pose.col(3) - old_pose.col(3);

    std::stringstream filename;
    filename << "loop_residual" << (timestamp_ - 1) << ".txt";
    std::ofstream plot(filename.str());

    std::cout << filename.str();

    map_->render_active(new_pose.cast<float>(), getConfidenceThreshold());
    objective_->setData(currentFrame_, map_->newMapFrame());
    objective_->initialize(Eigen::Matrix4d::Identity());

    float residual_new = objective_->jacobianProducts(JtJ, Jtr);

    for (uint32_t i = 0; i < 100; ++i) {
      float interp = 2.0f * i / 100.0f - 0.5f;
      Eigen::Matrix4d new_diff = diff_pose;
      new_diff.col(3) = interp * trans;
      new_diff(3, 3) = 1.0f;

      Eigen::Matrix4d interp_pose = old_pose * new_diff;

      map_->render_inactive(old_pose.cast<float>(), getConfidenceThreshold());
      objective_->setData(currentFrame_, map_->oldMapFrame());
      objective_->initialize(interp_pose);
      float residual_old = objective_->jacobianProducts(JtJ, Jtr);
      float outlier_old = objective_->outlier();
      float residual_inlier_old = objective_->inlier_residual();
      float inlier_old = objective_->inlier();

      map_->render_composed(interp_pose.cast<float>(), new_pose.cast<float>(), getConfidenceThreshold());
      objective_->setData(currentFrame_, map_->composedFrame());
      objective_->initialize(Eigen::Matrix4d::Identity());
      float residual_map = objective_->jacobianProducts(JtJ, Jtr);
      float outlier_map = objective_->outlier();
      float residual_inlier_map = objective_->inlier_residual();
      float inlier_map = objective_->inlier();

      plot << interp << " " << residual_old << " " << residual_map << " ";
      plot << residual_inlier_old << " " << residual_inlier_map << " ";
      plot << outlier_old << " " << outlier_map << " ";
      plot << inlier_old << " " << inlier_map << " ";
      plot << residual_new << std::endl;
    }

    plot.close();

    std::cout << std::endl;
  }
}

void SurfelMapping::initialize(const Laserscan& scan) {
  lastFrame_.swap(currentFrame_);  // current frame is the last frame.
  lastModelFrame_.swap(currentModelFrame_);

  current_pts_.assign(scan.points());

  current_labels_.assign(scan.labels_float);
  current_probs_.assign(scan.labels_prob);
}

float SurfelMapping::getConfidenceThreshold() {
  float ct = confidence_threshold_;
  if (timestamp_ < time_init) {
    float alpha = float(timestamp_) / float(time_init);
    ct = (1.0 - alpha) * log_unstable_ + alpha * confidence_threshold_;
  }
  return ct;
}

void SurfelMapping::preprocess() {
  Stopwatch::tic();
  preprocessor_.process(current_pts_, *currentFrame_, current_labels_, current_probs_, timestamp_);

  if (performMapping_) {
    // linearly increase in the beginning the confidence threshold, such that we have some information...
    float ct = getConfidenceThreshold();

    Stopwatch::tic();
    map_->render(currentPose_old_.cast<float>(), currentPose_new_.cast<float>(), *lastModelFrame_, ct);
    statistics_["map rendering"] = 1000. * Stopwatch::toc();

    lastModelFrame_->pose = currentPose_new_.cast<float>();
  }

  statistics_["preprocessing-time"] = Stopwatch::toc();
}

bool SurfelMapping::foundLoopClosureCandidate() {
  return foundLoopClosureCandidate_;
}

bool SurfelMapping::useLoopClosureCandidate() {
  return useLoopClosureCandidate_;
}

const std::vector<Eigen::Matrix4f>& SurfelMapping::getLoopClosurePoses() const {
  return loopClosurePoses_;
}

void SurfelMapping::updatePose() {
  if (!initialize_identity_) {
    T0 = lastIncrement_;  // take last pose increment for initialization.
  } else
    T0 = Eigen::Matrix4d::Identity();

  Stopwatch::tic();

  // determine ego-motion of vehicle. (Assuming the map only contains static stuff.)

  Stopwatch::tic();
  if (performMapping_)
    objective_->setData(currentFrame_, map_->newMapFrame());
  else
    objective_->setData(currentFrame_, lastFrame_);

  Stopwatch::tic();
  int32_t success = gn_->minimize(*objective_, T0);

  odom_poses_ = gn_->history();

  statistics_["opt-time"] = Stopwatch::toc();
  statistics_["num_iterations"] = gn_->iterationCount();

  Eigen::Matrix4d increment = gn_->pose();

  Eigen::Matrix4d delta = lastIncrement_.inverse() * increment;

  Eigen::MatrixXd JtJ_new(6, 6);
  Eigen::MatrixXd Jtr(6, 1);

  Stopwatch::tic();

  if (performMapping_) {
    map_->render_active((currentPose_new_ * increment).cast<float>(), getConfidenceThreshold());
    lastModelFrame_->copy(*map_->newMapFrame());
    objective_->setData(currentFrame_, map_->newMapFrame());
  }

  objective_->initialize(Eigen::Matrix4d::Identity());

  double residual = objective_->jacobianProducts(JtJ_new, Jtr);

  statistics_["time_residual_new"] = Stopwatch::toc();

  result_new_.error = residual;  // gn_->residual();
  result_new_.inlier = objective_->inlier();
  result_new_.outlier = objective_->outlier();
  result_new_.residual = result_new_.error / (result_new_.inlier + result_new_.outlier);
  result_new_.inlier_residual = objective_->inlier_residual() / result_new_.inlier;
  result_new_.invalid = objective_->invalid();
  result_new_.valid = objective_->valid();

  statistics_["icp-time"] = Stopwatch::toc();

  if (success < -1) {
    std::cerr << "minimization not successful. reason => " << gn_->reason(success) << std::endl;
  }

  // replace this with simple brute force try (try always both possible initializations and take the one with smaller
  // inlier point2point error.
  // check pose increment error:
  float t_err = std::sqrt(delta(0, 3) * delta(0, 3) + delta(1, 3) * delta(1, 3) + delta(2, 3) * delta(2, 3));
  float angle = 0.5 * (delta(0, 0) + delta(1, 1) + delta(2, 2) - 1.0);
  float r_err = std::acos(std::max(std::min(angle, 1.0f), -1.0f));

  if (timestamp_ > 1 && (t_err > 0.4 || r_err > 0.1) && fallbackMode_ && recovery_ != nullptr) {
    std::cout << "t_err: " << t_err << ", r_err: " << r_err << std::endl;
    std::cout << "Lost track: Pose increment change too large. Falling back to frame-to-frame tracking." << std::endl;
    std::cout << "timestamp_: " << timestamp_ << std::endl;
    trackLoss_ += 1;
    recovery_->setData(currentFrame_, lastFrame_);

    success = gn_->minimize(*recovery_, T0);
    increment = gn_->pose();

    if (success < -1) std::cout << "minimization not successful. reason => " << gn_->reason(success) << std::endl;
  }

  lastError = result_new_.error;

  lastPose_ = currentPose_;
  currentPose_ = currentPose_ * increment;
  lastPose_old_ = currentPose_old_;
  currentPose_old_ = currentPose_;
  currentPose_new_ = currentPose_;

  currentFrame_->pose = currentPose_.cast<float>();

  float distance = 0;

  if (timestamp_ > 0) {
    posegraph_->setInitial(timestamp_, posegraph_->pose(timestamp_ - 1) * increment);

    posegraph_->addEdge(timestamp_ - 1, timestamp_, increment, info_);
    distance = (posegraph_->pose(timestamp_ - 1).col(3) - currentPose_.col(3)).norm();
    distance += trajectory_distances_[timestamp_ - 1];
  }

  trajectory_distances_.push_back(distance);

  lastIncrement_ = increment;  // take last pose increment for initialization.

  statistics_["icp-overall"] = Stopwatch::toc();
}

std::vector<int32_t> SurfelMapping::getCandidateIndexes(float radius) {
  std::vector<int32_t> candidates;

  int32_t closest_idx = -1;
  float min_distance = radius;
  for (int32_t j = int32_t(timestamp_) - loopDetlaTimestamp_; j >= 0; --j) {
    float distance = pose_distance(currentPose_, posegraph_->pose(j));
    float tdistance = trajectory_distances_[timestamp_] - trajectory_distances_[j];

    if (distance < min_distance && tdistance > loopClosureMinTrajDist_) {
      closest_idx = j;
      min_distance = distance;
    }
  }

  if (closest_idx > -1) {
    candidates.push_back(closest_idx);
  }
  return candidates;
}

double SurfelMapping::pose_distance(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b) const {
  return (a.col(3) - b.col(3)).norm();
}

int32_t SurfelMapping::getClosestIndex(const Eigen::Matrix4d& query) {
  int32_t closest_idx = -1;

  float min_distance = loopClosureSearchDist_;
  for (int32_t j = int32_t(timestamp_) - loopDetlaTimestamp_; j >= 0; --j) {
    float distance = pose_distance(currentPose_, posegraph_->pose(j));
    float tdistance = trajectory_distances_[timestamp_] - trajectory_distances_[j];

    if (distance < min_distance && tdistance > loopClosureMinTrajDist_) {
      closest_idx = j;
      min_distance = distance;
    }
  }

  return closest_idx;
}

Eigen::Matrix4d R(const Eigen::Matrix4d& M) {
  Eigen::Matrix4d Rot = M;
  Rot(0, 3) = Rot(1, 3) = Rot(2, 3) = 0.0;

  return Rot;
}

void SurfelMapping::checkLoopClosure() {
  // Check nearby poses to search a loop closure.

  Stopwatch::tic();

  Eigen::MatrixXd JtJ(6, 6);
  Eigen::MatrixXd Jtr(6, 1);

  foundLoopClosureCandidate_ = false;
  loopClosurePoses_.clear();
  result_old_ = OptResult();  // reset.
  useLoopClosureCandidate_ = false;

  Eigen::Matrix4d increment_old;

  bool candidateAdded = false;
  int32_t minCandidate = -1;

  float outlier_ratio_new = result_new_.outlier / float(result_new_.outlier + result_new_.inlier);
  float valid_ratio_new = float(result_new_.valid) / float(result_new_.invalid + result_new_.valid);

  timeWithoutLoopClosure_ += 1;

  // 1. verify loop closure if there are unverified loop closures:
  if (unverifiedLoopClosures_.size() > 0 || alreadyVerifiedLoopClosure_) {

    Eigen::Matrix4f pose_old = lastPose_old_.cast<float>();
    // Verifying and adding constraints.
    map_->render_inactive(pose_old, getConfidenceThreshold());

    // adjust increment.
    objective_->setData(currentFrame_, map_->oldMapFrame());
    gn_->minimize(*objective_, lastIncrement_);

    float valid_ratio = float(objective_->valid()) / float(objective_->valid() + objective_->invalid());
    float outlier_ratio = float(objective_->outlier()) / float(objective_->outlier() + objective_->inlier());

    increment_old = gn_->pose();
    float increment_difference = (SE3::log(lastIncrement_) - SE3::log(increment_old)).norm();

    if (valid_ratio > 0.2 && outlier_ratio < 0.85 && increment_difference < 0.1) {
      pose_old = (lastPose_old_ * increment_old).cast<float>();

      // render fused virtual map and compare with current estimate from odometry:
      map_->render_composed(pose_old, currentPose_new_.cast<float>(), getConfidenceThreshold());

      objective_->setData(currentFrame_, map_->composedFrame());
      objective_->initialize(Eigen::Matrix4d::Identity());

      float error = objective_->jacobianProducts(JtJ, Jtr);
      float residual = error / float(objective_->inlier() + objective_->outlier());

      result_old_.error = error;  // gn_->residual();
      result_old_.inlier = objective_->inlier();
      result_old_.outlier = objective_->outlier();
      result_old_.residual = result_old_.error / float(result_old_.inlier + result_old_.outlier);
      result_old_.inlier_residual = objective_->inlier_residual() / result_old_.inlier;
      result_old_.valid = objective_->valid();
      result_old_.invalid = objective_->invalid();

      float rel_error_all = residual / result_new_.residual;

      foundLoopClosureCandidate_ = true;
      currentPose_old_ = lastPose_old_ * increment_old;
      loopClosurePoses_.push_back(currentPose_old_.cast<float>());

      bool loop_closure = (rel_error_all < loopResidualThres_) || (residual - result_new_.residual) < 0.1;

      if (loop_closure) {
        timeWithoutLoopClosure_ = 0;

        LoopClosureCandidate candidate;
        candidate.from = timestamp_;

        int32_t index = getClosestIndex(currentPose_old_);
        if (index > -1) {
          candidate.to = index;
          Eigen::Matrix4d nearest_pose = posegraph_->pose(index);
          lastAddedLoopClosureCandidate_ = candidate.to;
          candidate.rel_pose = currentPose_old_.inverse() * nearest_pose;

          auto* candidateList = &unverifiedLoopClosures_;
          if (alreadyVerifiedLoopClosure_) candidateList = &verifiedLoopClosures_;

          candidateList->push_back(candidate);
        }
      }

      if (loop_closure) {
        // needed for correct visualization...
        loopClosurePoses_.push_back(loopClosurePoses_[0]);
        loopClosurePoses_[0] = Eigen::Matrix4f::Zero();
        useLoopClosureCandidate_ = true;

      } else {
        loopClosurePoses_.push_back(Eigen::Matrix4f::Zero());
      }
    }
  }

  // enough verified loop closures.
  if (!alreadyVerifiedLoopClosure_ && int32_t(unverifiedLoopClosures_.size()) >= (loopMinNumberVerifications_ + 1)) {
    for (uint32_t i = 0; i < unverifiedLoopClosures_.size(); ++i)
      verifiedLoopClosures_.push_back(unverifiedLoopClosures_[i]);
    unverifiedLoopClosures_.clear();
    alreadyVerifiedLoopClosure_ = true;
  }

  int32_t lastFrom = -1;
  // 2. Add verified loop closures.
  {
    for (const auto& candidate : verifiedLoopClosures_) {
      int32_t from = candidate.from;
      int32_t to = candidate.to;
      const Eigen::Matrix4d& diff = candidate.rel_pose;

      // add constraints and optimize.
      if (lastFrom != from) {
        lastFrom = from;
        loopCount_ += 1;
      }

      posegraph_->addEdge(from, to, diff, info_);
    }

    verifiedLoopClosures_.clear();
  }

  if ((loopCount_ > 6 && !currentlyOptimizing_) ||
      (loopCount_ > 0 && !currentlyOptimizing_ && (timeWithoutLoopClosure_ > 3))) {
    optimizedPosegraph_ = posegraph_->clone();
    optimizeFuture_ = std::async(std::launch::async, std::bind(&SurfelMapping::optimizeAsync, this));
//    optimizeFuture_.wait();  // calling this synchronously to directly see the error.
  }

  if (timeWithoutLoopClosure_ > 3) {
    // Finding a potential loop closure.
    unverifiedLoopClosures_.clear();  // drop unverified.
    useLoopClosureCandidate_ = false;
    alreadyVerifiedLoopClosure_ = false;

    std::vector<int32_t> candidateTimestamps = getCandidateIndexes(loopClosureSearchDist_);
    int32_t loopClosureTimestamp = -1;

    for (uint32_t c = 0; c < candidateTimestamps.size(); ++c) {
      int32_t to = candidateTimestamps[c];

      Stopwatch::tic();

      Eigen::Matrix4d pose_prior = posegraph_->pose(to);
      map_->render_inactive(pose_prior.cast<float>(), getConfidenceThreshold());

      std::vector<Eigen::Matrix4d> initializations;
      Eigen::Matrix4d O = pose_prior.inverse() * currentPose_;
      O(2, 3) = 0.0;

      initializations.push_back(O);
      initializations.push_back(R(O));

      // FIXME: interpolate translational difference vector instead of value!!!
      Eigen::Matrix4d init = O;
      init(0, 3) = 0.5 * O(0, 3);
      init(1, 3) = 0.5 * O(1, 3);

      initializations.push_back(init);

      objective_->setData(currentFrame_, map_->oldMapFrame());

      double optimizationTime = 0.0f;
      double renderingTime = 0.0f;

      for (uint32_t i = 0; i < initializations.size(); ++i) {
        Stopwatch::tic();
        gn_->minimize(*objective_, initializations[i]);
        optimizationTime += 1000 * Stopwatch::toc();

        // float error =
        objective_->jacobianProducts(JtJ, Jtr);

        float valid_ratio = float(objective_->valid()) / float(objective_->valid() + objective_->invalid());
        float outlier_ratio = float(objective_->outlier()) / float(objective_->outlier() + objective_->inlier());
        // float residual = error / float(objective_->inlier() + objective_->outlier());
        foundLoopClosureCandidate_ = true;

        loopClosurePoses_.push_back((pose_prior * gn_->pose()).cast<float>());

        if (valid_ratio > 0.2 && outlier_ratio < 0.85) {
          Eigen::Matrix4f pose = (pose_prior * gn_->pose()).cast<float>();

          Stopwatch::tic();
          map_->render_composed(pose, currentPose_new_.cast<float>(), getConfidenceThreshold());
          renderingTime += 1000 * Stopwatch::toc();

          objective_->setData(currentFrame_, map_->composedFrame());
          objective_->initialize(Eigen::Matrix4d::Identity());

          float error = objective_->jacobianProducts(JtJ, Jtr);

          float residual = error / float(objective_->inlier() + objective_->outlier());
          float outlier_ratio_old = float(objective_->outlier()) / float(objective_->outlier() + objective_->inlier());
          float valid_ratio_old = objective_->valid() / float(objective_->valid() + objective_->invalid());

          float rel_error_all = residual / result_new_.residual;
          float rel_valid_ratio = valid_ratio_old / valid_ratio_new;
          float rel_outlier_ratio = outlier_ratio_old / outlier_ratio_new;

          if (!candidateAdded || (residual < result_old_.residual && outlier_ratio_old < result_old_.outlier_ratio)) {
            if (rel_valid_ratio >= loopValidThres_ && rel_outlier_ratio < loopOutlierThres_) {
              candidateAdded = true;
              loopClosureTimestamp = to;
              minCandidate = loopClosurePoses_.size() - 1;

              result_old_.pose = pose_prior * gn_->pose();
              result_old_.error = error;
              result_old_.information = JtJ;
              result_old_.inlier = objective_->inlier();
              result_old_.outlier = objective_->outlier();
              result_old_.outlier_ratio = outlier_ratio_old;
              result_old_.valid = objective_->valid();
              result_old_.residual = result_old_.error / result_old_.valid;
              result_old_.inlier_residual = objective_->inlier_residual() / result_old_.inlier;
              result_old_.invalid = objective_->invalid();

              bool loop_closure = (rel_error_all < loopResidualThres_) || (residual - result_new_.residual) < 0.1;
              if (loop_closure) {
                currentPose_old_ = pose_prior * gn_->pose();
              }
            }
          }
        }
      }

      statistics_["additional-icp-time"] = Stopwatch::toc();
    }

    if (minCandidate > -1) {
      // needed for correct visualization...
      loopClosurePoses_.push_back(loopClosurePoses_[minCandidate]);
      loopClosurePoses_[minCandidate] = Eigen::Matrix4f::Zero();

      LoopClosureCandidate candidate;
      candidate.from = timestamp_;
      int32_t to = loopClosureTimestamp;
      candidate.to = to;
      candidate.rel_pose = currentPose_old_.inverse() * posegraph_->pose(to);
      lastAddedLoopClosureCandidate_ = to;

      unverifiedLoopClosures_.push_back(candidate);

    } else {
      loopClosurePoses_.push_back(Eigen::Matrix4f::Zero());
    }
  }

  float valid_ratio_old = float(result_old_.valid) / float(result_old_.valid + result_old_.invalid);
  float outlier_ratio_old = float(result_old_.outlier) / float(result_old_.outlier + result_old_.inlier);

  statistics_["loop_valid_ratio"] = valid_ratio_old / valid_ratio_new;
  statistics_["loop_outlier_ratio"] = outlier_ratio_old / outlier_ratio_new;
  statistics_["loop_relative_error_all"] = result_old_.residual / result_new_.residual;

  statistics_["residual_old"] = result_old_.residual;
  statistics_["loopOutlierThres"] = loopOutlierThres_;
  statistics_["loopValidThres"] = loopValidThres_;
  statistics_["loopResidualThres"] = loopResidualThres_;

  statistics_["posegraph_error"] = posegraph_->error();
  statistics_["time_loopdetection"] = Stopwatch::toc();
}

void SurfelMapping::updateMap() {
  Stopwatch::tic();
  map_->update(currentPose_.cast<float>(), *currentFrame_);
  statistics_["map-update"] = Stopwatch::toc();

  float ct = getConfidenceThreshold();
  map_->render(currentPose_.cast<float>(), *currentModelFrame_, ct);
}

void SurfelMapping::globallyOptimize() {
  if (posegraph_->optimize(100)) {
    const std::vector<Eigen::Matrix4d>& poses = posegraph_->poses();
    std::vector<Eigen::Matrix4f> casted_poses;
    for (const auto& pose : poses) {
      casted_poses.push_back(pose.cast<float>());
    }
    map_->updatePoses(casted_poses);
    currentlyLoopClosing_ = false;
    currentPose_ = currentPose_new_ = currentPose_old_ = poses.back();
  }
}

bool SurfelMapping::optimizeAsync() {
  currentlyOptimizing_ = true;
  beforeID_ = timestamp_;
  beforeLoopCount_ = loopCount_;
  beforeOptimizationPose_ = optimizedPosegraph_->pose(timestamp_);

  return optimizedPosegraph_->optimize(100);
}

void SurfelMapping::initializeGraph() {
  posegraph_->reinitialize();
}

std::vector<Eigen::Matrix4d> SurfelMapping::getOptimizedPoses() const {
  return posegraph_->poses();
}

void SurfelMapping::storePoseGraph(const std::string& filename) const {
  posegraph_->save(filename);
}

std::shared_ptr<SurfelMap> SurfelMapping::getMap() {
  return map_;
}

const Eigen::Matrix4d& SurfelMapping::getLastPose() const {
  return lastPose_;
}

const Eigen::Matrix4d& SurfelMapping::getCurrentPose() const {
  return currentPose_;
}

Frame::Ptr SurfelMapping::getCurrentFrame() {
  return currentFrame_;
}

Frame::Ptr SurfelMapping::getCurrentModelFrame() {
  if (performMapping_) {
    map_->render(currentPose_.cast<float>(), *currentModelFrame_, getConfidenceThreshold());
  }

  return currentModelFrame_;
}

Frame::Ptr SurfelMapping::getLastModelFrame() {
  return lastModelFrame_;
}

Frame::Ptr SurfelMapping::getLastFrame() {
  return lastFrame_;
}

uint32_t SurfelMapping::width() const {
  return width_;
}

uint32_t SurfelMapping::height() const {
  return height_;
}

void SurfelMapping::setCurrentPose(const Eigen::Matrix4f& pose) {
  currentPose_ = pose.cast<double>();
}

Posegraph::ConstPtr SurfelMapping::getPosegraph() const {
  return posegraph_;
}

const SurfelMapping::Stats& SurfelMapping::getStatistics() const {
  return statistics_;
}

Eigen::Matrix4f SurfelMapping::getNewMapPose() const {
  return currentPose_new_.cast<float>();
}

Eigen::Matrix4f SurfelMapping::getOldMapPose() const {
  return currentPose_old_.cast<float>();
}

glow::GlTextureRectangle SurfelMapping::getResidualMap() {
  return currentFrame_->residual_map;
}

void SurfelMapping::setCalibration(const KITTICalibration& calib) {
  calib_ = calib;
  preprocessor_.setCalibration(calib);
}
