#ifndef INCLUDE_CORE_LASERFUSION_H_
#define INCLUDE_CORE_LASERFUSION_H_

#include <memory>

#include <rv/Laserscan.h>
#include <rv/ParameterList.h>

#include <glow/GlBuffer.h>
#include <glow/GlQuery.h>

#include "LieGaussNewton.h"

#include "Frame.h"
#include "Objective.h"
#include "Preprocessing.h"

#include "SurfelMap.h"

#include <future>
#include "Posegraph.h"

#include "util/kitti_utils.h"

typedef std::tuple< u_char, u_char, u_char> semantic_color;

/** \brief Building model of the static and the dynamic environment from laser point clouds.
 *
 *  \author behley
 *
 *  \author Xieyuanli Chen
 **/
class SurfelMapping {
 public:
  typedef std::unordered_map<std::string, float> Stats;

  SurfelMapping(const rv::ParameterList& params);
  ~SurfelMapping();

  /** \brief set parameters of the approach. **/
  void setParameters(const rv::ParameterList& params);

  /** \brief reset everything. **/
  void reset();

  /** \brief process a scan and update model, i.e., set data, pre-process it, updatePose, updateMap. **/
  void processScan(const rv::Laserscan& scan);

  uint32_t timestamp() const;

  /** \brief get frame of current timestep. **/
  Frame::Ptr getCurrentFrame();

  /** \brief get frame of last timestep. **/
  Frame::Ptr getLastFrame();

  /** \brief get frame of intermediate ICP step. **/
  Frame::Ptr getIntermediateFrame();

  Frame::Ptr getCurrentModelFrame();
  Frame::Ptr getLastModelFrame();

  const Eigen::Matrix4d& getLastPose() const;
  const Eigen::Matrix4d& getCurrentPose() const;

  void setCurrentPose(const Eigen::Matrix4f& pose);

  std::shared_ptr<SurfelMap> getMap();

  uint32_t width() const;
  uint32_t height() const;

  Frame::Ptr getOldSurfelMap() { return map_->oldMapFrame(); }
  Frame::Ptr getNewSurfelMap() { return map_->newMapFrame(); }
  glow::GlTextureRectangle getResidualMap();

  /** \brief initialize point buffer, etc. **/
  void initialize(const rv::Laserscan& scan);

  /** \brief pre-process data, i.e., perform projection, etc. **/
  void preprocess();

  const Stats& getStatistics() const;

  void globallyOptimize();

  void storePoseGraph(const std::string& filename) const;

  void initializeGraph();

  std::vector<Eigen::Matrix4d> getOptimizedPoses() const;

  Posegraph::ConstPtr getPosegraph() const;

  bool foundLoopClosureCandidate();
  bool useLoopClosureCandidate();
  const std::vector<Eigen::Matrix4f>& getLoopClosurePoses() const;

  Eigen::Matrix4f getNewMapPose() const;
  Eigen::Matrix4f getOldMapPose() const;

  void genResidualPlot();

  const std::vector<Eigen::Matrix4d>& getIntermediateOdometryPoses() const { return odom_poses_; }

  void setCalibration(const KITTICalibration& calib); // for semantic map

  /** \brief set the color map used for semantic mapping **/
  void setColorMap(std::map<uint32_t, semantic_color> color_map) { map_->setColorMap(color_map); }

 protected:
  struct OptResult {
   public:
    Eigen::Matrix4d bestIncrement;
    Eigen::Matrix4d pose;
    Eigen::MatrixXd information;
    double error{10000.0};
    double residual{10000.0}, inlier_residual{10000.0};
    uint32_t inlier{0}, outlier{0};
    uint32_t valid{0}, invalid{0};
    float outlier_ratio{1.0};
  };

  struct LoopClosureCandidate {
   public:
    int32_t from, to;
    Eigen::Matrix4d rel_pose;
  };

  /** \brief update the pose estimate using a certain number of Gauss-Newton steps. **/
  void updatePose();

  /** \brief if pose graph optimization finished: update poses, etc. **/
  void integrateLoopClosures();

  /** \brief try to find loop closure in old parts of the map & add constraint to pose graph. **/
  void checkLoopClosure();

  /** \brief use current pose and data to update map. **/
  void updateMap();

  float getConfidenceThreshold();

  /** \brief asynchronously optimize a copy of the posegraph. **/
  bool optimizeAsync();

  OptResult optimizePyramid(const std::vector<Eigen::Matrix4d>& inits);

  int32_t getClosestIndex(const Eigen::Matrix4d& query);
  std::vector<int32_t> getCandidateIndexes(float radius);

  double pose_distance(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b) const;

  glow::GlBuffer<rv::Point3f> current_pts_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_READ};
  glow::GlBuffer<float> current_labels_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_READ}; // for semantic map
  glow::GlBuffer<float> current_probs_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_READ};

  Preprocessing preprocessor_;

  uint32_t timestamp_{0};

  Eigen::Matrix4d currentPose_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d lastPose_{Eigen::Matrix4d::Identity()};

  uint32_t width_, height_;
  Frame::Ptr lastFrame_, currentFrame_, intermediateFrame_;

  int idx = 0;

  std::shared_ptr<LieGaussNewton> gn_;
  std::shared_ptr<Objective> objective_;

  rv::ParameterList params_;

  bool performMapping_{false};
  std::shared_ptr<SurfelMap> map_;
  Frame::Ptr currentModelFrame_, lastModelFrame_;
  Frame::Ptr lastLoopClosureFrame_;

  float confidence_threshold_;
  float log_unstable_;
  bool initialize_identity_{true};
  Eigen::Matrix4d T0{Eigen::Matrix4d::Identity()};

  float lastError{0.0f};
  Eigen::Matrix4d lastIncrement_{Eigen::Matrix4d::Identity()};
  uint32_t trackLoss_{0};
  bool fallbackMode_{false};

  std::shared_ptr<Objective> recovery_;

  uint32_t time_init{10};

  bool firstTime_{true};

  Stats statistics_;

  Posegraph::Ptr posegraph_;
  std::vector<float> trajectory_distances_;

  Eigen::Matrix4d currentPose_old_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d lastPose_old_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d currentPose_new_{Eigen::Matrix4d::Identity()};
  bool currentlyLoopClosing_{false};
  uint32_t loopCount_{0};
  uint32_t timeWithoutLoopClosure_{0};  // hysteresis thresholding.

  Eigen::DiagonalMatrix<double, 6> info_;

  bool currentlyOptimizing_{false};
  std::future<bool> optimizeFuture_;
  Posegraph::Ptr optimizedPosegraph_;
  int32_t beforeID_;
  Eigen::Matrix4d beforeOptimizationPose_;
  uint32_t beforeLoopCount_{0};

  bool foundLoopClosureCandidate_{false};
  bool useLoopClosureCandidate_{false};
  std::vector<Eigen::Matrix4f> loopClosurePoses_;

  float loopClosureSearchDist_{20.0f};
  float loopClosureMinTrajDist_{200.0f};
  int32_t loopMinNumberVerifications_{3};

  float loopResidualThres_{1.05};
  float loopOutlierThres_{1.1};
  float loopValidThres_{0.9};
  float loopDetlaTimestamp_{100};
  float validThres_{0.4};

  bool makeLoopClosures_{true};

  OptResult result_new_, result_old_;

  bool alreadyVerifiedLoopClosure_{false};
  std::vector<LoopClosureCandidate> unverifiedLoopClosures_;
  std::vector<LoopClosureCandidate> verifiedLoopClosures_;
  std::vector<LoopClosureCandidate> nearbyLoopClosures_;
  int32_t lastAddedLoopClosureCandidate_{-1};

  glow::GlQuery time_query_{glow::QueryTarget::TIME_ELAPSED};

  std::vector<Eigen::Matrix4d> odom_poses_;
  float init_factor_;

  KITTICalibration calib_; // for semantic map

};

#endif /* INCLUDE_CORE_LASERFUSION_H_ */
