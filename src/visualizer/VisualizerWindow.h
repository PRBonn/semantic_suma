#ifndef SRC_VISUALIZER_VISUALIZERWINDOW_H_
#define SRC_VISUALIZER_VISUALIZERWINDOW_H_

#include <core/SurfelMapping.h>
#include "rv/LaserscanReader.h"
#include "ui_visualizer.h"

#include "opengl/Model.h"
#include "util/kitti_utils.h"

#include "io/SimulationReader.h"
#include "util/ScanAccumulator.h"

/**
 *  \author behley
 *
 *  \author Xieyuanli Chen
 */

class VisualizerWindow : public QMainWindow {
  Q_OBJECT
 public:
  VisualizerWindow();
  VisualizerWindow(int argc, char** argv);
  ~VisualizerWindow();

  void initialize(const std::shared_ptr<SurfelMapping>& fusion, const rv::ParameterList& params);

  std::vector<SurfelMapping::Stats> statistics;

 public slots:
  /** \brief open the given file and initialize the laser scan reader. **/
  void openFile(const QString& filename);
  void startRecording();

 protected slots:
  /**  \brief open a laser scan file and initialize laser scan reader. **/
  void showOpenFileDialog();

  /** \brief start sequential reading of laser scans and update model. **/
  void play(bool toggle);

  /** \brief read next scan **/
  void nextScan();

  /** \brief read specific scan. **/
  void setScan(int idx);

  /** \brief reset reader's state and model. **/
  void reset();

  void updateParameters();

  void updateScanIndex();

  void renderMaps();

  void triggerPreprocessing();

  void optimizeGraph();
  void initializeGraph();

  void savePoses();

 protected:
  void updatePlaybackControls();

  void initilizeKITTIrobot();
  void printResultStatistics(const std::vector<Eigen::Matrix4f>& gt_poses,
                             const std::vector<Eigen::Matrix4f>& result_poses);
  void storeVelodynePoses();

  Ui::MainWindow ui_;
  rv::LaserscanReader* reader_{nullptr};

  uint32_t currentScanIdx_{0};
  rv::Laserscan currentLaserscan_;
  QString lastDirectory_;
  QTimer timer_;

  std::shared_ptr<SurfelMapping> fusion_;

  Eigen::Isometry3f pose_{Eigen::Isometry3f::Identity()};

  std::vector<bool> precomputed_;
  std::vector<Eigen::Matrix4f> oldPoses_;

  std::vector<Eigen::Matrix4f> groundtruth_;
  KITTICalibration calib_;

  bool recording_{false};
  std::string outputDirectory_;

  std::shared_ptr<ScanAccumulator> acc_;

  uint32_t start_scan{0};

  rv::ParameterList params_;

  Eigen::Matrix4f adjustment_{Eigen::Matrix4f::Identity()};
  bool parameterUpdateNeeded_{false};
  uint32_t history_stride_{1};
  float scanFrequency{10.0f};

};

#endif /* SRC_VISUALIZER_VISUALIZERWINDOW_H_ */
