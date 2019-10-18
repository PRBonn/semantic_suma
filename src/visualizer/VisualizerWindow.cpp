#include "VisualizerWindow.h"
#include <glow/glbase.h>
#include <rv/FileUtil.h>
#include <rv/PrimitiveParameters.h>
#include <rv/geometry.h>
#include <rv/string_utils.h>
#include <QtWidgets/QFileDialog>
#include "core/lie_algebra.h"
#include "io/KITTIReader.h"
#include "io/SimulationReader.h"
#include "opengl/ObjReader.h"

#include <rv/string_utils.h>
#include <boost/lexical_cast.hpp>
#include <core/SurfelMapping.h>
#include <fstream>
#include <iostream>
#include <vector>

#include "io/RobocarReader.h"
#include "util/kitti_utils.h"

#include <rv/Stopwatch.h>
#include <opencv2/core/core.hpp>

#ifdef QUERY_MEMORY_NV
#define GL_GPU_MEM_INFO_TOTAL_AVAILABLE_MEM_NVX 0x9048
#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049
#endif
#include <QMessageBox>
using namespace rv;
using namespace glow;

VisualizerWindow::VisualizerWindow(int argc, char** argv) {
  ui_.setupUi(this);
  lastDirectory_ = "~";
  // alternative icons (rcc) for windows/mac?
  ui_.actionOpenLaserscan->setIcon(QIcon::fromTheme("document-open"));
  ui_.btnPlay->setIcon(QIcon::fromTheme("media-playback-start"));
  ui_.btnNext->setIcon(QIcon::fromTheme("media-seek-forward"));
  ui_.btnReset->setIcon(QIcon::fromTheme("media-skip-backward"));
  ui_.actionRecord->setIcon(QIcon::fromTheme("media-record"));
  connect(ui_.actionOpenLaserscan, SIGNAL(triggered()), this, SLOT(showOpenFileDialog()));
  connect(ui_.actionRecord, SIGNAL(triggered()), this, SLOT(startRecording()));
  connect(&timer_, SIGNAL(timeout()), this, SLOT(nextScan()));
  connect(ui_.btnPlay, SIGNAL(toggled(bool)), this, SLOT(play(bool)));
  connect(ui_.btnNext, SIGNAL(clicked()), this, SLOT(nextScan()));
  connect(ui_.btnReset, SIGNAL(clicked()), this, SLOT(reset()));
  connect(ui_.sldTimeline, SIGNAL(valueChanged(int)), this, SLOT(setScan(int)));
  connect(ui_.spinPointSize, SIGNAL(valueChanged(int)), ui_.wCanvas, SLOT(setPointSize(int)));
  connect(ui_.cmbPointColorMode, SIGNAL(currentIndexChanged(int)), ui_.wCanvas, SLOT(setPointColorMode(int)));
  connect(ui_.cmbDepthColorMode, SIGNAL(currentIndexChanged(int)), ui_.wCanvas, SLOT(setDepthMapColorMode(int)));
  connect(ui_.cmbSurfelColor, SIGNAL(currentIndexChanged(int)), ui_.wCanvas, SLOT(setSurfelColorMode(int)));
  connect(ui_.spinHistorySize, SIGNAL(valueChanged(int)), ui_.wCanvas, SLOT(setHistorySize(int)));
  connect(ui_.spinConfidenceThreshold, SIGNAL(valueChanged(double)), this, SLOT(updateParameters()));
  connect(ui_.spinConfidenceThreshold, SIGNAL(valueChanged(double)), ui_.wCanvas, SLOT(setConfidenceThreshold(double)));

  connect(ui_.spinIcpMaxAngle, SIGNAL(valueChanged(double)), this, SLOT(updateParameters()));
  connect(ui_.spinIcpMaxDistance, SIGNAL(valueChanged(double)), this, SLOT(updateParameters()));
  connect(ui_.spinIcpMaxIterations, SIGNAL(valueChanged(int)), this, SLOT(updateParameters()));
  connect(ui_.chkWarmStart, SIGNAL(toggled(bool)), this, SLOT(updateParameters()));

  connect(ui_.spinProbStable, SIGNAL(valueChanged(double)), this, SLOT(updateParameters()));
  connect(ui_.spinProbPrior, SIGNAL(valueChanged(double)), this, SLOT(updateParameters()));
  connect(ui_.spinSigmaDistance, SIGNAL(valueChanged(double)), this, SLOT(updateParameters()));
  connect(ui_.spinSigmaAngle, SIGNAL(valueChanged(double)), this, SLOT(updateParameters()));
  connect(ui_.spinMapMaxDistance, SIGNAL(valueChanged(double)), this, SLOT(updateParameters()));
  connect(ui_.spinMapMaxAngle, SIGNAL(valueChanged(double)), this, SLOT(updateParameters()));

  connect(ui_.cmbOptWeighting, SIGNAL(currentIndexChanged(int)), this, SLOT(updateParameters()));
  connect(ui_.spinOptFactor, SIGNAL(valueChanged(double)), this, SLOT(updateParameters()));
  connect(ui_.cmbWeightingScheme, SIGNAL(currentIndexChanged(int)), this, SLOT(updateParameters()));
  connect(ui_.cmbAveragingScheme, SIGNAL(currentIndexChanged(int)), this, SLOT(updateParameters()));
  connect(ui_.chkUpdateAlways, SIGNAL(toggled(bool)), this, SLOT(updateParameters()));

  connect(ui_.spinResidualThres, SIGNAL(valueChanged(double)), this, SLOT(updateParameters()));
  connect(ui_.spinValidThres, SIGNAL(valueChanged(double)), this, SLOT(updateParameters()));
  connect(ui_.spinOutlierThres, SIGNAL(valueChanged(double)), this, SLOT(updateParameters()));

  connect(ui_.btnRenderMaps, SIGNAL(clicked()), this, SLOT(renderMaps()));

  connect(ui_.chkShowCurrentScan, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("current points", value); });
  connect(ui_.chkShowCoordinateAxis, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("coordinate axis", value); });

  // visual switches via lambda expression...woohoo. ;)
  connect(ui_.chkShowCurrentScan, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("current points", value); });

  connect(ui_.chkShowSemanticMap, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("semantic map", value); });

  connect(ui_.chkShowNormalMap, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("normal map", value); });
  connect(ui_.chkShowDepthImage, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("depth map", value); });
  connect(ui_.chkShowVertexMap3d, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("vertex map", value); });
  connect(ui_.chkShowNormalMap3d, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("normal map 3d", value); });
  connect(ui_.chkMarkInvalidDepthValues, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("mark invalid depths", value); });
  connect(ui_.chkShowCurrentSurfels, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("current surfels", value); });
  connect(ui_.chkShowIcpEstimate, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("pose estimate", value); });

  connect(ui_.chkShowLastFrame, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("last frame", value); });

  connect(ui_.chkShowGroundtruth, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("groundtruth", value); });

  connect(ui_.chkShowPoses, &QCheckBox::toggled, [=](bool value) { ui_.wCanvas->setDrawingOption("poses", value); });

  connect(ui_.chkFollowPose, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("follow pose", value); });

  connect(ui_.chkShowHistory, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("draw history", value); });

  connect(ui_.chkShowUpdatedSurfels, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("updated surfels", value); });

  connect(ui_.chkBackfaceCulling, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("backface culling", value); });

  connect(ui_.chkShowSubmaps, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("show submaps", value); });

  connect(ui_.chkShowRobot, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("draw robot", value); });

  connect(ui_.cmbSurfelDrawMode, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
          [=](int value) {
            if (value == 0)
              ui_.wCanvas->setDrawingOption("draw surfel points", false);
            else
              ui_.wCanvas->setDrawingOption("draw surfel points", true);
          });

  connect(ui_.chkShowPosegraph, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("pose graph", value); });

  connect(ui_.chkShowCandidates, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("show lc candidates", value); });

  connect(ui_.chkFastMode, &QCheckBox::toggled, [=](bool value) {
    if (value)
    {
      timer_.setInterval(10);
    }
    else
    {
      timer_.setInterval(1000. * 1. / scanFrequency);
    }
  });
  connect(ui_.chkOldMapOnly, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("old map only", value); });

  connect(ui_.chkColorizeHistory, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("colorize history", value); });

  connect(ui_.chkShowLoopPoses, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("show lc poses", value); });

  connect(ui_.btnOptimize, SIGNAL(clicked()), this, SLOT(optimizeGraph()));
  connect(ui_.btnInitialize, SIGNAL(clicked()), this, SLOT(initializeGraph()));

  ui_.txtScanNumber->setValidator(new QIntValidator());

  connect(ui_.txtScanNumber, SIGNAL(editingFinished()), this, SLOT(updateScanIndex()));
  connect(ui_.chkFilterVertices, SIGNAL(toggled(bool)), this, SLOT(triggerPreprocessing()));

  connect(ui_.actionSavePoses, SIGNAL(triggered()), this, SLOT(savePoses()));

  connect(ui_.chkBilateralFiltering, SIGNAL(toggled(bool)), this, SLOT(triggerPreprocessing()));
  connect(ui_.chkFilterVertices, SIGNAL(toggled(bool)), this, SLOT(triggerPreprocessing()));
  connect(ui_.spinBilateralSigmaRange, SIGNAL(valueChanged(double)), this, SLOT(triggerPreprocessing()));
  connect(ui_.spinBilateralSigmaSpace, SIGNAL(valueChanged(double)), this, SLOT(triggerPreprocessing()));

  connect(ui_.btnResidualPlot, &QPushButton::clicked, [=](bool value) { fusion_->genResidualPlot(); });
  connect(ui_.chkShowGraphWidget, &QCheckBox::toggled, [=](bool value) { ui_.wGraph->setVisible(value); });

  connect(ui_.chkShowOdomPoses, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("draw odom poses", value); });

  connect(ui_.chkBirdsEyeView, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("birds eye view", value); });

  connect(ui_.chkRemoveGround, &QCheckBox::toggled,
          [=](bool value) { ui_.wCanvas->setDrawingOption("remove ground", value); });

  ui_.chkFollowPose->setChecked(true);
}

VisualizerWindow::~VisualizerWindow() {
  delete reader_;
}

void VisualizerWindow::showOpenFileDialog() {
  play(false);  // stop playback.

  QString retValue =
      QFileDialog::getOpenFileName(this, "Select laserscan file", lastDirectory_,
                                   "Laserscan files(*.bin *.pcap *.sim);;KITTI laserscans (*.bin);; PCAP "
                                   "laserscans (*.pcap);; Simulation (*.sim)");

  if (!retValue.isNull()) openFile(retValue);
}

void VisualizerWindow::initialize(const std::shared_ptr<SurfelMapping>& fusion, const ParameterList& params) {
  params_ = params;
  fusion_ = fusion;
  ui_.wCanvas->initialize(params);
  ui_.wCanvas->setMap(fusion->getMap());

  uint32_t max_history = 10;
  uint32_t max_history_points = 150000;
  if (params.hasParam("history size")) max_history = params["history size"];
  if (params.hasParam("history points")) max_history_points = params["history points"];
  if (params.hasParam("history stride")) history_stride_ = params["history stride"];
  if (params.hasParam("confidence_threshold"))
    ui_.spinConfidenceThreshold->setValue((float)params["confidence_threshold"]);

  // get parameters.
  ui_.spinIcpMaxAngle->setValue((float)params["icp-max-angle"]);
  ui_.spinIcpMaxDistance->setValue((float)params["icp-max-distance"]);
  ui_.spinIcpMaxIterations->setValue((int)params["max iterations"]);
  ui_.chkWarmStart->setChecked(!((bool)params["initialize_identity"]));
  ui_.spinOptFactor->setValue((float)params["factor"]);

  ui_.spinProbStable->setValue((float)params["p_stable"]);
  ui_.spinProbPrior->setValue((float)params["p_prior"]);

  ui_.spinSigmaAngle->setValue((float)params["sigma_angle"]);
  ui_.spinSigmaDistance->setValue((float)params["sigma_distance"]);

  ui_.spinMapMaxDistance->setValue((float)params["map-max-distance"]);
  ui_.spinMapMaxAngle->setValue((float)params["map-max-angle"]);
  //  ui_.chkIterativeCutoff->setChecked((bool) params["iterative_cutoff"]);

  if (params.hasParam("filter_vertexmap")) {
    ui_.chkBilateralFiltering->setChecked((bool)params["filter_vertexmap"]);
    ui_.chkFilterVertices->setChecked((bool)params["use_filtered_vertexmap"]);
    ui_.spinBilateralSigmaRange->setValue((float)params["bilateral_sigma_range"]);
    ui_.spinBilateralSigmaSpace->setValue((float)params["bilateral_sigma_space"]);
  }

  std::string weightingMethod = (std::string)params["weighting"];
  int32_t idx = 0;
  if (weightingMethod == "huber") idx = 1;
  if (weightingMethod == "turkey") idx = 2;

  ui_.cmbOptWeighting->setCurrentIndex(idx);

  parameterUpdateNeeded_ = false;

  acc_ = std::make_shared<ScanAccumulator>(max_history, max_history_points);
  ui_.spinHistorySize->setMaximum(max_history);

  ui_.wCanvas->setScanAccumualator(acc_);

  if (params.hasParam("show robot")) {
    ui_.chkShowRobot->setChecked(params["show robot"]);
  }

  ui_.wCanvas->setPosegraph(fusion_->getPosegraph());

  ui_.wCanvas->setHistoryStride(history_stride_);

  GraphWidget::Timeseries::Ptr timeseries = ui_.wGraph->addTimeseries("runtime", Qt::red);
  timeseries->addThreshold(0.1);
  timeseries->setMinimum(0);
  timeseries->setMaximum(0.5f);

  timeseries = ui_.wGraph->addTimeseries("additional-time", Qt::yellow);
  timeseries->setMinimum(0);
  timeseries->setMaximum(0.5f);

  auto stats = fusion_->getStatistics();

  ui_.spinResidualThres->setValue(stats["loopResidualThres"]);
  ui_.spinOutlierThres->setValue(stats["loopOutlierThres"]);
  ui_.spinValidThres->setValue(stats["validThres"]);

  timeseries = ui_.wGraph->addTimeseries("loop_outlier_ratio", Qt::green);
  timeseries->addThreshold(stats["loopOutlierThres"]);
  timeseries->setMinimum(0);
  timeseries->setMaximum(1.5);
  timeseries->setFixed(true);

  //  timeseries = ui_.wGraph->addTimeseries("loop_relative_error_inlier", Qt::blue, Qt::DashLine);
  //  timeseries->addThreshold(stats["loopResidualThres"]);
  //  timeseries->setMinimum(0);
  //  timeseries->setMaximum(1.0);
  //  timeseries->setFixed(true);

  timeseries = ui_.wGraph->addTimeseries("loop_relative_error_all", Qt::blue);
  timeseries->addThreshold(stats["loopResidualThres"]);
  timeseries->setMinimum(0);
  timeseries->setMaximum(2.0);
  timeseries->setFixed(true);

  timeseries = ui_.wGraph->addTimeseries("loop_valid_ratio", Qt::green, Qt::DashLine);
  timeseries->addThreshold(stats["validThres"]);
  timeseries->setMinimum(0);
  timeseries->setMaximum(2.0);
  timeseries->setFixed(true);

  timeseries = ui_.wGraph->addTimeseries("loop_outlier_ratio", Qt::green);
  timeseries->addThreshold(stats["outlierThresh"]);
  timeseries->setMinimum(0);
  timeseries->setMaximum(2.0);
  timeseries->setFixed(true);

  //  timeseries = ui_.wGraph->addTimeseries("icp_percentage", Qt::magenta);
  //  timeseries->setMinimum(0);
  //  timeseries->setMaximum(1.0);
  //  timeseries->setFixed(true);

  //  timeseries = ui_.wGraph->addTimeseries("num_iterations", Qt::magenta);
  //  timeseries->setMinimum(0);
  //  timeseries->setMaximum(13);

  timeseries = ui_.wGraph->addTimeseries("increment_difference", Qt::green, Qt::DashLine);
  timeseries->setMinimum(0);
  timeseries->setMaximum(1.0);
  timeseries->addThreshold(0.1);
  timeseries->setFixed(true);

  // plot absolute residual
  //  timeseries = ui_.wGraph->addTimeseries("num_iterations", Qt::magenta);
  //  timeseries->setMinimum(0);
  //  timeseries->setMaximum(13);

}

void VisualizerWindow::startRecording() {
  if (!ui_.actionRecord->isChecked()) {
    recording_ = false;
  } else {
    QString retValue = QFileDialog::getExistingDirectory(this, "Select output directory", lastDirectory_);
    if (!retValue.isNull()) {
      outputDirectory_ = retValue.toStdString();
      recording_ = true;
    }
  }
}

void VisualizerWindow::openFile(const QString& filename) {
  play(false);  // stop playback.

  if (!filename.isNull()) {
    QString title = "Semantic SuMa";
    title.append(" - ");
    QStringList parts = filename.split("/");
    title.append(QString::fromStdString(FileUtil::dirName(filename.toStdString())));
    setWindowTitle(title);

    std::shared_ptr<Model> model;
    ui_.wCanvas->setRobotModel(model);

    std::string extension = "INVALID";

    QFileInfo file_info(filename);
    if (file_info.isDir()) {
      QDir directory(filename);
      QStringList files = directory.entryList(QDir::NoFilter, QDir::Name);

      std::map<std::string, int32_t> suffix_count;
      for (int32_t i = 0; i < files.size(); ++i) {
        QFileInfo info(directory.filePath(files[i]));
        if (!info.isFile()) continue;

        std::string ext = info.suffix().toStdString();
        if (suffix_count.find(ext) == suffix_count.end()) suffix_count[ext] = 0;
        suffix_count[ext] += 1;
      }

      if (suffix_count.size() > 0) {
        int32_t maxCount = suffix_count.begin()->second;
        extension = std::string(".") + suffix_count.begin()->first;

        for (auto& entry : suffix_count) {
          if (entry.second > maxCount) {
            extension = std::string(".") + entry.first;
            maxCount = entry.second;
          }
        }

        std::cout << "Heuristically found '" << extension << "' as extension." << std::endl;
      }
    } else if (file_info.isFile()) {
      extension = std::string(".") + file_info.suffix().toStdString();
    }

    if (extension == ".bin") {
      delete reader_;
      KITTIReader* reader_tmp = new KITTIReader(filename.toStdString(), params_, 50);

      ui_.wCanvas->setColorMap(reader_tmp->getColorMap());  // get color map from rangenet
      fusion_->setColorMap(reader_tmp->getColorMap());

      reader_ = reader_tmp;

      scanFrequency = 10.0f;   // in Hz.
      timer_.setInterval(10);  // 1./10. second = 100 msecs.

      if (fusion_ != nullptr) fusion_->reset();
      ui_.wCanvas->reset();

      uint32_t N = reader_->count();
      precomputed_.resize(N);
      oldPoses_.resize(N);

      for (uint32_t i = 0; i < N; ++i) {
        precomputed_[i] = false;
      }
      if (oldPoses_.size() > 0) oldPoses_[0] = Eigen::Matrix4f::Identity();

      calib_.clear();
      groundtruth_.clear();

      if (parts.size() > 2) {
        std::string calibration_file;
        for (int32_t i = 0; i < parts.size() - 2; ++i) {
          calibration_file += parts[i].toStdString();
          calibration_file += "/";
        }
        calibration_file += "calib.txt";

        std::cout << "calibration filename: " << calibration_file << "..." << std::flush;

        if (rv::FileUtil::exists(calibration_file)) {
          calib_.initialize(calibration_file);
          std::cout << "loaded." << std::endl;

          ui_.wCanvas->setCalibration(calib_);
          fusion_->setCalibration(calib_);
        } else {
          std::cout << "not found." << std::endl;
        }
      }

      if (parts.size() > 3 && calib_.exists("Tr")) {
        std::string sequence = parts[parts.size() - 3].toStdString();

        Eigen::Matrix4f Tr = calib_["Tr"];
        Eigen::Matrix4f Tr_inv = Tr.inverse();

        // find ground truth poses.
        std::string poses_file;
        for (int32_t i = 0; i < parts.size() - 4; ++i) {
          poses_file += parts[i].toStdString();
          poses_file += "/";
        }
        poses_file += "poses/" + sequence + ".txt";

        std::cout << "ground truth filename: " << poses_file << std::endl;
        if (rv::FileUtil::exists(poses_file)) {
          std::vector<Eigen::Matrix4f> poses = KITTI::Odometry::loadPoses(poses_file);

          for (uint32_t i = 0; i < poses.size(); ++i) {
            Eigen::Matrix4f pose = poses[i];

            groundtruth_.push_back(Tr_inv * pose * Tr);
          }

          std::cout << groundtruth_.size() << " poses read." << std::endl;
        } else {
          std::cout << "not found." << std::endl;
        }
      } else {
        std::string sequence = parts[parts.size() - 3].toStdString();

        // find ground truth poses.
        std::string poses_file;
        for (int32_t i = 0; i < parts.size() - 4; ++i) {
          poses_file += parts[i].toStdString();
          poses_file += "/";
        }
        poses_file += "poses/" + sequence + ".txt";

        std::cout << "ground truth filename: " << poses_file << std::endl;
        if (rv::FileUtil::exists(poses_file)) {
          std::vector<Eigen::Matrix4f> poses = KITTI::Odometry::loadPoses(poses_file);

          for (uint32_t i = 0; i < poses.size(); ++i) {
            Eigen::Matrix4f pose = poses[i];

            groundtruth_.push_back(pose);
          }

          std::cout << groundtruth_.size() << " poses read." << std::endl;
        } else {
          std::cout << "not found." << std::endl;
        }
      }

      // load the car model.
      initilizeKITTIrobot();

      ui_.wCanvas->setGroundtruth(groundtruth_);

      ui_.sldTimeline->setEnabled(reader_->isSeekable());
      ui_.sldTimeline->setMaximum(reader_->count() - 1);
      if (ui_.sldTimeline->value() == 0) setScan(0);  // triggers update of scan.
      ui_.sldTimeline->setValue(0);
    }
    else {
      std::cerr << "SuMa++ will fail because of wrong format of input LiDAR scans" << std::endl;
    }
  }

  if (reader_ != nullptr) {
    ui_.wCanvas->setScanCount(reader_->count());
  }
}

void VisualizerWindow::initilizeKITTIrobot() {
  // FIXME: read model,extrinsic calibration, etc.  from settings.xml in scan
  // directory.

  if (!rv::FileUtil::exists("../assets/KIT_passat.obj")) {
    std::cerr << "Error: KIT robot model not found." << std::endl;
    return;
  }

  Eigen::Matrix4f T = glTranslate(0.5, 0, -1.0);

  std::shared_ptr<Model> model = std::make_shared<Model>();

  Mesh passat = ObjReader::fromFile("../assets/KIT_passat.obj");
  model->attach(T * glRotateZ(Radians(180.0f)) * glScale(5., 5., 5.), passat);

  Mesh tire = ObjReader::fromFile("../assets/KIT_tire.obj");
  model->attach(T * glScale(0.65, 0.65, 0.65) * glTranslate(2.27, -1.17, -0.86), tire);  // front right
  model->attach(T * glScale(0.65, 0.65, 0.65) * glTranslate(2.27, 1.17, -0.86) * glRotateZ(Radians(180.0f)),
                tire);  // front left
  model->attach(T * glScale(0.65, 0.65, 0.65) * glTranslate(-2.05, -1.17, -0.86), tire);
  model->attach(T * glScale(0.65, 0.65, 0.65) * glTranslate(-2.05, 1.17, -0.86) * glRotateZ(Radians(180.0f)), tire);

  ui_.wCanvas->setRobotModel(model);
}

void VisualizerWindow::play(bool start) {
  if (start) {
    timer_.start();
    ui_.btnPlay->setChecked(true);
  } else {
    timer_.stop();
    ui_.btnPlay->setChecked(false);
  }

  updatePlaybackControls();
}

void VisualizerWindow::nextScan() {
  if (reader_ == nullptr) return;

  if (currentScanIdx_ < reader_->count() - 1) {
    ui_.sldTimeline->setValue(currentScanIdx_ + 1);
  } else {
    if (ui_.btnPlay->isChecked()) {
      play(false);
    }
  }

  updatePlaybackControls();
}

void VisualizerWindow::optimizeGraph() {
  Stopwatch::tic();
  fusion_->globallyOptimize();
  std::cout << "Global optimization took " << Stopwatch::toc() << std::endl;

  ui_.wCanvas->setOptimizedPoses(fusion_->getOptimizedPoses());
  ui_.wCanvas->updateGL();
}

void VisualizerWindow::reset() {
  if (reader_ != nullptr) {
    play(false);

    reader_->reset();

    if (fusion_ != nullptr) fusion_->reset();
    ui_.wCanvas->reset();
    //    setScan(0);
    if (ui_.sldTimeline->value() == 0) setScan(0);
    ui_.sldTimeline->setValue(0);

    uint32_t N = reader_->count();
    precomputed_.resize(N);
    oldPoses_.resize(N);

    for (uint32_t i = 0; i < N; ++i) precomputed_[i] = false;
    if (oldPoses_.size() > 0) oldPoses_[0] = Eigen::Matrix4f::Identity();

    acc_->clear();

    updatePlaybackControls();

    for (auto& it : ui_.wGraph->getAllTimeseries()) {
      it.second->clear();
    }

    ui_.wGraph->reset();
  }
}

void VisualizerWindow::updatePlaybackControls() {
  if (reader_ != nullptr) {
    ui_.btnPlay->setEnabled(true);
    ui_.btnReset->setEnabled(true);

    // maybe better to define separate up/down icons.
    if (ui_.btnPlay->isChecked())
      ui_.btnPlay->setIcon(QIcon::fromTheme("media-playback-pause"));
    else
      ui_.btnPlay->setIcon(QIcon::fromTheme("media-playback-start"));

    if (currentScanIdx_ < reader_->count() - 1)
      ui_.btnNext->setEnabled(true);
    else
      ui_.btnNext->setEnabled(false);
  } else {
    ui_.btnPlay->setEnabled(false);
    ui_.btnNext->setEnabled(false);
    ui_.btnReset->setEnabled(false);
  }
}

void VisualizerWindow::setScan(int idx) {
  if (reader_ == nullptr) return;
  if (static_cast<uint32_t>(idx) >= reader_->count()) return;  // ignore invalid calls.
  if (!reader_->isSeekable() && ((uint32_t)idx < currentScanIdx_)) return;

  uint32_t lastScanIdx = currentScanIdx_;
  currentScanIdx_ = idx;

  if (reader_->isSeekable()) reader_->seek(idx);
  reader_->read(currentLaserscan_);

  if (lastScanIdx > currentScanIdx_) {
    acc_->clear();
  }

  if (parameterUpdateNeeded_) {
    fusion_->setParameters(params_);

    // update timeseries thresholds.
    //    GraphWidget::Timeseries::Ptr ts = ui_.wGraph->getTimeseries("loop_relative_error_inlier");
    //    ts->removeAllThresholds();
    //    ts->addThreshold(params_["loop-residual-thresh"]);

    GraphWidget::Timeseries::Ptr ts = ui_.wGraph->getTimeseries("loop_relative_error_all");
    ts->removeAllThresholds();
    ts->addThreshold(params_["loop-residual-threshold"]);

    ts = ui_.wGraph->getTimeseries("loop_outlier_ratio");
    ts->removeAllThresholds();
    ts->addThreshold(params_["loop-outlier-threshold"]);

    ts = ui_.wGraph->getTimeseries("loop_valid_ratio");
    ts->removeAllThresholds();
    ts->addThreshold(params_["loop-valid-threshold"]);

    parameterUpdateNeeded_ = false;
  }

//  for (rv::ParameterList::const_iterator pit = params_.begin(); pit != params_.end(); ++pit) {
//    std::cout << "pit: " << *pit << std::endl;
//  }

  if (fusion_ != nullptr) {
    if (precomputed_[currentScanIdx_]) {
      fusion_->setCurrentPose(oldPoses_[currentScanIdx_]);
      fusion_->initialize(currentLaserscan_);
      fusion_->preprocess();
      fusion_->getCurrentFrame()->pose = oldPoses_[currentScanIdx_];
    } else {
      int32_t timestamp = fusion_->timestamp();

      fusion_->processScan(currentLaserscan_);
      oldPoses_[currentScanIdx_] = fusion_->getCurrentPose().cast<float>();
      precomputed_[currentScanIdx_] = true;

      if (currentScanIdx_ % history_stride_ == 0) {
        std::vector<rv::Point3f> pts = currentLaserscan_.points();
        if (currentLaserscan_.hasRemission()) {
          for (uint32_t i = 0; i < pts.size(); ++i) {
            pts[i].vec[3] = currentLaserscan_.remission(i);
          }
        }
        acc_->insert(timestamp, fusion_->getCurrentPose().cast<float>(), pts);
      }

      SurfelMapping::Stats stats = fusion_->getStatistics();

      statistics.push_back(stats);

      ui_.wGraph->getTimeseries("runtime")->insert(timestamp, stats["complete-time"]);
      ui_.wGraph->getTimeseries("loop_outlier_ratio")->insert(timestamp, stats["loop_outlier_ratio"]);
      ui_.wGraph->getTimeseries("loop_relative_error_all")->insert(timestamp, stats["loop_relative_error_all"]);
      //      ui_.wGraph->getTimeseries("num_iterations")->insert(timestamp, stats["num_iterations"]);
      ui_.wGraph->getTimeseries("loop_valid_ratio")->insert(timestamp, stats["valid_ratio"]);
      ui_.wGraph->getTimeseries("loop_outlier_ratio")->insert(timestamp, stats["outlier_ratio"]);
      ui_.wGraph->getTimeseries("increment_difference")->insert(timestamp, stats["increment_difference"]);
      //      ui_.wGraph->getTimeseries("icp_percentage")->insert(timestamp, stats["icp_percentage"]);

      if (fusion_->useLoopClosureCandidate()) ui_.wGraph->insertMarker(timestamp, Qt::green);

      ui_.wCanvas->setResidualMap(fusion_->getCurrentFrame()->residual_map);      
    }

    ui_.wCanvas->setCurrentFrame(currentScanIdx_, *fusion_->getCurrentFrame());
    ui_.wCanvas->setOldSurfelFrame(*fusion_->getOldSurfelMap());
    ui_.wCanvas->setNewSurfelFrame(*fusion_->getNewSurfelMap());
    ui_.wCanvas->setModelFrame(*fusion_->getCurrentModelFrame());

    std::vector<Eigen::Matrix4d> optimizedPoses = fusion_->getOptimizedPoses();
    ui_.wCanvas->setOptimizedPoses(optimizedPoses);

    ui_.wCanvas->setLoopClosurePose(fusion_->foundLoopClosureCandidate(), fusion_->useLoopClosureCandidate(),
                                    fusion_->getLoopClosurePoses(), fusion_->getNewMapPose(), fusion_->getOldMapPose());

    // update poses in accumulator.
    std::vector<int32_t>& timestamps = acc_->getTimestamps();
    std::vector<Eigen::Matrix4f>& acc_poses = acc_->getPoses();

    for (uint32_t i = 0; i < timestamps.size(); ++i) {
      if (timestamps[i] > -1) {
        acc_poses[i] = optimizedPoses[timestamps[i]].cast<float>();
      }
    }

    ui_.wGraph->setTimestamp(currentScanIdx_);
  }

  // triggers redraw:
  ui_.wCanvas->setLaserscan(currentLaserscan_);

  updatePlaybackControls();
  ui_.txtScanNumber->setText(QString::number(currentScanIdx_));

  ui_.wCanvas->setOdomPoses(fusion_->getIntermediateOdometryPoses());

  if (recording_) {
    std::string out_filename = outputDirectory_;
    out_filename += QString("/%1.png").arg((int)currentScanIdx_, 5, 10, (QChar)'0').toStdString();

    ui_.wCanvas->grabFrameBuffer().save(QString::fromStdString(out_filename));
    std::cout << "Writing to " << out_filename << std::endl;
  }

#ifdef QUERY_MEMORY_NV
  GLint totalMemory = 0, availableMemory = 0;
  glGetIntegerv(GL_GPU_MEM_INFO_TOTAL_AVAILABLE_MEM_NVX, &totalMemory);
  glGetIntegerv(GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX, &availableMemory);
  CheckGlError();
  ui_.pbGpuMemory->setValue(100 * float(availableMemory) / float(totalMemory));
#endif
}

void VisualizerWindow::updateParameters() {
//  std::cout << "updating params" << std::endl;
  params_.insert(FloatParameter("confidence_threshold", ui_.spinConfidenceThreshold->value()));
  params_.insert(FloatParameter("icp-max-distance", ui_.spinIcpMaxDistance->value()));
  params_.insert(FloatParameter("icp-max-angle", ui_.spinIcpMaxAngle->value()));
  params_.insert(IntegerParameter("max iterations", ui_.spinIcpMaxIterations->value()));
  params_.insert(BooleanParameter("initialize_identity", !ui_.chkWarmStart->isChecked()));

  params_.insert(BooleanParameter("filter_vertexmap", ui_.chkBilateralFiltering->isChecked()));
  params_.insert(BooleanParameter("use_filtered_vertexmap", ui_.chkFilterVertices->isChecked()));

  params_.insert(FloatParameter("bilateral_sigma_range", ui_.spinBilateralSigmaRange->value()));
  params_.insert(FloatParameter("bilateral_sigma_space", ui_.spinBilateralSigmaSpace->value()));
  params_.insert(FloatParameter("p_stable", ui_.spinProbStable->value()));
  params_.insert(FloatParameter("p_prior", ui_.spinProbPrior->value()));
  params_.insert(FloatParameter("sigma_angle", ui_.spinSigmaAngle->value()));
  params_.insert(FloatParameter("sigma_distance", ui_.spinSigmaDistance->value()));
  params_.insert(FloatParameter("map-max-distance", ui_.spinMapMaxDistance->value()));
  params_.insert(FloatParameter("map-max-angle", ui_.spinMapMaxAngle->value()));
  params_.insert(FloatParameter("factor", ui_.spinOptFactor->value()));
  params_.insert(StringParameter("weighting", ui_.cmbOptWeighting->currentText().toStdString()));
  params_.insert(IntegerParameter("weighting_scheme", ui_.cmbWeightingScheme->currentIndex()));
  params_.insert(IntegerParameter("averaging_scheme", ui_.cmbAveragingScheme->currentIndex()));
  params_.insert(BooleanParameter("update_always", ui_.chkUpdateAlways->isChecked()));

  params_.insert(FloatParameter("loop-residual-threshold", ui_.spinResidualThres->value()));
  params_.insert(FloatParameter("loop-outlier-threshold", ui_.spinOutlierThres->value()));
  params_.insert(FloatParameter("loop-valid-threshold", ui_.spinValidThres->value()));

  parameterUpdateNeeded_ = true;
}

void VisualizerWindow::triggerPreprocessing() {
  updateParameters();
  fusion_->setParameters(params_);
  parameterUpdateNeeded_ = false;
  fusion_->preprocess();
  ui_.wCanvas->setCurrentFrame(currentScanIdx_, *fusion_->getCurrentFrame());
  ui_.wCanvas->updateGL();
}

void VisualizerWindow::updateScanIndex() {
  bool converted = false;
  int32_t idx = ui_.txtScanNumber->text().toInt(&converted);
  if (converted) ui_.sldTimeline->setValue(idx);
}

void VisualizerWindow::renderMaps() {
  if (fusion_ != nullptr) {
    fusion_->getCurrentFrame()->vertex_map.save("current_vertexmap.ppm");
    fusion_->getCurrentFrame()->normal_map.save("current_normalmap.ppm");
    fusion_->getLastFrame()->vertex_map.save("last_vertexmap.ppm");
    fusion_->getLastFrame()->normal_map.save("last_normalmap.ppm");
    fusion_->getCurrentModelFrame()->vertex_map.save("model_vertexmap.png");
    fusion_->getCurrentModelFrame()->normal_map.save("model_normalmap.png");
    fusion_->getCurrentModelFrame()->semantic_map.save("model_semanticmap.ppm");
    fusion_->getCurrentFrame()->semantic_map.save("current_semanticmap.ppm");

    std::vector<Surfel> surfels = fusion_->getMap()->getAllSurfels();
    std::vector<Eigen::Matrix4d> poses = fusion_->getOptimizedPoses();
    std::vector<Surfel>::iterator surfel_it;
    std::ofstream points_file("points.txt");
    for ( surfel_it = surfels.begin(); surfel_it != surfels.end(); surfel_it++ )
    {
        Eigen::Matrix4d pose = poses[surfel_it->count];
        Eigen::Vector4d position(surfel_it->x, surfel_it->y, surfel_it->z, 1.0);
        Eigen::Vector4d pose_world = pose * position;
        points_file << pose_world.x() << " " << pose_world.y() << " " << pose_world.z() << " " << std::endl;
    }

    ui_.wCanvas->grabFrameBuffer().save(QString::fromStdString("current_view.png"));
  }
}

void VisualizerWindow::initializeGraph() {
  fusion_->initializeGraph();
  ui_.wCanvas->setOptimizedPoses(fusion_->getOptimizedPoses());
  ui_.wCanvas->updateGL();
}

void VisualizerWindow::savePoses() {
  QString save_path = "~";
  QString retValue = QFileDialog::getSaveFileName(this, "Save poses", save_path, "Pose file(*.txt)");

  if (!retValue.isNull() && fusion_ != nullptr) {
        Eigen::Matrix4f T_cam_velo = calib_["Tr"];
        Eigen::Matrix4f T_velo_cam = calib_["Tr"].inverse();

    std::ofstream out(retValue.toStdString());

    auto poses = fusion_->getOptimizedPoses();

    for (uint32_t i = 0; i < poses.size(); ++i) {
        Eigen::Matrix4f pose = T_cam_velo * poses[i].cast<float>() * T_velo_cam;
      for (uint32_t r = 0; r < 3; ++r) {
        for (uint32_t c = 0; c < 4; ++c) {
          out << ((r == 0 && c == 0) ? "" : " ") << pose(r, c);
        }
      }
      out << std::endl;
    }

    out.close();

    std::cout << "Saved poses." << std::endl;

    QString retValue1 = QFileDialog::getSaveFileName(this, "Save runtime", save_path, "runtime file(*.txt)");

    std::ofstream out_runtime(retValue1.toStdString());

    out_runtime << "# initialization preprocessing icp loop mapping" << std::endl;
    for (uint32_t i = 0; i < statistics.size(); ++i) {
      out_runtime << i << " ";
      out_runtime << statistics[i]["initialize-time"] << " ";
      out_runtime << statistics[i]["preprocessing-time"] << " ";
      out_runtime << statistics[i]["icp-time"] << " ";
      out_runtime << statistics[i]["loop-time"] << " ";
      out_runtime << statistics[i]["mapping-time"] << " ";
      out_runtime << statistics[i]["complete-time"] << std::endl;
    }
    out_runtime.close();
    std::cout << "Saved runtime." << std::endl;
  }
}
