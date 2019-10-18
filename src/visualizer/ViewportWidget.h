#ifndef SRC_VISUALIZER_VIEWPORTWIDGET_H_
#define SRC_VISUALIZER_VIEWPORTWIDGET_H_

#include <glow/GlBuffer.h>
#include <glow/GlProgram.h>
#include <glow/GlSampler.h>
#include <glow/GlTexture.h>
#include <glow/GlVertexArray.h>
#include <glow/util/RoSeCamera.h>
#include <rv/Laserscan.h>
#include <memory>
#include "core/Surfel.h"

#include <QtCore/QTimer>
#include <QtOpenGL/QGLWidget>

#include <glow/GlFramebuffer.h>
#include <rv/ParameterList.h>

#include <glow/glutil.h>

#include <rv/RingBuffer.h>
#include "core/Frame.h"
#include "opengl/Model.h"

#include "core/Posegraph.h"
#include "core/SurfelMap.h"
#include "util/ScanAccumulator.h"
#include "util/kitti_utils.h"

/**
 * \brief Viewport for point cloud rendering offering some mouse navigation.
 *
 *  \author behley
 *
 *  \author Xieyuanli Chen
 */
class ViewportWidget : public QGLWidget {
  Q_OBJECT
 public:
  ViewportWidget(QWidget* parent = 0, const QGLWidget* shareWidget = 0, Qt::WindowFlags f = 0);
  ~ViewportWidget();

  /** \brief initialize with given parameters. **/
  void initialize(const rv::ParameterList& params);

  void setLaserscan(const rv::Laserscan& scan);

  //    void setCurrentPose(const Eigen::Matrix4f& pose);
  void setIcpSteps(const std::vector<Eigen::Matrix4f>& pose);
  void setAssociationMap(const glow::GlTexture& tex);

  void setCurrentFrame(uint32_t t, const Frame& frame);
  void setModelFrame(const Frame& frame);

  void setPoseAdjustment(const Eigen::Matrix4f& pose);
  void setGroundtruth(const std::vector<Eigen::Matrix4f>& gt);

  void setRobotModel(const std::shared_ptr<Model>& model);

  void setCalibration(const KITTICalibration& calib);

  void setMap(const std::shared_ptr<SurfelMap>& map);

  void reset();

  void setScanAccumualator(const std::shared_ptr<ScanAccumulator>& acc);

  void setOldSurfelFrame(const Frame& frame);
  void setNewSurfelFrame(const Frame& frame);

  void setScanCount(uint32_t count);
  void setHistoryStride(int stride);

  void setOptimizedPoses(const std::vector<Eigen::Matrix4d>& poses);

  void setPosegraph(const Posegraph::ConstPtr& graph);

  void setLoopClosurePose(bool valid, bool used, const std::vector<Eigen::Matrix4f>& loopClosurePoses,
                          const Eigen::Matrix4f& loopClosurePose_new, const Eigen::Matrix4f& loopClosurePose_old);

  void setResidualMap(const glow::GlTextureRectangle& residualmap);

  void setOdomPoses(const std::vector<Eigen::Matrix4d>& poses);

 public slots:

  // --- visualization options --
  void setDrawingOption(const std::string& name, bool value);

  void setPointSize(int pointSize);
  void setPointColorMode(int idx);
  void setDepthMapColorMode(int idx);
  void setSurfelColorMode(int idx);
  void setIcpStep(int idx);
  void setHistorySize(int size);

  void setConfidenceThreshold(double value);

  /** \brief set the color map used for semantic mapping **/
  void setColorMap(std::map<uint32_t, semantic_color> colormap);

 protected slots:
  void redraw();

 protected:
  bool initContext();
  void initializePrograms();
  void initializeVertexBuffers();

  void mousePressEvent(QMouseEvent* event);
  void mouseReleaseEvent(QMouseEvent* event);
  void mouseMoveEvent(QMouseEvent* event);

  void initializeGL();
  void resizeGL(int w, int h);
  void paintGL();

  float rgb2float(float r, float g, float b);

  glow::GlCamera::KeyboardModifier resolveKeyboardModifier(Qt::KeyboardModifiers modifiers);
  glow::GlCamera::MouseButton resolveMouseButton(Qt::MouseButtons button);

  void updateVertexBuffers(uint32_t idx, const rv::Laserscan& scan);

  /** \brief set uniforms in programs **/
  void updateProgramUniforms();

  Eigen::Matrix4f getBirdsEyeView();

  bool contextInitialized_;

  glow::GlProgram pointProgram_;
  glow::GlProgram coloredPointProgram_;
  glow::GlProgram prgDrawDepthImage_;
  glow::GlProgram prgDrawNormalMap_;

  glow::GlProgram prgDrawSemanticMap_;
  glow::GlProgram prgDrawSemanticRangeMap_;

  glow::GlProgram prgDrawVertexMap3d_;
  glow::GlProgram prgDrawNormalMap3d_;
  glow::GlProgram prgDrawResidualMap_;

  glow::GlProgram prgDrawRobotModel_;

  glow::GlProgram prgDrawPosegraph_;

  // "double buffered" vertex arrays...allows to set the points while rendering in a different thread.

  glow::GlBuffer<float> coordAxisVBO_{glow::BufferTarget::ARRAY_BUFFER,
                                      glow::BufferUsage::STATIC_DRAW};  // x, y, z, color(float)
  glow::GlVertexArray coordAxisVAO_;

  // TODO: add mutexes?
  int32_t currentBuffer_{0};
  glow::GlVertexArray pointVAOs_[2];
  glow::GlBuffer<rv::Point3f> pointVBOs_[2]{{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW},
                                            {glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW}};
  glow::GlBuffer<float> pointRemissions_[2]{{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW},
                                            {glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW}};

  // vertices for image:
  glow::GlVertexArray vao_imageCoords_;
  glow::GlBuffer<glow::vec2> vbo_imageCoords_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::STATIC_DRAW};

  // depth images etc.
  glow::GlVertexArray vao_no_point_;

  // drawing options:
  int32_t pointSize_{1};
  std::map<std::string, bool> drawing_options_;

  glow::GlUniform<Eigen::Matrix4f> mvp_{"mvp", Eigen::Matrix4f::Identity()};
  glow::GlUniform<Eigen::Matrix4f> mvp_inv_t_{"mvp_inv_t", Eigen::Matrix4f::Identity()};
  glow::GlUniform<int32_t> pointColorMode_{"pointColorMode", 1};  // 0 - none, 1 - remission, 2 - depth, ...
  glow::GlUniform<int32_t> depthMapColorMode_{"colorMode", 0};
  glow::GlUniform<bool> markInvalidDepthValues_{"markInvalidDepthValues", false};
  glow::GlUniform<int32_t> uniformPointSize_{"pointSize", 1};

  Eigen::Matrix4f model_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f view_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f projection_{Eigen::Matrix4f::Identity()};

  QTimer timer_;

  glow::RoSeCamera camera_;
  Eigen::Matrix4f conversion_;

  std::vector<Eigen::Matrix4f> icp_poses_;
  int32_t icp_iteration_{0};
  uint32_t timestep_{0};
  uint32_t scanCount_{0};

  std::shared_ptr<Frame> currentFrame_, lastFrame_;
  std::vector<Eigen::Matrix4f> poses_;

  glow::GlVertexArray vao_history_;

  Eigen::Matrix4f poseAdjustment_{Eigen::Matrix4f::Identity()};

  std::vector<Eigen::Matrix4f> groundtruth_;

  std::shared_ptr<Model> robot_;

  int32_t currentIndexBuffer_{0};
  glow::GlVertexArray segmentVAOs_[2];
  glow::GlBuffer<uint32_t> segmentIndexes_[2]{
      {glow::BufferTarget::ELEMENT_ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW},
      {glow::BufferTarget::ELEMENT_ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW}};

  KITTICalibration calib_;

  std::shared_ptr<ScanAccumulator> acc_;
  uint32_t historySize_{10}, historyStride_{1};

  glow::GlSampler linearSampler_;

  std::shared_ptr<SurfelMap> map_;
  SurfelMapVisualOptions opts_;

  std::shared_ptr<Frame> model_frame_;

  std::shared_ptr<Frame> old_surfel_frame_;
  std::shared_ptr<Frame> new_surfel_frame_;

  std::vector<Eigen::Matrix4d> optimizedPoses_;
  Posegraph::ConstPtr posegraph_;

  bool loopClosure_{false};
  bool loopClosureUsed_{false};
  std::vector<Eigen::Matrix4f> loopClosurePoses_;
  Eigen::Matrix4f lcPoseOld_;
  Eigen::Matrix4f lcPoseNew_;

  glow::GlTextureRectangle residualMap_{1, 1, glow::TextureFormat::RGBA};

  std::vector<Eigen::Matrix4d> odom_poses_;

  // semantic color map
  glow::GlTexture color_map_1dtex_;

  typedef std::tuple< unsigned char, unsigned char , unsigned char> semantic_label;
  std::map<uint32_t, semantic_label> color_map_;
};

#endif /* SRC_VISUALIZER_VIEWPORTWIDGET_H_ */
