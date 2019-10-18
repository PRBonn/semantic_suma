/*
 * ViewportWidget.cpp
 *
 *  Created on: Apr 14, 2016
 *      Author: behley
 *
 *      Author: Xieyuanli Chen
 */

#include "ViewportWidget.h"
#include <QtCore/QTime>
#include <QtGui/QMouseEvent>
#include <iostream>

#include <glow/GlColor.h>
#include <glow/GlQuery.h>
#include <glow/GlShader.h>
#include <glow/GlState.h>
#include <glow/ScopedBinder.h>
#include <glow/colormaps.h>
#include <glow/glutil.h>
#include <glow/util/RandomColorGenerator.h>
#include <rv/Math.h>

#include "opengl/MatrixStack.h"

using namespace rv;
using namespace glow;

ViewportWidget::ViewportWidget(QWidget* parent, const QGLWidget* shareWidget, Qt::WindowFlags f)
    : QGLWidget(parent, shareWidget, f), contextInitialized_(initContext()),
      color_map_1dtex_(260, TextureFormat::RGB_FLOAT){
  if (!contextInitialized_) throw std::runtime_error("OpenGL context initialization failed!");
  initializePrograms();
  initializeVertexBuffers();

  lastFrame_ = std::make_shared<Frame>(0, 0);
  currentFrame_ = std::make_shared<Frame>(0, 0);

  linearSampler_.setMagnifyingOperation(TexMagOp::LINEAR);
  linearSampler_.setMinifyingOperation(TexMinOp::LINEAR);
  linearSampler_.setWrapOperation(TexWrapOp::CLAMP_TO_BORDER, TexWrapOp::CLAMP_TO_BORDER);

  connect(&timer_, SIGNAL(timeout()), this, SLOT(redraw()));

  conversion_ = RoSe2GL::matrix;
}

ViewportWidget::~ViewportWidget() {}

void ViewportWidget::initialize(const rv::ParameterList& params) {
  uint32_t width = params["data_width"];
  uint32_t height = params["data_height"];
  currentFrame_ = std::make_shared<Frame>(width, height);
  lastFrame_ = std::make_shared<Frame>(width, height);

  // "midpoints" of the texture pixels in [0, 1] x [0, 1]
  std::vector<vec2> img_coords;
  img_coords.reserve(width * height);

  for (uint32_t x = 0; x < width; ++x) {
    for (uint32_t y = 0; y < height; ++y) {
      img_coords.push_back(vec2((x + 0.5f), (y + 0.5f)));
    }
  }

  vbo_imageCoords_.assign(img_coords);

  // initializing the drawing parameters.
  drawing_options_["coordinate axis"] = true;
  drawing_options_["current points"] = false;
  drawing_options_["global model"] = false;
  drawing_options_["depth map"] = false;
  drawing_options_["normal map"] = false;
  drawing_options_["semantic map"] = true;
  drawing_options_["normal map 3d"] = false;
  drawing_options_["vertex map"] = false;
  drawing_options_["validity map"] = false;
  drawing_options_["current surfels"] = true;
  drawing_options_["surfel coords"] = false;
  drawing_options_["pose estimate"] = true;
  drawing_options_["poses"] = false;
  drawing_options_["icp step"] = false;
  drawing_options_["correspondences"] = false;
  drawing_options_["draw robot"] = true;
  drawing_options_["residual map"] = false;
  drawing_options_["draw surfel points"] = false;
  drawing_options_["draw history"] = false;
  drawing_options_["draw odom poses"] = true;

  drawing_options_["last frame"] = false;
  drawing_options_["pose graph"] = true;

  drawing_options_["history height"] = false;

  model_frame_ = std::make_shared<Frame>(params["model_width"], params["model_height"]);
  old_surfel_frame_ = std::make_shared<Frame>(params["model_width"], params["model_height"]);
  new_surfel_frame_ = std::make_shared<Frame>(params["model_width"], params["model_height"]);

  // init color_map
  color_map_1dtex_.setMinifyingOperation(TexMinOp::NEAREST);
  color_map_1dtex_.setMagnifyingOperation(TexMagOp::NEAREST);
  color_map_1dtex_.setWrapOperation(TexWrapOp::CLAMP_TO_BORDER, TexWrapOp::CLAMP_TO_BORDER);
}

bool ViewportWidget::initContext() {
  // enabling core profile
  QGLFormat corefmt;
  corefmt.setVersion(5, 0);  // getting highest compatible format...
  corefmt.setProfile(QGLFormat::CoreProfile);
  setFormat(corefmt);

  // version info.
  QGLFormat fmt = this->format();
  std::cout << "OpenGL Context Version " << fmt.majorVersion() << "." << fmt.minorVersion() << " "
            << ((fmt.profile() == QGLFormat::CoreProfile) ? "core profile" : "compatibility profile") << std::endl;

  makeCurrent();
  inititializeGLEW();

  return true;
}

void ViewportWidget::initializePrograms() {
  makeCurrent();

  pointProgram_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/laserscan.vert"));
  pointProgram_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/laserscan.frag"));
  pointProgram_.link();

  coloredPointProgram_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/coloredvertices.vert"));
  coloredPointProgram_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/coloredvertices.frag"));
  coloredPointProgram_.link();

  prgDrawDepthImage_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/empty.vert"));
  prgDrawDepthImage_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shader/quad.geom"));
  prgDrawDepthImage_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/draw_depthimg.frag"));
  prgDrawDepthImage_.link();

  prgDrawNormalMap_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/empty.vert"));
  prgDrawNormalMap_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shader/quad.geom"));
  prgDrawNormalMap_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/draw_normalmap.frag"));
  prgDrawNormalMap_.link();

  prgDrawSemanticMap_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/empty.vert"));
  prgDrawSemanticMap_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shader/quad.geom"));
  prgDrawSemanticMap_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/draw_semanticmap.frag"));
  prgDrawSemanticMap_.link();

  prgDrawSemanticRangeMap_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/empty.vert"));
  prgDrawSemanticRangeMap_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shader/quad.geom"));
  prgDrawSemanticRangeMap_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/draw_semantic_colormap.frag"));
  prgDrawSemanticRangeMap_.link();

  prgDrawVertexMap3d_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/draw_vertexmap3d.vert"));
  prgDrawVertexMap3d_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/passthrough.frag"));
  prgDrawVertexMap3d_.link();

  prgDrawVertexMap3d_.setUniform(GlUniform<int>("texVertexMap", 0));
  prgDrawVertexMap3d_.setUniform(GlUniform<int>("texResidualMap", 1));

  prgDrawNormalMap3d_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/draw_normalmap3d.vert"));
  prgDrawNormalMap3d_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shader/draw_normalmap3d.geom"));
  prgDrawNormalMap3d_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/passthrough.frag"));
  prgDrawNormalMap3d_.link();

  prgDrawNormalMap3d_.setUniform(GlUniform<int>("texVertexMap", 0));
  prgDrawNormalMap3d_.setUniform(GlUniform<int>("texNormalMap", 1));

  prgDrawRobotModel_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/draw_mesh.vert"));
  prgDrawRobotModel_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/draw_mesh.frag"));
  prgDrawRobotModel_.link();

  // sun comes roughly from above...
  prgDrawRobotModel_.setUniform(GlUniform<vec4>("lights[0].position", vec4(0, 0, -1, 0)));
  prgDrawRobotModel_.setUniform(GlUniform<vec3>("lights[0].ambient", vec3(.9, .9, .9)));
  prgDrawRobotModel_.setUniform(GlUniform<vec3>("lights[0].diffuse", vec3(.9, .9, .9)));
  prgDrawRobotModel_.setUniform(GlUniform<vec3>("lights[0].specular", vec3(.9, .9, .9)));

  // more evenly distributed sun light...
  std::vector<vec4> dirs = {vec4(1, -1, 1, 0), vec4(-1, -1, 1, 0), vec4(1, -1, -1, 0), vec4(-1, -1, -1, 0)};
  vec3 indirect_intensity = vec3(.1, .1, .1);

  for (uint32_t i = 0; i < 4; ++i) {
    std::stringstream light_name;
    light_name << "lights[" << (i + 1) << "]";

    prgDrawRobotModel_.setUniform(GlUniform<vec4>(light_name.str() + ".position", dirs[i]));
    prgDrawRobotModel_.setUniform(GlUniform<vec3>(light_name.str() + ".ambient", indirect_intensity));
    prgDrawRobotModel_.setUniform(GlUniform<vec3>(light_name.str() + ".diffuse", indirect_intensity));
    prgDrawRobotModel_.setUniform(GlUniform<vec3>(light_name.str() + ".specular", indirect_intensity));
  }

  prgDrawRobotModel_.setUniform(GlUniform<int>("num_lights", 5));

  prgDrawResidualMap_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/empty.vert"));
  prgDrawResidualMap_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shader/quad.geom"));
  prgDrawResidualMap_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/draw_residuals.frag"));
  prgDrawResidualMap_.link();

  prgDrawPosegraph_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/empty.vert"));
  prgDrawPosegraph_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shader/draw_posegraph_edge.geom"));
  prgDrawPosegraph_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/passthrough.frag"));
  prgDrawPosegraph_.link();

  prgDrawSemanticRangeMap_.setUniform(GlUniform<int32_t>("color_map", 1));
}

void ViewportWidget::setScanAccumualator(const std::shared_ptr<ScanAccumulator>& acc) {
  acc_ = acc;

  vao_history_.bind();
  vao_history_.setVertexAttribute(0, acc_->getVBO(), 4, AttributeType::FLOAT, false, 4 * sizeof(float), 0);
  vao_history_.enableVertexAttribute(0);
  vao_history_.release();
}

void ViewportWidget::setMap(const std::shared_ptr<SurfelMap>& map) {
  map_ = map;
}

void ViewportWidget::setHistorySize(int size) {
  historySize_ = size;
  updateGL();
}

float ViewportWidget::rgb2float(float r, float g, float b) {
  int32_t rgb = int32_t(round(r * 255.0f));
  rgb = (rgb << 8) + int32_t(round(g * 255.0f));
  rgb = (rgb << 8) + int32_t(round(b * 255.0f));

  return float(rgb);
}

void ViewportWidget::reset() {
  poses_.clear();
}

void ViewportWidget::initializeVertexBuffers() {
  makeCurrent();

  {
    // initialize coordinate axis buffer
    coordAxisVAO_.bind();

    float red = rgb2float(1.0f, 0.0f, 0.0f);
    float green = rgb2float(0.0f, 1.0f, 0.0f);
    float blue = rgb2float(0.0f, 0.0f, 1.0f);

    std::vector<float> cverts(
        {0, 0, 0, red, 1, 0, 0, red, 0, 0, 0, green, 0, 1, 0, green, 0, 0, 0, blue, 0, 0, 1, blue});
    coordAxisVBO_.assign(cverts);
    coordAxisVAO_.setVertexAttribute(0, coordAxisVBO_, 4, AttributeType::FLOAT, false, 4 * sizeof(GLfloat), 0);
    coordAxisVAO_.enableVertexAttribute(0);

    coordAxisVAO_.release();
  }

  // initialize both vertex arrays and buffers.
  for (uint32_t i = 0; i <= 1; ++i) {
    // What happens with Element buffers?

    ScopedBinder<GlVertexArray> vao(pointVAOs_[i]);

    pointVBOs_[i].reserve(150000);  // enough space for a complete Velodyne scan.

    pointVAOs_[i].setVertexAttribute(0, pointVBOs_[i], 4, AttributeType::FLOAT, false, 4 * sizeof(GLfloat), 0);
    pointVAOs_[i].enableVertexAttribute(0);

    pointRemissions_[i].reserve(150000);

    pointVAOs_[i].setVertexAttribute(1, pointRemissions_[i], 1, AttributeType::FLOAT, false, sizeof(GLfloat), 0);
    pointVAOs_[i].enableVertexAttribute(1);
  }

  // for each image coordinate we need a vertex.
  {
    vao_imageCoords_.bind();
    vao_imageCoords_.setVertexAttribute(0, vbo_imageCoords_, 2, AttributeType::FLOAT, false, 2 * sizeof(GLfloat), 0);
    vao_imageCoords_.enableVertexAttribute(0);
    vao_imageCoords_.release();
  }

  CheckGlError();
// check if everything is unbound:
#ifdef DEBUG_GL
  GLint id = 0;
  glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &id);
  assert(id == 0);
  glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &id);
  assert(id == 0);
  CheckGlError();
  std::cout << "everything healthy here." << std::endl;
#endif
}

void ViewportWidget::updateVertexBuffers(uint32_t idx, const Laserscan& scan) {
  pointVBOs_[idx].assign(scan.points());

  if (scan.hasRemission()) {
    pointRemissions_[idx].assign(scan.remissions());
  } else {
    // ensure that there are some values available.
    pointRemissions_[idx].assign(std::vector<float>(scan.size(), 0.0f));
  }
}

void ViewportWidget::setLaserscan(const rv::Laserscan& scan) {
  makeCurrent();

  if (currentBuffer_ == 0) {
    updateVertexBuffers(1, scan);
    currentBuffer_ = 1;  // update buffer index after copying.
  } else {
    updateVertexBuffers(0, scan);
    currentBuffer_ = 0;  // update buffer index after copying.
  }

  CheckGlError();

  updateGL();  // trigger repaint.
}

void ViewportWidget::setRobotModel(const std::shared_ptr<Model>& model) {
  robot_ = model;
  updateGL();
}

void ViewportWidget::redraw() {
  updateGL();
}

void ViewportWidget::setDrawingOption(const std::string& name, bool value) {
  drawing_options_[name] = value;

  //  std::cout << "setting " << name << " to " << (value ? "true" : "false") << std::endl;

  if (name == "follow pose" && drawing_options_["pose estimate"]) {
    //    std::cout << "before \n" << camera_.matrix() << std::endl;
    if (value == false)  // currently following, but now defollowing:
    {
      camera_.setMatrix(camera_.matrix() * conversion_ * currentFrame_->pose.inverse() * conversion_.inverse());
    } else {
      camera_.setMatrix(camera_.matrix() * conversion_ * currentFrame_->pose * conversion_.inverse());
    }
    //    std::cout << "after \n" << camera_.matrix() << std::endl;
  } else if (name == "updated surfels") {
    opts_.drawUpdatedSurfels = value;
  } else if (name == "rendered surfels") {
    opts_.drawRenderedSurfels = value;
  } else if (name == "backface culling") {
    opts_.backfaceCulling = value;
  } else if (name == "show submaps") {
    opts_.drawSubmaps = value;
  } else if (name == "draw surfel points") {
    opts_.drawPoints = value;
  }

  updateProgramUniforms();
  updateGL();
}

void ViewportWidget::setConfidenceThreshold(double value) {
  opts_.confidence_threshold = value;
  updateGL();
}

void ViewportWidget::updateProgramUniforms() {
  markInvalidDepthValues_ = drawing_options_["mark invalid depths"];
  prgDrawDepthImage_.setUniform(markInvalidDepthValues_);
}

void ViewportWidget::setPointSize(int pointSize) {
  pointSize_ = pointSize;
  updateGL();
}

void ViewportWidget::setPointColorMode(int idx) {
  pointColorMode_ = idx;
  pointProgram_.setUniform(pointColorMode_);

  updateGL();
}

void ViewportWidget::setDepthMapColorMode(int idx) {
  depthMapColorMode_ = idx;
  std::cout << "depthMapColorMode = " << idx << std::endl;
  prgDrawDepthImage_.setUniform(depthMapColorMode_);
  prgDrawVertexMap3d_.setUniform(depthMapColorMode_);

  updateGL();
}

void ViewportWidget::setSurfelColorMode(int idx) {
  opts_.colorMode = idx;

  updateGL();
}

void ViewportWidget::setCalibration(const KITTICalibration& calib) {
  calib_ = calib;
}

void ViewportWidget::setCurrentFrame(uint32_t t, const Frame& frame) {
  lastFrame_.swap(currentFrame_);
  currentFrame_->copy(frame);

  timestep_ = t;
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  if (calib_.exists("Tr") && calib_.exists("P0")) {
    T = calib_["Tr"];
  }

  if (t >= poses_.size())
    poses_.push_back(frame.pose);  // insertation.
  else
    poses_[t] = frame.pose;  // update.

  CheckGlError();
}

void ViewportWidget::setModelFrame(const Frame& frame) {
  if (model_frame_ == nullptr) return;

  model_frame_->copy(frame);
}

void ViewportWidget::setOldSurfelFrame(const Frame& frame) {
  if (old_surfel_frame_ == nullptr) return;

  old_surfel_frame_->copy(frame);
}

void ViewportWidget::setNewSurfelFrame(const Frame& frame) {
  if (new_surfel_frame_ == nullptr) return;

  new_surfel_frame_->copy(frame);
}

void ViewportWidget::setIcpStep(int idx) {
  icp_iteration_ = idx;
  updateGL();
}

void ViewportWidget::setIcpSteps(const std::vector<Eigen::Matrix4f>& pose) {
  icp_poses_ = pose;
}

void ViewportWidget::setOptimizedPoses(const std::vector<Eigen::Matrix4d>& poses) {
  optimizedPoses_ = poses;
}

void ViewportWidget::setLoopClosurePose(bool valid, bool used, const std::vector<Eigen::Matrix4f>& loopClosurePoses,
                                        const Eigen::Matrix4f& loopClosurePose_new,
                                        const Eigen::Matrix4f& loopClosurePose_old) {
  loopClosure_ = valid;
  loopClosureUsed_ = used;
  loopClosurePoses_ = loopClosurePoses;
  lcPoseNew_ = loopClosurePose_new;
  lcPoseOld_ = loopClosurePose_old;
}

void ViewportWidget::setResidualMap(const glow::GlTextureRectangle& residualmap) {
  residualMap_ = residualmap;
}

void ViewportWidget::setOdomPoses(const std::vector<Eigen::Matrix4d>& poses) {
  odom_poses_ = poses;
}

void ViewportWidget::initializeGL() {
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_LINE_SMOOTH);

  camera_.lookAt(5.0f, 5.0f, 5.0f, 0.0f, 0.0f, 0.0f);
}

void ViewportWidget::resizeGL(int w, int h) {
  glViewport(0, 0, w, h);

  // set projection matrix
  float fov = Math::deg2rad(45.0f);
  float aspect = float(w) / float(h);

  projection_ = glPerspective(fov, aspect, 0.1f, 10000.0f);
}

Eigen::Vector4f min(const Eigen::Vector4f& a, const Eigen::Vector4f& b) {
  float x = std::min(a.x(), b.x());
  float y = std::min(a.y(), b.y());
  float z = std::min(a.z(), b.z());
  float w = std::min(a.w(), b.w());

  return Eigen::Vector4f({x, y, z, w});
}

Eigen::Vector4f max(const Eigen::Vector4f& a, const Eigen::Vector4f& b) {
  float x = std::max(a.x(), b.x());
  float y = std::max(a.y(), b.y());
  float z = std::max(a.z(), b.z());
  float w = std::max(a.w(), b.w());

  return Eigen::Vector4f({x, y, z, w});
}

Eigen::Matrix4f ViewportWidget::getBirdsEyeView() {
  RoSeCamera camera;
  Eigen::Vector4f minimum = Eigen::Vector4f::Zero(), maximum = Eigen::Vector4f::Zero(),
                  cam_pos = Eigen::Vector4f::Zero();

  for (uint32_t i = 0; i <= timestep_ && i < poses_.size(); ++i) {
    Eigen::Vector4f pos = poses_[i].col(3);
    // todo replace by pcp::bounds3f
    if (i == 0) minimum = maximum = pos;
    Eigen::Vector4f extent(100, 100, 0, 1);
    minimum = min(minimum, pos - extent);
    maximum = max(maximum, pos + extent);
  }

  float w = std::max(std::abs(maximum.y() - minimum.y()), 10.0f);
  float h = std::max(std::abs(maximum.x() - minimum.x()), 10.0f);
  float fov = Math::deg2rad(45.0f);
  float z = 0.0f;

  if (width() > height()) {
    if (w > h)
      z = 0.5 * w / std::tan(0.5 * fov);
    else
      z = 0.5 * h / std::tan(0.5 * fov);
  } else {
    if (w > h)
      z = 0.5 * w / std::tan(0.5 * fov);
    else
      z = 0.5 * h / std::tan(0.5 * fov);
  }

  //  z = 100;

  Eigen::Vector4f mid({minimum.x() + 0.5f * h, minimum.y() + 0.5f * w, 0, 1});
  cam_pos = Eigen::Vector4f(mid.x(), mid.y(), z, 1);

  cam_pos = conversion_ * cam_pos;
  mid = conversion_ * mid;

  camera.lookAt(cam_pos.x(), cam_pos.y(), cam_pos.z(), mid.x(), mid.y(), mid.z());

  return glRotateZ(radians(180)) * camera.matrix();
}

void ViewportWidget::paintGL() {
  glEnable(GL_DEPTH_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPointSize(pointSize_);

  // update models rotation using the current time:

  model_ = Eigen::Matrix4f::Identity();
  view_ = camera_.matrix();

  if (drawing_options_["birds eye view"]) {
    view_ = getBirdsEyeView();
  }

  Eigen::Vector4f view_pos = conversion_.inverse() * camera_.getPosition();

  if (drawing_options_["pose estimate"]) {
    model_ = currentFrame_->pose;
  }

  if (drawing_options_["pose estimate"] && drawing_options_["groundtruth"] && drawing_options_["last frame"]) {
    if (groundtruth_.size() > timestep_ && timestep_ > 0) {
      model_ = lastFrame_->pose * groundtruth_[timestep_ - 1].inverse() * groundtruth_[timestep_];
    }
  }

  if (drawing_options_["follow pose"] && drawing_options_["pose estimate"]) {
    //    view_ = view_ * conversion_ * currentFrame_->pose.inverse() * conversion_.inverse();

    RoSeCamera dummy;
    dummy.setMatrix(conversion_ * currentFrame_->pose.inverse() * conversion_.inverse());
    view_ = view_ * dummy.matrix();
  }

  //  std::cout << "view: \n " << view_ << std::endl;

  // draw coordinate axis
  if (drawing_options_["coordinate axis"]) {
    mvp_ = projection_ * view_ * conversion_;

    ScopedBinder<GlVertexArray> vao_binder(coordAxisVAO_);
    ScopedBinder<GlProgram> program_binder(coloredPointProgram_);

    coloredPointProgram_.setUniform(mvp_);

    glDrawArrays(GL_LINES, 0, 6);
  }

  if (drawing_options_["poses"]) {
    ScopedBinder<GlVertexArray> vao_binder(coordAxisVAO_);
    ScopedBinder<GlProgram> program_binder(coloredPointProgram_);
    Eigen::Matrix4f scale = glScale(0.5, 0.5, 0.5);
    for (uint32_t i = 0; i <= timestep_ && i < poses_.size(); ++i) {
      mvp_ = projection_ * view_ * conversion_ * poses_[i] * scale;
      coloredPointProgram_.setUniform(mvp_);
      glDrawArrays(GL_LINES, 0, 6);
    }
  }

  if (drawing_options_["pose graph"]) {
    ScopedBinder<GlVertexArray> vao_binder(coordAxisVAO_);
    ScopedBinder<GlProgram> program_binder(coloredPointProgram_);
    coloredPointProgram_.setUniform(GlUniform<bool>("useCustomColor", true));
    coloredPointProgram_.setUniform(GlUniform<GlColor>("customColor", GlColor::GREEN));
    Eigen::Matrix4f scale = glScale(0.25, 0.25, 0.25);
    for (uint32_t i = 0; i <= timestep_ && i < optimizedPoses_.size(); ++i) {
      mvp_ = projection_ * view_ * conversion_ * optimizedPoses_[i].cast<float>() * scale;
      coloredPointProgram_.setUniform(mvp_);
      glDrawArrays(GL_LINES, 0, 6);
    }
    coloredPointProgram_.setUniform(GlUniform<bool>("useCustomColor", false));  // rest.
  }

  if (drawing_options_["pose graph"] && posegraph_ != nullptr) {
    ScopedBinder<GlVertexArray> vao_binder(vao_no_point_);
    ScopedBinder<GlProgram> program_binder(prgDrawPosegraph_);
    //      coloredPointProgram_.setUniform(GlUniform<bool>("useCustomColor", true));
    //      coloredPointProgram_.setUniform(GlUniform<GlColor>("customColor", GlColor::GREEN));
    const std::vector<Posegraph::Edge>& edges = posegraph_->getEdges();
    //    std::cout << "drawing " << edges.size() << " edges." << std::endl;

    mvp_ = projection_ * view_ * conversion_;
    prgDrawPosegraph_.setUniform(mvp_);

    for (const Posegraph::Edge& edge : edges) {
      prgDrawPosegraph_.setUniform(GlUniform<Eigen::Matrix4f>("from", posegraph_->pose(edge.from).cast<float>()));
      prgDrawPosegraph_.setUniform(GlUniform<Eigen::Matrix4f>("to", posegraph_->pose(edge.to).cast<float>()));
      prgDrawPosegraph_.setUniform(GlUniform<Eigen::Matrix4f>("measurement", edge.measurement.cast<float>()));

      glDrawArrays(GL_POINTS, 0, 1);
    }
  }

  if (drawing_options_["groundtruth"]) {
    // usage of instancing would be better!
    ScopedBinder<GlVertexArray> vao_binder(coordAxisVAO_);
    ScopedBinder<GlProgram> program_binder(coloredPointProgram_);
    Eigen::Matrix4f scale = glScale(0.5, 0.5, 0.5);
    coloredPointProgram_.setUniform(GlUniform<bool>("useCustomColor", true));
    coloredPointProgram_.setUniform(GlUniform<GlColor>("customColor", GlColor::GOLD));
    for (uint32_t i = 0; i <= timestep_ && i < groundtruth_.size(); ++i) {
      mvp_ = projection_ * view_ * conversion_ * groundtruth_[i] * scale;
      coloredPointProgram_.setUniform(mvp_);
      glDrawArrays(GL_LINES, 0, 6);
    }
    coloredPointProgram_.setUniform(GlUniform<bool>("useCustomColor", false));  // rest.
  }

  if (drawing_options_["draw odom poses"]) {
    if (timestep_ > 0) {
      glLineWidth(2.0);
      ScopedBinder<GlVertexArray> vao_binder(coordAxisVAO_);
      ScopedBinder<GlProgram> program_binder(coloredPointProgram_);
      Eigen::Matrix4f scale = glScale(0.2, 0.2, 0.2);
      coloredPointProgram_.setUniform(GlUniform<bool>("useCustomColor", true));
      coloredPointProgram_.setUniform(GlUniform<GlColor>("customColor", GlColor::PINK));
      for (uint32_t i = 0; i < odom_poses_.size(); ++i) {
        mvp_ = projection_ * view_ * conversion_ * poses_[timestep_ - 1] * odom_poses_[i].cast<float>() * scale;
        coloredPointProgram_.setUniform(mvp_);
        glDrawArrays(GL_LINES, 0, 6);
      }
      coloredPointProgram_.setUniform(GlUniform<bool>("useCustomColor", false));  // rest.
      glLineWidth(1.0);
    }
  }

  if (drawing_options_["pose estimate"] && drawing_options_["icp steps"] && icp_poses_.size() > 0) {
    model_ = icp_poses_[icp_iteration_ % icp_poses_.size()].matrix() * lastFrame_->pose;
  }

  if (drawing_options_["increment pose"]) {
    model_ = poseAdjustment_ * model_;
    model_ = poseAdjustment_;
  }

  mvp_ = projection_ * view_ * conversion_ * model_;
  mvp_inv_t_ = projection_ * view_.inverse().transpose() * conversion_ * model_.inverse().transpose();

  // check what to paint.
  if (drawing_options_["current points"]) {
    ScopedBinder<GlVertexArray> vao_binder(pointVAOs_[currentBuffer_]);
    ScopedBinder<GlProgram> program_binder(pointProgram_);

    //    if (timestep_ < 250)
    //      pointProgram_.setUniform(GlUniform<GlColor>("customColor", GlColor::ORANGE));
    //    else
    //      pointProgram_.setUniform(GlUniform<GlColor>("customColor", GlColor::BLUE));
    //    pointProgram_.setUniform(GlUniform<int>("pointColorMode", 6));

    pointProgram_.setUniform(mvp_);

    glDrawArrays(GL_POINTS, 0, pointVBOs_[currentBuffer_].size());
  }

  if (drawing_options_["show lc poses"] && loopClosure_) {
    ScopedBinder<GlVertexArray> vao_binder(coordAxisVAO_);
    ScopedBinder<GlProgram> program_binder(coloredPointProgram_);

    coloredPointProgram_.setUniform(GlUniform<bool>("useCustomColor", true));

    coloredPointProgram_.setUniform(GlUniform<GlColor>("customColor", GlColor::GREEN));

    Eigen::Matrix4f scale = glScale(0.5, 0.5, 0.5);

    mvp_ = projection_ * view_ * conversion_ * lcPoseNew_ * scale;
    coloredPointProgram_.setUniform(mvp_);
    glDrawArrays(GL_LINES, 0, 6);

    coloredPointProgram_.setUniform(GlUniform<GlColor>("customColor", GlColor::RED));

    mvp_ = projection_ * view_ * conversion_ * lcPoseOld_ * scale;
    coloredPointProgram_.setUniform(mvp_);
    glDrawArrays(GL_LINES, 0, 6);

    // reset.
    coloredPointProgram_.setUniform(GlUniform<bool>("useCustomColor", false));

    mvp_ = projection_ * view_ * conversion_ * model_;
  }

  if (drawing_options_["show lc candidates"] && loopClosure_) {
    ScopedBinder<GlVertexArray> vao_binder(pointVAOs_[currentBuffer_]);
    ScopedBinder<GlProgram> program_binder(pointProgram_);
    pointProgram_.setUniform(GlUniform<int>("pointColorMode", 3));

    std::vector<GlColor> colors = {GlColor::BLUE, GlColor::RED};

    for (uint32_t i = 0; i < loopClosurePoses_.size() - 1; ++i) {
      GlUniform<Eigen::Matrix4f> lcPose{"mvp", projection_ * view_ * conversion_ * loopClosurePoses_[i]};
      pointProgram_.setUniform(lcPose);

      pointProgram_.setUniform(GlUniform<GlColor>("customColor", colors[i % colors.size()]));
      glDrawArrays(GL_POINTS, 0, pointVBOs_[currentBuffer_].size());
    }

    if (loopClosureUsed_) {
      GlUniform<Eigen::Matrix4f> lcPose{"mvp", projection_ * view_ * conversion_ * loopClosurePoses_.back()};
      pointProgram_.setUniform(lcPose);

      pointProgram_.setUniform(GlUniform<GlColor>("customColor", GlColor::GREEN));
      glDrawArrays(GL_POINTS, 0, pointVBOs_[currentBuffer_].size());
    }

    pointProgram_.setUniform(pointColorMode_);  // reset.
  }

  if (drawing_options_["vertex map"] && currentFrame_->valid) {
    ScopedBinder<GlProgram> program_binder(prgDrawVertexMap3d_);
    ScopedBinder<GlVertexArray> vao_binder(vao_imageCoords_);

    prgDrawVertexMap3d_.setUniform(mvp_);
    glActiveTexture(GL_TEXTURE0);
    ScopedBinder<GlTextureRectangle> texture_binder(currentFrame_->vertex_map);
    glActiveTexture(GL_TEXTURE1);
    ScopedBinder<GlTextureRectangle> texture_binder1(residualMap_);

    glPointSize(2.0f * pointSize_);

    glDrawArrays(GL_POINTS, 0, vbo_imageCoords_.size());

    glPointSize(pointSize_);

    CheckGlError();
  }

  if (drawing_options_["normal map 3d"] && currentFrame_->valid) {
    ScopedBinder<GlProgram> program_binder(prgDrawNormalMap3d_);
    ScopedBinder<GlVertexArray> vao_binder(vao_imageCoords_);

    prgDrawNormalMap3d_.setUniform(mvp_);
    prgDrawNormalMap3d_.setUniform(mvp_inv_t_);

    glActiveTexture(GL_TEXTURE0);
    ScopedBinder<GlTextureRectangle> texture_binder1(currentFrame_->vertex_map);

    glActiveTexture(GL_TEXTURE1);
    ScopedBinder<GlTextureRectangle> texture_binder2(currentFrame_->normal_map);

    glDrawArrays(GL_POINTS, 0, vbo_imageCoords_.size());

    glActiveTexture(GL_TEXTURE0);
  }

  if (drawing_options_["last frame"] && lastFrame_->valid) {
    Eigen::Matrix4f T = projection_ * view_ * conversion_;

    if (drawing_options_["pose estimate"] && drawing_options_["groundtruth"] && timestep_ > 0 &&
        groundtruth_.size() > timestep_) {
      T = T * groundtruth_[timestep_ - 1];
    } else if (drawing_options_["pose estimate"]) {
      T = T * lastFrame_->pose;
    }

    T = projection_ * view_ * conversion_ * lastFrame_->pose;

    if (drawing_options_["icp steps"]) {
      T = projection_ * view_ * conversion_ * lastFrame_->pose;
      //      mvp_inv_t_ = projection_ * ((view_ * model_).inverse()).transpose() * conversion_;
    }

    ScopedBinder<GlProgram> program_binder(prgDrawVertexMap3d_);
    ScopedBinder<GlVertexArray> vao_binder(vao_imageCoords_);
    prgDrawVertexMap3d_.setUniform(GlUniform<int>("colorMode", 3));
    prgDrawVertexMap3d_.setUniform(GlUniform<vec4>("customColor", vec4(0.0f, 0.5f, 0.0f, 0.5f)));

    prgDrawVertexMap3d_.setUniform(GlUniform<Eigen::Matrix4f>("mvp", T));
    glActiveTexture(GL_TEXTURE0);
    ScopedBinder<GlTextureRectangle> texture_binder(lastFrame_->vertex_map);
    //    glActiveTexture(GL_TEXTURE1);
    //    ScopedBinder<GlTextureRectangle> texture_binder1(lastFrame_->depth_map);

    glPointSize(2.0f * pointSize_);

    glDrawArrays(GL_POINTS, 0, vbo_imageCoords_.size());

    glPointSize(pointSize_);

    prgDrawVertexMap3d_.setUniform(depthMapColorMode_);  // reset.
    CheckGlError();
  }

  if (drawing_options_["draw history"] && acc_ != nullptr) {
    ScopedBinder<GlVertexArray> vao_binder(vao_history_);
    ScopedBinder<GlProgram> program_binder(pointProgram_);

    uint32_t numScans = std::min(acc_->size(), historySize_);

    pointProgram_.setUniform(GlUniform<bool>("removeGround", drawing_options_["remove ground"]));

    if (drawing_options_["colorize history"]) {
      ViridisColorMap map;

      pointProgram_.setUniform(GlUniform<int>("pointColorMode", 5));

      for (int32_t i = numScans - 1; i >= 0; --i) {
        GlColor color = map(1.0f * (timestep_ - i * historyStride_) / scanCount_);

        pointProgram_.setUniform(GlUniform<GlColor>("customColor", color));
        pointProgram_.setUniform(GlUniform<Eigen::Matrix4f>("mvp", projection_ * view_ * conversion_ * acc_->pose(i)));

        glDrawArrays(GL_POINTS, acc_->offset(i), acc_->size(i));
      }

      pointProgram_.setUniform(mvp_);
      pointProgram_.setUniform(pointColorMode_);  // reset.
    } else if (drawing_options_["history height"]) {
      pointProgram_.setUniform(GlUniform<int>("pointColorMode", 7));

      for (int32_t i = numScans - 1; i >= 0; --i) {
        pointProgram_.setUniform(GlUniform<Eigen::Matrix4f>("mvp", projection_ * view_ * conversion_ * acc_->pose(i)));

        glDrawArrays(GL_POINTS, acc_->offset(i), acc_->size(i));
      }

      pointProgram_.setUniform(mvp_);
      pointProgram_.setUniform(pointColorMode_);  // reset.

    } else if (drawing_options_["old map only"]) {
      int32_t threshold = timestep_ - 100;
      int32_t startIdx = 0;

      for (uint32_t i = 0; i < acc_->size(); ++i) {
        startIdx = i;
        if (int32_t(acc_->timestamp(i)) < threshold) break;
      }

      int32_t endIdx = std::min(acc_->size() - 1, startIdx + historySize_);
      // drawing remission values in w coordinate.
      pointProgram_.setUniform(GlUniform<int>("pointColorMode", 4));

      for (int32_t i = endIdx; i >= startIdx; --i) {
        pointProgram_.setUniform(GlUniform<Eigen::Matrix4f>("mvp", projection_ * view_ * conversion_ * acc_->pose(i)));

        glDrawArrays(GL_POINTS, acc_->offset(i), acc_->size(i));
      }

      pointProgram_.setUniform(mvp_);
      pointProgram_.setUniform(pointColorMode_);  // reset.
    } else {
      // drawing remission values in w coordinate.
      pointProgram_.setUniform(GlUniform<int>("pointColorMode", 4));

      for (int32_t i = numScans - 1; i >= 0; --i) {
        pointProgram_.setUniform(GlUniform<Eigen::Matrix4f>("mvp", projection_ * view_ * conversion_ * acc_->pose(i)));

        glDrawArrays(GL_POINTS, acc_->offset(i), acc_->size(i));
      }

      pointProgram_.setUniform(mvp_);
      pointProgram_.setUniform(pointColorMode_);  // reset.
    }

    pointProgram_.setUniform(GlUniform<bool>("removeGround", false));
  }

  if (drawing_options_["current surfels"] && map_ != nullptr) {
    opts_.view_pos.x = view_pos.x();
    opts_.view_pos.y = view_pos.y();
    opts_.view_pos.z = view_pos.z();

    map_->draw(projection_ * view_ * conversion_, opts_);
    CheckGlError();
  }

  if (drawing_options_["draw robot"] && robot_ != nullptr) {
    ScopedBinder<GlProgram> program_binder(prgDrawRobotModel_);

    MatrixStack m;
    m.mvp = mvp_.value();
    m.model = model_;
    m.normal = model_.inverse().transpose();

    prgDrawRobotModel_.setUniform(GlUniform<Eigen::Vector4f>("view_pos", view_pos));

    robot_->draw(prgDrawRobotModel_, m);
    CheckGlError();
  }

  uint32_t tex_width = 720;
  uint32_t tex_height = 64;
  // overlayed images:
  uint32_t vp_x = std::max(0U, uint32_t(0.5f * (width() - tex_width))), vp_y = 0;
  glDisable(GL_DEPTH_TEST);
  linearSampler_.bind(0);

  if (drawing_options_["depth map"] && currentFrame_->valid) {
    // draw framebuffer on texture.
    {
      glViewport(vp_x, vp_y, tex_width, tex_height);
      ScopedBinder<GlProgram> program_binder(prgDrawDepthImage_);
      ScopedBinder<GlVertexArray> vao_binder(
          vao_no_point_);  // a vao must be bound; even if nothing is actually inputed to the vertex shader.
      glActiveTexture(GL_TEXTURE0);
      ScopedBinder<GlTextureRectangle> texture_binder(currentFrame_->vertex_map);
      glDrawArrays(GL_POINTS, 0, 1);

      vp_y += tex_height;
    }
  }

  if (drawing_options_["normal map"] && currentFrame_->valid) {
    // draw framebuffer on texture.
    {
      glDisable(GL_DEPTH_TEST);
      glViewport(vp_x, vp_y, tex_width, tex_height);
      ScopedBinder<GlProgram> program_binder(prgDrawNormalMap_);
      ScopedBinder<GlVertexArray> vao_binder(
          vao_no_point_);  // a vao must be bound; even if nothing is actually inputed to the vertex shader.
      glActiveTexture(GL_TEXTURE0);
      ScopedBinder<GlTextureRectangle> texture_binder(currentFrame_->normal_map);

      glDrawArrays(GL_POINTS, 0, 1);

      vp_y += tex_height;
    }
  }

  if (drawing_options_["semantic map"] && currentFrame_->valid) {
    // draw framebuffer on texture.
    {
      glDisable(GL_DEPTH_TEST);
      glViewport(vp_x, vp_y, tex_width, tex_height);
      ScopedBinder<GlProgram> program_binder(prgDrawSemanticMap_);

      ScopedBinder<GlVertexArray> vao_binder(
          vao_no_point_);  // a vao must be bound; even if nothing is actually inputed to the vertex shader.
      glActiveTexture(GL_TEXTURE0);
      ScopedBinder<GlTextureRectangle> texture_binder(currentFrame_->semantic_map);

      glDrawArrays(GL_POINTS, 0, 1);

      vp_y += tex_height;
    }
  }

  if (drawing_options_["semantic map"] && currentFrame_->valid) {
    // draw framebuffer on texture.
    {     
      glDisable(GL_DEPTH_TEST);
      glViewport(vp_x, vp_y, tex_width, tex_height);
      ScopedBinder<GlProgram> program_binder(prgDrawSemanticRangeMap_);

      ScopedBinder<GlVertexArray> vao_binder(
          vao_no_point_);  // a vao must be bound; even if nothing is actually inputed to the vertex shader.
      glActiveTexture(GL_TEXTURE0);
      ScopedBinder<GlTextureRectangle> texture_binder(model_frame_->semantic_map);

      glActiveTexture(GL_TEXTURE1);
      color_map_1dtex_.bind();

      glDrawArrays(GL_POINTS, 0, 1);

      glActiveTexture(GL_TEXTURE1);
      color_map_1dtex_.release();

      vp_y += tex_height;
    }
  }

  if (drawing_options_["depth map"] && model_frame_ != nullptr) {
    // draw framebuffer on texture.
    {
      glViewport(vp_x, vp_y, tex_width, tex_height);
      ScopedBinder<GlProgram> program_binder(prgDrawDepthImage_);
      ScopedBinder<GlVertexArray> vao_binder(
          vao_no_point_);  // a vao must be bound; even if nothing is actually inputed to the vertex shader.
      glActiveTexture(GL_TEXTURE0);
      ScopedBinder<GlTextureRectangle> texture_binder(model_frame_->vertex_map);

      glDrawArrays(GL_POINTS, 0, 1);

      vp_y += tex_height;
    }
  }

  if (drawing_options_["normal map"] && model_frame_ != nullptr) {
    // draw framebuffer on texture.
    {
      glDisable(GL_DEPTH_TEST);
      glViewport(vp_x, vp_y, tex_width, tex_height);
      ScopedBinder<GlProgram> program_binder(prgDrawNormalMap_);
      ScopedBinder<GlVertexArray> vao_binder(
          vao_no_point_);  // a vao must be bound; even if nothing is actually inputed to the vertex shader.
      glActiveTexture(GL_TEXTURE0);
      ScopedBinder<GlTextureRectangle> texture_binder(model_frame_->normal_map);

      glDrawArrays(GL_POINTS, 0, 1);

      vp_y += tex_height;
    }
  }

  if (drawing_options_["normal map"] && old_surfel_frame_ != nullptr && new_surfel_frame_ != nullptr) {
    glDisable(GL_DEPTH_TEST);
    if (old_surfel_frame_->valid) {
      glViewport(vp_x, vp_y, 0.5 * tex_width, tex_height);
      ScopedBinder<GlProgram> program_binder(prgDrawNormalMap_);
      ScopedBinder<GlVertexArray> vao_binder(vao_no_point_);
      glActiveTexture(GL_TEXTURE0);
      ScopedBinder<GlTextureRectangle> texture_binder(old_surfel_frame_->normal_map);

      glDrawArrays(GL_POINTS, 0, 1);
    }

    if (new_surfel_frame_->valid) {
      glViewport(vp_x + 0.5 * tex_width, vp_y, 0.5 * tex_width, tex_height);
      ScopedBinder<GlProgram> program_binder(prgDrawNormalMap_);
      ScopedBinder<GlVertexArray> vao_binder(vao_no_point_);
      glActiveTexture(GL_TEXTURE0);
      ScopedBinder<GlTextureRectangle> texture_binder(new_surfel_frame_->normal_map);

      glDrawArrays(GL_POINTS, 0, 1);
    }

    vp_y += tex_height;
  }

  // draw framebuffer on texture.
  if (drawing_options_["residual map"] && currentFrame_->valid && currentFrame_->residual_map.width() > 0) {
    glDisable(GL_DEPTH_TEST);
    glViewport(vp_x, vp_y, tex_width, 2 * tex_height);
    ScopedBinder<GlProgram> program_binder(prgDrawResidualMap_);
    ScopedBinder<GlVertexArray> vao_binder(
        vao_no_point_);  // a vao must be bound; even if nothing is actually inputed to the vertex shader.
    glActiveTexture(GL_TEXTURE0);
    ScopedBinder<GlTextureRectangle> texture_binder(currentFrame_->residual_map);

    glDrawArrays(GL_POINTS, 0, 1);

    vp_y += tex_height;
  }

  glViewport(0, 0, width(), height());  // reset.
  glEnable(GL_DEPTH_TEST);
  glPointSize(1.0f);
  linearSampler_.release(0);
}

glow::GlCamera::KeyboardModifier ViewportWidget::resolveKeyboardModifier(Qt::KeyboardModifiers modifiers) {
  // currently only single button presses are supported.
  GlCamera::KeyboardModifier modifier = GlCamera::KeyboardModifier::None;

  if (modifiers & Qt::ControlModifier)
    modifier = GlCamera::KeyboardModifier::CtrlDown;
  else if (modifiers & Qt::ShiftModifier)
    modifier = GlCamera::KeyboardModifier::ShiftDown;
  else if (modifiers & Qt::AltModifier)
    modifier = GlCamera::KeyboardModifier::AltDown;

  return modifier;
}

glow::GlCamera::MouseButton ViewportWidget::resolveMouseButton(Qt::MouseButtons button) {
  // currently only single button presses are supported.
  GlCamera::MouseButton btn = GlCamera::MouseButton::NoButton;

  if (button & Qt::LeftButton)
    btn = GlCamera::MouseButton::LeftButton;
  else if (button & Qt::RightButton)
    btn = GlCamera::MouseButton::RightButton;
  else if (button & Qt::MiddleButton)
    btn = GlCamera::MouseButton::MiddleButton;

  return btn;
}

void ViewportWidget::mousePressEvent(QMouseEvent* event) {
  // if camera consumes the signal, simply return. // here we could also include some remapping.
  if (camera_.mousePressed(event->windowPos().x(), event->windowPos().y(), resolveMouseButton(event->buttons()),
                           resolveKeyboardModifier(event->modifiers()))) {
    timer_.start(1. / 30.);
    return;
  }
}

void ViewportWidget::mouseReleaseEvent(QMouseEvent* event) {
  // if camera consumes the signal, simply return. // here we could also include some remapping.
  if (camera_.mouseReleased(event->windowPos().x(), event->windowPos().y(), resolveMouseButton(event->buttons()),
                            resolveKeyboardModifier(event->modifiers()))) {
    timer_.stop();
    redraw();  // get the last action.

    return;
  }
}

void ViewportWidget::mouseMoveEvent(QMouseEvent* event) {
  // if camera consumes the signal, simply return. // here we could also include some remapping.
  if (camera_.mouseMoved(event->windowPos().x(), event->windowPos().y(), resolveMouseButton(event->buttons()),
                         resolveKeyboardModifier(event->modifiers())))
    return;
}

void ViewportWidget::setPoseAdjustment(const Eigen::Matrix4f& pose) {
  poseAdjustment_ = pose;
  updateGL();
}

void ViewportWidget::setGroundtruth(const std::vector<Eigen::Matrix4f>& gt) {
  groundtruth_ = gt;
}

void ViewportWidget::setScanCount(uint32_t count) {
  scanCount_ = count;
}

void ViewportWidget::setHistoryStride(int stride) {
  historyStride_ = stride;
}

void ViewportWidget::setPosegraph(const Posegraph::ConstPtr& graph) {
  posegraph_ = graph;
}

void ViewportWidget::setColorMap(std::map<uint32_t, semantic_color> colormap) {
  color_map_ = colormap;
  u_char* color_map = new u_char[260*3];
  for (uint32_t i = 0; i < 260; ++i) {
    if (color_map_.find(i) != color_map_.end())
    {
      color_map[3 * i]     = std::get<2>(color_map_[i]);
      color_map[3 * i + 1] = std::get<1>(color_map_[i]);
      color_map[3 * i + 2] = std::get<0>(color_map_[i]);
    }
    else
    {
      color_map[3 * i]     = 0;
      color_map[3 * i + 1] = 0;
      color_map[3 * i + 2] = 0;
    }
  }
  color_map_1dtex_.assign(PixelFormat::RGB, PixelType::UNSIGNED_BYTE, color_map);
}
