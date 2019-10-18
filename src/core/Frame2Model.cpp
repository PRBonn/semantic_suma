#include "core/Frame2Model.h"

#include <rv/Math.h>
#include <rv/Stopwatch.h>
#include <fstream>
#include <iostream>
#include "core/lie_algebra.h"

#include <exception>

using namespace rv;
using namespace glow;

Frame2Model::Frame2Model(const rv::ParameterList& params)
    : JtJJtf_blend_(2, 8, TextureFormat::RGB_FLOAT),
      fbo_blend_(2, 8)  // blending for matrix comp.
{
  params_ = params;

  uint32_t width = params["data_width"], height = params["data_height"];
  if (params.hasParam("entriesPerKernel")) {
    entriesPerKernel_ = params["entriesPerKernel"];
  }
  std::vector<vec2> img_coords;
  img_coords.reserve(width * height);
  for (uint32_t i = 0; i < width; i += entriesPerKernel_) {
    for (uint32_t j = 0; j < height; ++j) {
      img_coords.push_back(vec2(i + 0.5f, j + 0.5f));
    }
  }

  vbo_img_coords_.assign(img_coords);
  vao_img_coords_.bind();
  vao_img_coords_.setVertexAttribute(0, vbo_img_coords_, 2, AttributeType::FLOAT, false, 2 * sizeof(float), nullptr);
  vao_img_coords_.enableVertexAttribute(0);
  vao_img_coords_.release();

  // Computation of the Jacobian using blending (TM):

  JtJJtf_blend_.setMinifyingOperation(TexMinOp::NEAREST);
  JtJJtf_blend_.setMagnifyingOperation(TexMagOp::NEAREST);
  JtJJtf_blend_.setWrapOperation(TexWrapOp::CLAMP_TO_BORDER, TexWrapOp::CLAMP_TO_BORDER);

  fbo_blend_.attach(FramebufferAttachment::COLOR0, JtJJtf_blend_);
  GlRenderbuffer rbo2(fbo_blend_.width(), fbo_blend_.height(), RenderbufferFormat::DEPTH_STENCIL);
  fbo_blend_.attach(FramebufferAttachment::DEPTH_STENCIL, rbo2);

  program_blend_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/Frame2Model_jacobians.vert"));
  program_blend_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shader/Frame2Model_jacobians.geom"));
  program_blend_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/Frame2Model_jacobians.frag"));
  program_blend_.link();

  program_blend_.setUniform(GlUniform<int32_t>("vertex_model", 0));
  program_blend_.setUniform(GlUniform<int32_t>("normal_model", 1));
  program_blend_.setUniform(GlUniform<int32_t>("vertex_data", 2));
  program_blend_.setUniform(GlUniform<int32_t>("normal_data", 3));
  program_blend_.setUniform(GlUniform<int32_t>("semantic_model", 4));
  program_blend_.setUniform(GlUniform<int32_t>("semantic_data", 5));
  program_blend_.setUniform(GlUniform<Eigen::Matrix4f>("pose", Eigen::Matrix4f::Identity()));
  program_blend_.setUniform(GlUniform<int32_t>("entries_per_kernel", entriesPerKernel_));

  updateParameters();
}

void Frame2Model::updateParameters() {
  float angle_thresh = std::cos(Math::deg2rad(params_["icp-max-angle"]));
  float distance_thresh = params_["icp-max-distance"];

  int32_t weight_function = 0;
  float factor = 1.0f;
  if (params_.hasParam("weighting")) {
    std::string weighting_name = params_["weighting"];
    if (weighting_name == "huber")
      weight_function = 1;
    else if (weighting_name == "turkey")
      weight_function = 2;
    else if (weighting_name == "stability")
      weight_function = 3;
    factor = params_["factor"];
  }

  float fov_up = std::abs(float(params_["data_fov_up"]));
  float fov_down = std::abs(float(params_["data_fov_down"]));

  bool distance_outliers = false;
  if (params_.hasParam("distance_outliers")) distance_outliers = params_["distance_outliers"];

  program_blend_.setUniform(GlUniform<bool>("distance_outliers", (weight_function == 0) || distance_outliers));
  program_blend_.setUniform(GlUniform<int32_t>("weight_function", weight_function));
  program_blend_.setUniform(GlUniform<float>("factor", factor));

  program_blend_.setUniform(GlUniform<float>("factor", factor));
  program_blend_.setUniform(GlUniform<float>("distance_thresh", distance_thresh));
  program_blend_.setUniform(GlUniform<float>("angle_thresh", angle_thresh));
  program_blend_.setUniform(GlUniform<float>("fov_up", fov_up));
  program_blend_.setUniform(GlUniform<float>("fov_down", fov_down));
  program_blend_.setUniform(GlUniform<float>("fov", fov_up + fov_down));
  program_blend_.setUniform(GlUniform<float>("min_depth", float(params_["min_depth"])));
  program_blend_.setUniform(GlUniform<float>("max_depth", float(params_["max_depth"])));
  program_blend_.setUniform(GlUniform<float>("cutoff_threshold", float(params_["cutoff_threshold"])));

  sampler_.setWrapOperation(TexWrapOp::CLAMP_TO_BORDER, TexWrapOp::CLAMP_TO_BORDER);
  if (params_.hasParam("bilinear_sampling") && (bool)params_["bilinear_sampling"]) {
    sampler_.setMinifyingOperation(TexMinOp::LINEAR);
    sampler_.setMagnifyingOperation(TexMagOp::LINEAR);
  } else {
    sampler_.setMinifyingOperation(TexMinOp::NEAREST);
    sampler_.setMagnifyingOperation(TexMagOp::NEAREST);
  }
}

void Frame2Model::setParameter(const rv::Parameter& param) {
  params_.insert(param);
  updateParameters();
}

void Frame2Model::setData(const std::shared_ptr<Frame>& current, const std::shared_ptr<Frame>& last) {
  current_ = current;
  num_residuals_ = current->points.size();
  last_ = last;

  iteration_ = 0;
}

void Frame2Model::setLevel(uint32_t lvl) {}

uint32_t Frame2Model::getMaxLevel() const {
  return 0;
}

double Frame2Model::residual(const Eigen::VectorXd& delta) {
  throw std::runtime_error("not implemented.");
  return -1;
}

double Frame2Model::jacobianProducts(Eigen::MatrixXd& JtJ, Eigen::MatrixXd& Jtf) {
  assert(JtJ.rows() == 6 && JtJ.cols() == 6);
  assert(Jtf.rows() == 6 && Jtf.cols() == 1);

  //  Stopwatch::tic();

  double F = 0.0;

  GLint ov[4];
  GLfloat cc[4];

  glGetIntegerv(GL_VIEWPORT, ov);
  glGetFloatv(GL_COLOR_CLEAR_VALUE, cc);

  glPointSize(1.0f);
  glClearColor(0, 0, 0, 0);
  glDisable(GL_DEPTH_TEST);

  glActiveTexture(GL_TEXTURE0);
  last_->vertex_map.bind();

  glActiveTexture(GL_TEXTURE1);
  last_->normal_map.bind();

  glActiveTexture(GL_TEXTURE2);
  current_->vertex_map.bind();

  glActiveTexture(GL_TEXTURE3);
  current_->normal_map.bind();

  glActiveTexture(GL_TEXTURE4);
  last_->semantic_map.bind();

  glActiveTexture(GL_TEXTURE5);
  current_->semantic_map.bind();

  sampler_.bind(0);
  sampler_.bind(1);
  sampler_.bind(2);
  sampler_.bind(3);
  sampler_.bind(4);
  sampler_.bind(5);

  vao_img_coords_.bind();

  //  Stopwatch::tic();

  // second stage: compute jacobian.
  fbo_blend_.bind();

  glViewport(0, 0, fbo_blend_.width(), fbo_blend_.height());  // matrix width/height.
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);         // set jacobian to zero.

  glEnable(GL_BLEND);
  glBlendFunc(GL_ONE, GL_ONE);  // computing the sum of values

  program_blend_.bind();

  program_blend_.setUniform(GlUniform<Eigen::Matrix4f>("pose", pose_.cast<float>()));
  program_blend_.setUniform(GlUniform<int32_t>("iteration", iteration_));

  glDrawArrays(GL_POINTS, 0, vbo_img_coords_.size());  // size depending on last's size.
  program_blend_.release();

  vao_img_coords_.release();
  fbo_blend_.release();

  glDisable(GL_BLEND);

  glFinish();

  //  double blendTime = Stopwatch::toc();

  //  Stopwatch::tic();

  std::vector<float> blending(6 * 8);
  JtJJtf_blend_.download(PixelFormat::RGB, &blending[0]);

  for (uint32_t i = 0; i < 6 * 6; ++i) {
    JtJ.data()[i] = blending[i];
  }

  for (uint32_t i = 0; i < 6; ++i) {
    Jtf.data()[i] = blending[36 + i];
  }

  uint32_t valid = blending[42];
  F = blending[43];
  outlier_ = blending[44];
  inlier_residual_ = blending[45];
  inlier_ = valid - outlier_;
  invalid_ = blending[46];

  glEnable(GL_DEPTH_TEST);
  sampler_.release(0);
  sampler_.release(1);
  sampler_.release(2);
  sampler_.release(3);
  sampler_.release(4);
  sampler_.release(5);

  glActiveTexture(GL_TEXTURE0);
  last_->vertex_map.release();

  glActiveTexture(GL_TEXTURE1);
  last_->normal_map.release();

  glActiveTexture(GL_TEXTURE2);
  current_->vertex_map.release();

  glActiveTexture(GL_TEXTURE3);
  current_->normal_map.release();

  glActiveTexture(GL_TEXTURE4);
  last_->semantic_map.release();

  glActiveTexture(GL_TEXTURE5);
  current_->semantic_map.release();

  glViewport(ov[0], ov[1], ov[2], ov[3]);
  glClearColor(cc[0], cc[1], cc[2], cc[3]);

  glFinish();

  return F;
}

uint32_t Frame2Model::num_parameters() const {
  return 6;
}
