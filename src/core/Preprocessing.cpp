/*
 * Preprocessing.cpp
 *
 *  Created on: Jul 29, 2016
 *      Author: behley
 *
 *      Author: Xieyuanli Chen
 */

#include "core/Preprocessing.h"

#include <glow/GlState.h>
#include <glow/glutil.h>

using namespace rv;
using namespace glow;

// -- OpenGL-based preprocessing:

inline void PASS(const char* sFile, const int nLine) {
  std::cout << "Passed: " << sFile << " on Line: " << nLine << std::endl;
}

Preprocessing::Preprocessing(const rv::ParameterList& params)
    : width_(params["data_width"]),
      height_(params["data_height"]),
      framebuffer_(width_, height_, FramebufferTarget::BOTH),
      semanticbuffer_(width_, height_, FramebufferTarget::BOTH),
      temp_vertices_(width_, height_, TextureFormat::RGBA_FLOAT){

  depth_program_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/gen_vertexmap.vert"));
  depth_program_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/gen_vertexmap.frag"));
  depth_program_.link();

  avg_program_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/empty.vert"));
  avg_program_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shader/quad.geom"));
  avg_program_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/avg_vertexmap.frag"));
  avg_program_.link();

  bilateral_program_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/empty.vert"));
  bilateral_program_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shader/quad.geom"));
  bilateral_program_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/bilateral_filter.frag"));
  bilateral_program_.link();

  normal_program_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/empty.vert"));
  normal_program_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shader/quad.geom"));
  normal_program_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/gen_normalmap.frag"));
  normal_program_.link();

  floodfill_program_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/empty.vert"));
  floodfill_program_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shader/quad.geom"));
  floodfill_program_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/floodfill.frag"));
  floodfill_program_.link();

  // 1. setup framebuffer.
  GlRenderbuffer rbo(width_, height_, RenderbufferFormat::DEPTH_STENCIL);
  framebuffer_.attach(FramebufferAttachment::DEPTH_STENCIL, rbo);

  GlRenderbuffer rbo2(width_, height_, RenderbufferFormat::DEPTH_STENCIL);
  semanticbuffer_.attach(FramebufferAttachment::DEPTH_STENCIL, rbo2);

  GLenum buffs2[] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};

  semanticbuffer_.bind();
  glDrawBuffers(2, buffs2);
  semanticbuffer_.release();

  sampler_.setMinifyingOperation(TexMinOp::NEAREST);
  sampler_.setMagnifyingOperation(TexMagOp::NEAREST);
  sampler_.setWrapOperation(TexWrapOp::CLAMP_TO_BORDER, TexWrapOp::CLAMP_TO_BORDER);

  // finally set parameters of shaders, etc.
  setParameters(params);
}

void Preprocessing::setParameters(const rv::ParameterList& params) {
  float fov_up = params["data_fov_up"];
  float fov_down = params["data_fov_down"];
  float sigma_space = 0.0f, sigma_range = 0.0f;

  useFilteredVertexmap_ = false;
  filterVertexmap_ = false;
  avgVertexmap_ = false;
  if (params.hasParam("filter_vertexmap")) filterVertexmap_ = params["filter_vertexmap"];
  if (params.hasParam("avg_vertexmap")) avgVertexmap_ = params["avg_vertexmap"];
  if (filterVertexmap_) {
    sigma_space = params["bilateral_sigma_space"];
    sigma_range = params["bilateral_sigma_range"];
    useFilteredVertexmap_ = params["use_filtered_vertexmap"];
  }

  fov_up = std::abs(fov_up);
  fov_down = std::abs(fov_down);

  depth_program_.setUniform(GlUniform<float>("width", width_));
  depth_program_.setUniform(GlUniform<float>("height", height_));
  depth_program_.setUniform(GlUniform<float>("fov_up", fov_up));
  depth_program_.setUniform(GlUniform<float>("fov_down", fov_down));
  depth_program_.setUniform(GlUniform<float>("min_depth", static_cast<float>(params["min_depth"])));
  depth_program_.setUniform(GlUniform<float>("max_depth", static_cast<float>(params["max_depth"])));

  // for semantic map
  depth_program_.setUniform(GlUniform<Eigen::Matrix4f>("Tr", Tr));
  depth_program_.setUniform(GlUniform<Eigen::Matrix4f>("Tr_inv", Tr_inv));
  depth_program_.setUniform(GlUniform<Eigen::Matrix4f>("P2", P2));

  bilateral_program_.setUniform(GlUniform<float>("width", width_));
  bilateral_program_.setUniform(GlUniform<float>("height", height_));
  bilateral_program_.setUniform(GlUniform<float>("sigma_space", sigma_space));
  bilateral_program_.setUniform(GlUniform<float>("sigma_range", sigma_range));

  // for semantic map
  normal_program_.setUniform(GlUniform<int32_t>("vertex_map", 0));
  normal_program_.setUniform(GlUniform<int32_t>("semantic_map", 1));
  floodfill_program_.setUniform(GlUniform<int32_t>("vertex_map", 0));
  floodfill_program_.setUniform(GlUniform<int32_t>("semantic_map", 1));
}

/** \brief pre-process the point cloud (validity map, vertex map, normal map, ... etc.) and store results in frame. **/
void Preprocessing::process(glow::GlBuffer<rv::Point3f>& points, Frame& frame, glow::GlBuffer<float>& labels, glow::GlBuffer<float>& probs, uint32_t timestamp_) {
  CheckGlError();

  frame.points.assign(points);
  frame.labels.assign(labels); // for semantic map
  frame.probs.assign(probs);

  GLboolean depthTest;
  GLint ov[4], depthFunc, old_fbo;
  GLfloat cc[4];
  GLfloat psize;

  glGetIntegerv(GL_VIEWPORT, ov);
  glGetBooleanv(GL_DEPTH_TEST, &depthTest);
  glGetIntegerv(GL_DEPTH_FUNC, &depthFunc);
  glGetFloatv(GL_COLOR_CLEAR_VALUE, cc);
  glGetFloatv(GL_POINT_SIZE, &psize);
  glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &old_fbo);

  glPointSize(1.0f);
  vao_points_.setVertexAttribute(0, points, 4, AttributeType::FLOAT, false, 4 * sizeof(float), nullptr);
  vao_points_.enableVertexAttribute(0);
  vao_points_.setVertexAttribute(1, labels, 1, AttributeType::FLOAT, false, 1 * sizeof(float), (void*)(4* sizeof(float)));
  vao_points_.enableVertexAttribute(1);
  vao_points_.setVertexAttribute(2, probs, 1, AttributeType::FLOAT, false, 1 * sizeof(float), (void*)(5* sizeof(float)));
  vao_points_.enableVertexAttribute(2);

  // 1. pass: generate raw vertex map:

  if (avgVertexmap_)
    semanticbuffer_.attach(FramebufferAttachment::COLOR0, temp_vertices_);
  else
    semanticbuffer_.attach(FramebufferAttachment::COLOR0, frame.vertex_map);
  assert(semanticbuffer_.valid());

  semanticbuffer_.attach(FramebufferAttachment::COLOR1, frame.semantic_map);
  semanticbuffer_.bind();

  glDepthFunc(GL_LESS);

  if (avgVertexmap_) {
    // average vertex map.
    glDisable(GL_DEPTH_TEST);

    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE);  // computing the sum of values
  } else {
    // z-buffered vertex map
    glEnable(GL_DEPTH_TEST);
  }

  glClearColor(0, 0, 0, 0);
  glViewport(0, 0, width_, height_);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  // reset depth/vertexmap

  if (timestamp_ < 10)
    depth_program_.setUniform(GlUniform<bool>("isfirst", true));
  else
    depth_program_.setUniform(GlUniform<bool>("isfirst", false));

  vao_points_.bind();
  depth_program_.bind();

  glDrawArrays(GL_POINTS, 0, points.size());

  depth_program_.release();
  vao_points_.release();
  semanticbuffer_.release();
  glFinish();

  if (avgVertexmap_) {
    glDisable(GL_BLEND);

    framebuffer_.attach(FramebufferAttachment::COLOR0, frame.vertex_map);
    framebuffer_.bind();
    vao_no_points_.bind();
    avg_program_.bind();

    glActiveTexture(GL_TEXTURE0);
    temp_vertices_.bind();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDrawArrays(GL_POINTS, 0, 1);

    temp_vertices_.release();

    vao_no_points_.release();
    avg_program_.release();
    framebuffer_.release();

    glEnable(GL_DEPTH_TEST);
  }

  // step 1.5: do bilateral filtering of vertex map.
  if (filterVertexmap_) {
    framebuffer_.attach(FramebufferAttachment::COLOR0, temp_vertices_);

    glActiveTexture(GL_TEXTURE0);
    frame.vertex_map.bind();

    framebuffer_.bind();
    vao_no_points_.bind();
    bilateral_program_.bind();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDrawArrays(GL_POINTS, 0, 1);

    vao_no_points_.release();
    bilateral_program_.release();
    framebuffer_.release();

    glActiveTexture(GL_TEXTURE0);
    frame.vertex_map.release();

    if (useFilteredVertexmap_) frame.vertex_map.copy(temp_vertices_);
  }

  // 2. pass: generate normal map:
  glDisable(GL_DEPTH_TEST);

  glow::GlTextureRectangle erode_semantic_map(width_, height_, TextureFormat::RGBA_FLOAT);

  semanticbuffer_.attach(FramebufferAttachment::COLOR0, frame.normal_map);
  semanticbuffer_.attach(FramebufferAttachment::COLOR1, erode_semantic_map);
  semanticbuffer_.bind();

  glActiveTexture(GL_TEXTURE0);
  if (filterVertexmap_)
    temp_vertices_.bind();
  else
    frame.vertex_map.bind();

  glActiveTexture(GL_TEXTURE1);
  frame.semantic_map.bind();

  sampler_.bind(0);
  sampler_.bind(1);

  vao_no_points_.bind();
  normal_program_.bind();

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  // reset depth/normalmap
  glDrawArrays(GL_POINTS, 0, 1);

  normal_program_.release();
  vao_no_points_.release();
  semanticbuffer_.release();

  glActiveTexture(GL_TEXTURE0);
  if (filterVertexmap_)
    temp_vertices_.release();
  else
    frame.vertex_map.release();

  glActiveTexture(GL_TEXTURE1);
  frame.semantic_map.release();

  sampler_.release(0);
  sampler_.release(1);

  // 3. refine: floodfill semantic map:
  glDisable(GL_DEPTH_TEST);
  glow::GlTextureRectangle refine_semantic_map(width_, height_, TextureFormat::RGBA_FLOAT);

  framebuffer_.attach(FramebufferAttachment::COLOR0, refine_semantic_map);
  assert(framebuffer_.valid());

  framebuffer_.bind();

  glActiveTexture(GL_TEXTURE0);
  if (filterVertexmap_)
    temp_vertices_.bind();
  else
    frame.vertex_map.bind();

  glActiveTexture(GL_TEXTURE1);
  erode_semantic_map.bind();

  sampler_.bind(0);
  sampler_.bind(1);

  vao_no_points_.bind();
  floodfill_program_.bind();

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  // reset depth/normalmap
  glDrawArrays(GL_POINTS, 0, 1);

  floodfill_program_.release();
  vao_no_points_.release();
  framebuffer_.release();

  glActiveTexture(GL_TEXTURE0);
  if (filterVertexmap_)
    temp_vertices_.release();
  else
    frame.vertex_map.release();

  glActiveTexture(GL_TEXTURE1);
  erode_semantic_map.release();

  sampler_.release(0);
  sampler_.release(1);

  glFinish();

  frame.valid = true;
  frame.semantic_map.copy(refine_semantic_map);

  // restore settings.
  glViewport(ov[0], ov[1], ov[2], ov[3]);
  if (depthTest) glEnable(GL_DEPTH_TEST);

  glDepthFunc(depthFunc);
  glClearColor(cc[0], cc[1], cc[2], cc[3]);
  glPointSize(psize);
  glBindFramebuffer(GL_FRAMEBUFFER, old_fbo);

  CheckGlError();
}

void Preprocessing::setCalibration(const KITTICalibration& calib) {
  calib_ = calib;
  if (calib_.exists("Tr") && calib_.exists("P0")) {
    Tr = calib_["Tr"];
    P2 = calib_["P2"];
    Tr_inv = Tr.inverse();
  }
}
