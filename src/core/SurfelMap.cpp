#include "core/SurfelMap.h"
#include <glow/GlState.h>
#include <glow/ScopedBinder.h>
#include <rv/Math.h>

using namespace rv;
using namespace glow;

SurfelMap::SurfelMap(const ParameterList& params)
    : dataWidth_(params["data_width"]),
      dataHeight_(params["data_height"]),
      modelWidth_(params["model_width"]),
      modelHeight_(params["model_height"]),
      indexMapFramebuffer_(dataWidth_, dataHeight_),
      indexMap_(dataWidth_, dataHeight_, TextureFormat::R_FLOAT),
      indexVertexMap_(dataWidth_, dataHeight_, TextureFormat::RGBA_FLOAT),
      indexNormalMap_(dataWidth_, dataHeight_, TextureFormat::RGBA_FLOAT),
      updateFramebuffer_(dataWidth_, dataHeight_),
      measurementIntegrated_(dataWidth_, dataHeight_, TextureFormat::RGBA_FLOAT),
      renderFramebuffer_(modelWidth_, modelHeight_),
      radiusConfidenceFramebuffer_(dataWidth_, dataHeight_),
      centerizedVertexMap_(dataWidth_, dataHeight_, TextureFormat::RGBA_FLOAT),
      filteredNormalMap_(dataWidth_, dataHeight_, TextureFormat::RGBA_FLOAT),
      radiusConfidenceMap_(dataWidth_, dataHeight_, TextureFormat::RGBA_FLOAT),
      poseTexture_(poseBuffer_, TextureFormat::RGBA_FLOAT),
      color_map_1dtex_(260, TextureFormat::RGB_FLOAT)
{
  glow::_CheckGlError(__FILE__, __LINE__);
  draw_surfels_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/draw_surfels.vert"));
  draw_surfels_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shader/draw_surfels.geom"));
  draw_surfels_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/draw_surfels.frag"));
  draw_surfels_.link();

  draw_surfelPoints_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/draw_surfelPoints.vert"));
  draw_surfelPoints_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/passthrough.frag"));
  draw_surfelPoints_.link();

  std::vector<std::string> surfel_varyings{
      "sfl_position_radius", "sfl_normal_confidence", "sfl_timestamp", "sfl_color_weight_count", "sfl_semantic_map",
  };

  surfels_.reserve(maxNumSurfels_);
  copy_feedback_.attach(surfel_varyings, surfels_);

  // now we can set the vertex attributes. (the "shallow copy" of surfels now contains the correct id.
  vao_surfels_.setVertexAttribute(0, surfels_, 4, AttributeType::FLOAT, false, sizeof(Surfel),
                                  reinterpret_cast<GLvoid*>(0));
  vao_surfels_.setVertexAttribute(1, surfels_, 4, AttributeType::FLOAT, false, sizeof(Surfel),
                                  reinterpret_cast<GLvoid*>(4 * sizeof(GLfloat)));
  vao_surfels_.setVertexAttribute(2, surfels_, 1, AttributeType::INT, false, sizeof(Surfel),
                                  reinterpret_cast<GLvoid*>(8 * sizeof(GLfloat)));
  vao_surfels_.setVertexAttribute(3, surfels_, 3, AttributeType::FLOAT, false, sizeof(Surfel),
                                  reinterpret_cast<GLvoid*>(offsetof(Surfel, color)));
  vao_surfels_.setVertexAttribute(4, surfels_, 4, AttributeType::FLOAT, false, sizeof(Surfel),
                                  reinterpret_cast<GLvoid*>(offsetof(Surfel, r)));

  data_surfels_.reserve(2 * dataWidth_ * dataHeight_);
  initialize_feedback_.attach(surfel_varyings, data_surfels_);

  vao_data_surfels_.setVertexAttribute(0, data_surfels_, 4, AttributeType::FLOAT, false, sizeof(Surfel),
                                       reinterpret_cast<GLvoid*>(0));
  vao_data_surfels_.setVertexAttribute(1, data_surfels_, 4, AttributeType::FLOAT, false, sizeof(Surfel),
                                       reinterpret_cast<GLvoid*>(4 * sizeof(GLfloat)));
  vao_data_surfels_.setVertexAttribute(2, data_surfels_, 1, AttributeType::INT, false, sizeof(Surfel),
                                       reinterpret_cast<GLvoid*>(8 * sizeof(GLfloat)));
  vao_data_surfels_.setVertexAttribute(3, data_surfels_, 3, AttributeType::FLOAT, false, sizeof(Surfel),
                                       reinterpret_cast<GLvoid*>(offsetof(Surfel, color)));
  vao_data_surfels_.setVertexAttribute(4, data_surfels_, 4, AttributeType::FLOAT, false, sizeof(Surfel),
                                  reinterpret_cast<GLvoid*>(offsetof(Surfel, r)));

  updated_surfels_.reserve(maxNumSurfels_);
  update_feedback_.attach(surfel_varyings, updated_surfels_);

  vao_updated_surfels_.setVertexAttribute(0, updated_surfels_, 4, AttributeType::FLOAT, false, sizeof(Surfel),
                                          reinterpret_cast<GLvoid*>(0));
  vao_updated_surfels_.setVertexAttribute(1, updated_surfels_, 4, AttributeType::FLOAT, false, sizeof(Surfel),
                                          reinterpret_cast<GLvoid*>(4 * sizeof(GLfloat)));
  vao_updated_surfels_.setVertexAttribute(2, updated_surfels_, 1, AttributeType::INT, false, sizeof(Surfel),
                                          reinterpret_cast<GLvoid*>(8 * sizeof(GLfloat)));
  vao_updated_surfels_.setVertexAttribute(3, updated_surfels_, 3, AttributeType::FLOAT, false, sizeof(Surfel),
                                          reinterpret_cast<GLvoid*>(offsetof(Surfel, color)));
  vao_updated_surfels_.setVertexAttribute(4, updated_surfels_, 4, AttributeType::FLOAT, false, sizeof(Surfel),
                                  reinterpret_cast<GLvoid*>(offsetof(Surfel, r)));

  std::vector<vec2> img_coords;
  img_coords.reserve(dataWidth_ * dataHeight_);

  for (uint32_t x = 0; x < dataWidth_; ++x) {
    for (uint32_t y = 0; y < dataHeight_; ++y) {
      img_coords.push_back(vec2(x + 0.5f, y + 0.5f));
    }
  }

  vbo_img_coords_.assign(img_coords);

  vao_img_coords_.bind();
  vao_img_coords_.setVertexAttribute(0, vbo_img_coords_, 2, AttributeType::FLOAT, false, 2 * sizeof(float), 0);
  vao_img_coords_.release();

  indexMapFramebuffer_.attach(FramebufferAttachment::COLOR0, indexMap_);
  indexMapFramebuffer_.attach(FramebufferAttachment::COLOR1, indexVertexMap_);
  indexMapFramebuffer_.attach(FramebufferAttachment::COLOR2, indexNormalMap_);
  GlRenderbuffer rbo_indexmap(indexMapFramebuffer_.width(), indexMapFramebuffer_.height(),
                              RenderbufferFormat::DEPTH_STENCIL);
  indexMapFramebuffer_.attach(FramebufferAttachment::DEPTH_STENCIL, rbo_indexmap);

  GLenum buffs3[] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2};
  indexMapFramebuffer_.bind();
  glDrawBuffers(3, buffs3);
  indexMapFramebuffer_.release();

  updateFramebuffer_.attach(FramebufferAttachment::COLOR0, measurementIntegrated_);
  GlRenderbuffer rbo_update(updateFramebuffer_.width(), updateFramebuffer_.height(), RenderbufferFormat::DEPTH_STENCIL);
  updateFramebuffer_.attach(FramebufferAttachment::DEPTH_STENCIL, rbo_update);

  radiusConfidenceFramebuffer_.attach(FramebufferAttachment::COLOR0, centerizedVertexMap_);
  radiusConfidenceFramebuffer_.attach(FramebufferAttachment::COLOR1, radiusConfidenceMap_);
  radiusConfidenceFramebuffer_.attach(FramebufferAttachment::COLOR2, filteredNormalMap_);

  GlRenderbuffer rbo_radius(radiusConfidenceFramebuffer_.width(), radiusConfidenceFramebuffer_.height(),
                            RenderbufferFormat::DEPTH_STENCIL);
  radiusConfidenceFramebuffer_.attach(FramebufferAttachment::DEPTH_STENCIL, rbo_radius);

  radiusConfidenceFramebuffer_.bind();
  glDrawBuffers(3, buffs3);
  radiusConfidenceFramebuffer_.release();

  initialize_program_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/gen_surfels.vert"));
  initialize_program_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shader/gen_surfels.geom"));
  initialize_program_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/empty.frag"));
  initialize_program_.attach(initialize_feedback_);
  initialize_program_.link();

  initialize_program_.setUniform(GlUniform<int32_t>("poseBuffer", 5));

  radConf_program_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/init_radiusConf.vert"));
  radConf_program_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/init_radiusConf.frag"));
  radConf_program_.link();

  indexMap_program_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/gen_indexmap.vert"));
  indexMap_program_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/gen_indexmap.frag"));
  indexMap_program_.link();

  indexMap_program_.setUniform(GlUniform<float>("fov_up", std::abs((float)params["data_fov_up"])));
  indexMap_program_.setUniform(GlUniform<float>("fov_down", std::abs((float)params["data_fov_down"])));
  indexMap_program_.setUniform(GlUniform<float>("min_depth", params["min_depth"]));
  indexMap_program_.setUniform(GlUniform<float>("max_depth", params["max_depth"]));
  indexMap_program_.setUniform(GlUniform<float>("width", indexMap_.width()));
  indexMap_program_.setUniform(GlUniform<float>("height", indexMap_.height()));
  indexMap_program_.setUniform(GlUniform<int32_t>("poseBuffer", 5));

  update_program_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/update_surfels.vert")); // for semantic map
  update_program_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shader/update_surfels.geom"));
  update_program_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/update_surfels.frag"));
  update_program_.attach(update_feedback_);
  update_program_.link();

  update_program_.setUniform(GlUniform<int32_t>("poseBuffer", 5));

  copy_program_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/copy_surfels.vert"));
  copy_program_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shader/copy_surfels.geom"));
  copy_program_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/empty.frag"));
  copy_program_.attach(copy_feedback_);
  copy_program_.link();

  copy_program_.setUniform(GlUniform<int32_t>("poseBuffer", 5));

  sampler_.setMinifyingOperation(TexMinOp::NEAREST);
  sampler_.setMagnifyingOperation(TexMagOp::LINEAR);
  sampler_.setWrapOperation(TexWrapOp::CLAMP_TO_BORDER, TexWrapOp::CLAMP_TO_BORDER);

  GlRenderbuffer rbo_render(renderFramebuffer_.width(), renderFramebuffer_.height(), RenderbufferFormat::DEPTH_STENCIL);
  renderFramebuffer_.attach(FramebufferAttachment::DEPTH_STENCIL, rbo_render);

  renderFramebuffer_.bind();
  glDrawBuffers(3, buffs3);
  renderFramebuffer_.release();
  CheckGlError();

  render_program_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/render_surfels.vert"));
  render_program_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shader/render_surfels.geom"));
  render_program_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/render_surfels.frag"));
  render_program_.link();

  render_program_.setUniform(GlUniform<int32_t>("poseBuffer", 5));

  draw_surfels_.setUniform(GlUniform<int32_t>("color_map", 7)); // for semantic map
  draw_surfels_.setUniform(GlUniform<int32_t>("poseBuffer", 5));
  draw_surfels_.setUniform(GlUniform<vec4>("lights[0].position", vec4(0, -1, -1, 0)));
  draw_surfels_.setUniform(GlUniform<vec3>("lights[0].ambient", vec3(.9, .9, .9)));
  draw_surfels_.setUniform(GlUniform<vec3>("lights[0].diffuse", vec3(.9, .9, .9)));
  draw_surfels_.setUniform(GlUniform<vec3>("lights[0].specular", vec3(.9, .9, .9)));

  // nießner values:
  draw_surfels_.setUniform(GlUniform<vec4>("lights[0].position", vec4(0, -1, -1, 0)));
  draw_surfels_.setUniform(GlUniform<vec3>("lights[0].ambient", vec3(.4, .4, .4)));
  draw_surfels_.setUniform(GlUniform<vec3>("lights[0].diffuse", vec3(0.6f, 0.52944f, 0.4566f)));
  draw_surfels_.setUniform(GlUniform<vec3>("lights[0].specular", vec3(.3, .3, .3)));

  // more evenly distributed sun light...
  std::vector<vec4> dirs = {vec4(1, -1, 1, 0), vec4(-1, -1, 1, 0), vec4(1, -1, -1, 0), vec4(-1, -1, -1, 0)};
  vec3 indirect_intensity = vec3(.1, .1, .1);

  for (uint32_t i = 0; i < 4; ++i) {
    std::stringstream light_name;
    light_name << "lights[" << (i + 1) << "]";

    draw_surfels_.setUniform(GlUniform<vec4>(light_name.str() + ".position", dirs[i]));
    draw_surfels_.setUniform(GlUniform<vec3>(light_name.str() + ".ambient", indirect_intensity));
    draw_surfels_.setUniform(GlUniform<vec3>(light_name.str() + ".diffuse", indirect_intensity));
    draw_surfels_.setUniform(GlUniform<vec3>(light_name.str() + ".specular", indirect_intensity));
  }

  draw_surfels_.setUniform(GlUniform<int>("num_lights", 1));

  draw_surfels_.setUniform(GlUniform<vec3>("material.ambient", vec3(0.2, 0.2, 0.2)));
  //  draw_program_.setUniform(GlUniform<vec3>("material.diffuse", vec3(0.85, 0.71, 0.50)));
  draw_surfels_.setUniform(GlUniform<vec3>("material.diffuse", vec3(65. / 255., 20. / 255., 100.0 / 255.)));
  draw_surfels_.setUniform(GlUniform<vec3>("material.emission", vec3(0.0, 0.0, 0.0)));
  draw_surfels_.setUniform(GlUniform<vec3>("material.specular", vec3(0.0, 0.0, 0.0)));
  draw_surfels_.setUniform(GlUniform<float>("material.shininess", 0.25));
  draw_surfels_.setUniform(GlUniform<float>("material.alpha", 1.0));

  // nießner values:
  draw_surfels_.setUniform(GlUniform<vec3>("material.ambient", vec3(0.75f, 0.65f, 0.5f)));
  draw_surfels_.setUniform(GlUniform<vec3>("material.diffuse", vec3(1.0f, 0.9f, 0.7f)));
  draw_surfels_.setUniform(GlUniform<vec3>("material.emission", vec3(0.0, 0.0, 0.0)));
  draw_surfels_.setUniform(GlUniform<vec3>("material.specular", vec3(1.0, 1.0, 1.0)));
  draw_surfels_.setUniform(GlUniform<float>("material.shininess", 16));

//  draw_surfels_.setUniform(GlUniform<int>("surfelDrawMethod", 1)); // for semantic map

  draw_surfelPoints_.setUniform(GlUniform<int32_t>("poseBuffer", 5));
  draw_surfelPoints_.setUniform(GlUniform<int32_t>("color_map", 7)); // for semantic map

  vao_rendered_surfels_.setVertexAttribute(0, rendered_surfels_, 4, AttributeType::FLOAT, false, sizeof(Surfel),
                                           reinterpret_cast<GLvoid*>(0));
  vao_rendered_surfels_.setVertexAttribute(1, rendered_surfels_, 4, AttributeType::FLOAT, false, sizeof(Surfel),
                                           (GLvoid*)(4 * sizeof(GLfloat)));
  vao_rendered_surfels_.setVertexAttribute(2, rendered_surfels_, 1, AttributeType::INT, false, sizeof(Surfel),
                                           (GLvoid*)(8 * sizeof(GLfloat)));
  vao_rendered_surfels_.setVertexAttribute(3, rendered_surfels_, 3, AttributeType::FLOAT, false, sizeof(Surfel),
                                           reinterpret_cast<GLvoid*>(offsetof(Surfel, color)));
  vao_rendered_surfels_.setVertexAttribute(4, rendered_surfels_, 3, AttributeType::FLOAT, false, sizeof(Surfel),
                                           reinterpret_cast<GLvoid*>(offsetof(Surfel, r)));

  //  composeRendering_ = params["compose_rendering"];

  compose_program_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/empty.vert"));
  compose_program_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shader/quad.geom"));
  compose_program_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/render_compose.frag"));
  compose_program_.link();

  //  compose_program_.setUniform(GlUniform<float>("max_distance", params["max_loop_closure_distance"]));

  compose_program_.setUniform(GlUniform<int32_t>("old_vertexmap", 0));
  compose_program_.setUniform(GlUniform<int32_t>("old_normalmap", 1));
  compose_program_.setUniform(GlUniform<int32_t>("new_vertexmap", 2));
  compose_program_.setUniform(GlUniform<int32_t>("new_normalmap", 3));
  compose_program_.setUniform(GlUniform<int32_t>("poseBuffer", 5));

  compose_program_.setUniform(GlUniform<int32_t>("new_semanticmap", 4)); // for semantic map
  compose_program_.setUniform(GlUniform<int32_t>("old_semanticmap", 6));

  submap_extent_ = float(params["submap-extent"]);
  submap_dim_ = int32_t(params["submap-dimension"]);
  submap_size_ = 2 * submap_dim_ + 1;

  partial_extraction_ = false;
  if (params.hasParam("partial-extraction")) {
    std::cout << "Extracting surfel maps partially." << std::endl;
    partial_extraction_ = params["partial-extraction"];
  }

  initializeSubmaps();

  // BEWARE: If too large, then the performance of the copy operation is really, really bad!
  // I suppose this has something to do with the location of that buffer.
  extractBuffer_.reserve(500000);
  extractFeedback_.attach(surfel_varyings, extractBuffer_);
  old_surfel_cache_.reserve(submap_size_ * extractBuffer_.capacity());

  extractProgram_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/extract_surfels.vert"));
  extractProgram_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shader/copy_surfels.geom"));
  extractProgram_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/empty.frag"));
  extractProgram_.attach(extractFeedback_);
  extractProgram_.link();

  extractProgram_.setUniform(GlUniform<int32_t>("poseBuffer", 5));

  vbo_submap_centers_.resize(submap_size_ * submap_size_);
  vao_submap_centers_.setVertexAttribute(0, vbo_submap_centers_, 2, AttributeType::FLOAT, false, 2 * sizeof(GL_FLOAT),
                                         reinterpret_cast<GLvoid*>(0));

  drawSubmaps_.attach(GlShader::fromCache(ShaderType::VERTEX_SHADER, "shader/draw_submaps.vert"));
  drawSubmaps_.attach(GlShader::fromCache(ShaderType::GEOMETRY_SHADER, "shader/draw_submaps.geom"));
  drawSubmaps_.attach(GlShader::fromCache(ShaderType::FRAGMENT_SHADER, "shader/coloredvertices.frag"));
  drawSubmaps_.link();

  drawSubmaps_.setUniform(GlUniform<float>("submap_extent", submap_extent_));

  poses_.resize(maxPoses_);
  std::fill(poses_.begin(), poses_.end(), Eigen::Matrix4f::Identity());
  poseBuffer_.assign(poses_);

  // init color_map texture
  color_map_1dtex_.setMinifyingOperation(TexMinOp::NEAREST);
  color_map_1dtex_.setMagnifyingOperation(TexMagOp::NEAREST);
  color_map_1dtex_.setWrapOperation(TexWrapOp::CLAMP_TO_BORDER, TexWrapOp::CLAMP_TO_BORDER);

//  u_char* color_map = new u_char[260*3];
//  for (uint32_t i = 0; i < 260; ++i) {
//    if (color_map_.find(i) != color_map_.end())
//    {
//      color_map[3 * i]     = std::get<2>(color_map_[i]);
//      color_map[3 * i + 1] = std::get<1>(color_map_[i]);
//      color_map[3 * i + 2] = std::get<0>(color_map_[i]);
//      std::cout << "color_map[3 * i]: " << color_map[3 * i] << std::endl;
//    }
//    else
//    {
//      color_map[3 * i]     = 0;
//      color_map[3 * i + 1] = 0;
//      color_map[3 * i + 2] = 0;
//    }
//  }
//  color_map_1dtex_.assign(PixelFormat::RGB, PixelType::UNSIGNED_BYTE, color_map);

  glow::_CheckGlError(__FILE__, __LINE__);

  setParameters(params);

  glow::_CheckGlError(__FILE__, __LINE__);
}

void SurfelMap::setParameters(const rv::ParameterList& params) {
  // pixel size corresponds to the size of a surfel scaled by a distance, i.e., size of surfel at 1 meter in world
  // coordinates, ...
  float vfov = (std::abs(float(params["data_fov_up"])) + std::abs(float(params["data_fov_down"])));
  float hfov = 360.0;
  // which is the tangens of the half per-pixel angular size...
  float vpix = std::tan(0.5f * Math::deg2rad(vfov) / uint32_t(params["data_height"])) * 1.0f;
  float hpix = std::tan(0.5f * Math::deg2rad(hfov) / uint32_t(params["data_width"])) * 1.0f;
  float pixel_size = std::max(vpix, hpix);

  float p_stable = params["p_stable"];
  float p_unstable = 1.0f - p_stable;
  float p_prior = params["p_prior"];
  float log_prior = std::log(p_prior / (1.0 - p_prior));
  float log_unstable = std::log(p_unstable / (1.0 - p_unstable));
  bool use_stability = true;
  if (params.hasParam("use_stability")) use_stability = (bool)params["use_stability"];

  float sigma_angle = params["sigma_angle"];
  float sigma_distance = params["sigma_distance"];

  int32_t active_timestamps = 100;
  if (params.hasParam("active_timestamps")) active_timestamps = params["active_timestamps"];

  initialize_program_.setUniform(GlUniform<int32_t>("vertex_map", 0));
  initialize_program_.setUniform(GlUniform<int32_t>("normal_map", 1));
  initialize_program_.setUniform(GlUniform<int32_t>("radiusConfidence_map", 2));
  initialize_program_.setUniform(GlUniform<int32_t>("measurementIntegrated_map", 4));
  initialize_program_.setUniform(GlUniform<float>("min_radius", 0.0));
  if (params.hasParam("min_radius"))
    initialize_program_.setUniform(GlUniform<float>("min_radius", params["min_radius"]));

  // for semantic map
  initialize_program_.setUniform(GlUniform<int32_t>("semantic_map", 6));
  initialize_program_.setUniform(GlUniform<int32_t>("model_semantic_map", 9));
  initialize_program_.setUniform(GlUniform<int32_t>("prior_map", 8));
  initialize_program_.setUniform(GlUniform<float>("log_prior", log_prior));
  initialize_program_.setUniform(GlUniform<float>("fov_up", std::abs((float)params["data_fov_up"])));
  initialize_program_.setUniform(GlUniform<float>("fov_down", std::abs((float)params["data_fov_down"])));
  initialize_program_.setUniform(GlUniform<float>("min_depth", params["min_depth"]));
  initialize_program_.setUniform(GlUniform<float>("max_depth", params["max_depth"]));
  initialize_program_.setUniform(GlUniform<float>("width", indexMap_.width()));
  initialize_program_.setUniform(GlUniform<float>("height", indexMap_.height()));
  initialize_program_.setUniform(GlUniform<float>("pixel_size", pixel_size));

  radConf_program_.setUniform(GlUniform<int32_t>("vertex_map", 0));
  radConf_program_.setUniform(GlUniform<int32_t>("normal_map", 1));
  radConf_program_.setUniform(GlUniform<float>("fov_up", std::abs((float)params["data_fov_up"])));
  radConf_program_.setUniform(GlUniform<float>("fov_down", std::abs((float)params["data_fov_down"])));
  radConf_program_.setUniform(GlUniform<float>("min_depth", params["min_depth"]));
  radConf_program_.setUniform(GlUniform<float>("max_depth", params["max_depth"]));
  radConf_program_.setUniform(GlUniform<float>("pixel_size", pixel_size));
  radConf_program_.setUniform(GlUniform<int32_t>("confidence_mode", params["confidence_mode"]));
  radConf_program_.setUniform(GlUniform<float>("min_radius", 0.0));
  radConf_program_.setUniform(GlUniform<float>("max_radius", 5.0));
  radConf_program_.setUniform(GlUniform<float>("angle_thresh", 0.0));
  if (params.hasParam("min_radius")) radConf_program_.setUniform(GlUniform<float>("min_radius", params["min_radius"]));
  if (params.hasParam("max_radius")) radConf_program_.setUniform(GlUniform<float>("max_radius", params["max_radius"]));
  if (params.hasParam("max_angle")) {
    float angle_thresh = std::cos(radians(float(params["max_angle"])));
    radConf_program_.setUniform(GlUniform<float>("angle_thresh", angle_thresh));
  }

  update_program_.setUniform(GlUniform<float>("fov_up", std::abs((float)params["data_fov_up"])));
  update_program_.setUniform(GlUniform<float>("fov_down", std::abs((float)params["data_fov_down"])));
  update_program_.setUniform(GlUniform<float>("min_depth", params["min_depth"]));
  update_program_.setUniform(GlUniform<float>("max_depth", params["max_depth"]));
  update_program_.setUniform(GlUniform<float>("width", indexMap_.width()));
  update_program_.setUniform(GlUniform<float>("height", indexMap_.height()));
  update_program_.setUniform(GlUniform<float>("pixel_size", pixel_size));
  update_program_.setUniform(GlUniform<float>("distance_thresh", params["map-max-distance"]));
  update_program_.setUniform(GlUniform<float>("angle_thresh", std::sin(Radians(params["map-max-angle"]))));
  update_program_.setUniform(GlUniform<int32_t>("confidence_mode", params["confidence_mode"]));
  update_program_.setUniform(GlUniform<int32_t>("vertex_map", 0));
  update_program_.setUniform(GlUniform<int32_t>("normal_map", 1));
  update_program_.setUniform(GlUniform<int32_t>("radiusConfidence_map", 2));
  update_program_.setUniform(GlUniform<int32_t>("index_map", 3));
  update_program_.setUniform(GlUniform<int32_t>("unstable_age", params["unstable_age"]));
  update_program_.setUniform(GlUniform<float>("p_stable", p_stable));
  update_program_.setUniform(GlUniform<float>("p_unstable", p_unstable));
  update_program_.setUniform(GlUniform<float>("p_prior", p_prior));
  update_program_.setUniform(GlUniform<float>("log_prior", log_prior));
  update_program_.setUniform(GlUniform<float>("log_unstable", log_unstable));
  update_program_.setUniform(GlUniform<float>("sigma_angle", sigma_angle));
  update_program_.setUniform(GlUniform<float>("sigma_distance", sigma_distance));
  update_program_.setUniform(GlUniform<float>("confidence_threshold", params["confidence_threshold"]));
  update_program_.setUniform(GlUniform<float>("min_radius", 0.0));
  update_program_.setUniform(GlUniform<float>("max_weight", 20.0));
  update_program_.setUniform(GlUniform<int32_t>("weighting_scheme", 0));
  update_program_.setUniform(GlUniform<int32_t>("averaging_scheme", 0));
  update_program_.setUniform(GlUniform<bool>("update_always", false));
  update_program_.setUniform(GlUniform<int>("active_timestamps", active_timestamps));
  update_program_.setUniform(GlUniform<bool>("use_stability", use_stability));
  update_program_.setUniform(GlUniform<int32_t>("prior_map", 8)); // for semantic map
  update_program_.setUniform(GlUniform<int32_t>("semantic_map_in", 6));

  if (params.hasParam("max_weight")) update_program_.setUniform(GlUniform<float>("max_weight", params["max_weight"]));
  if (params.hasParam("weighting_scheme"))
    update_program_.setUniform(GlUniform<int32_t>("weighting_scheme", params["weighting_scheme"]));
  if (params.hasParam("averaging_scheme"))
    update_program_.setUniform(GlUniform<int32_t>("averaging_scheme", params["averaging_scheme"]));
  if (params.hasParam("update_always"))
    update_program_.setUniform(GlUniform<bool>("update_always", params["update_always"]));

  render_program_.setUniform(GlUniform<float>("fov_up", std::abs((float)params["model_fov_up"])));
  render_program_.setUniform(GlUniform<float>("fov_down", std::abs((float)params["model_fov_down"])));
  render_program_.setUniform(GlUniform<float>("min_depth", params["model_min_depth"]));
  render_program_.setUniform(GlUniform<float>("max_depth", params["model_max_depth"]));
  render_program_.setUniform(GlUniform<float>("conf_threshold", 0.5f));
  render_program_.setUniform(GlUniform<bool>("use_stability", use_stability));

  composeRendering_ = params["compose_rendering"];

  compose_program_.setUniform(GlUniform<float>("max_distance", params["max_loop_closure_distance"]));

  oldMapFrame_ = std::make_shared<Frame>(modelWidth_, modelHeight_);
  newMapFrame_ = std::make_shared<Frame>(modelWidth_, modelHeight_);
  composedFrame_ = std::make_shared<Frame>(modelWidth_, modelHeight_);

  draw_surfels_.setUniform(GlUniform<bool>("use_stability", use_stability));
  draw_surfelPoints_.setUniform(GlUniform<bool>("use_stability", use_stability));
}

void SurfelMap::initializeSubmaps() {
  extraction_buffer_.clear();
  submap_origin_ = SubmapIndex();
  submapCache_.clear();

  std::vector<vec2> centers(submap_size_ * submap_size_);
  for (int32_t i = -submap_dim_; i <= submap_dim_; ++i) {
    for (int32_t j = -submap_dim_; j <= submap_dim_; ++j) {
      centers[offset2index(i, j)] = vec2(2.0f * i * submap_extent_, 2 * j * submap_extent_);
    }
  }
  vbo_submap_centers_.replace(0, centers);
}

void SurfelMap::reset() {
  surfels_.resize(0);

  initializeSubmaps();  // re-initialize submaps.

  timestamp_ = 0;
  poses_.resize(maxPoses_);
  std::fill(poses_.begin(), poses_.end(), Eigen::Matrix4f::Identity());
  poseBuffer_.assign(poses_);
}

/** \brief update the poses of the integrated scans (maybe, due to loop closure) **/
void SurfelMap::updatePoses(const std::vector<Eigen::Matrix4f>& poses) {
  for (uint32_t i = 0; i < poses.size(); ++i) {
    poses_[i] = poses[i];  // replacing poses => memcpy?
  }
  poseBuffer_.assign(poses_);
}

void SurfelMap::update(const Eigen::Matrix4f& pose, Frame& frame) {
  // update pose buffer.
  poses_[timestamp_] = pose;
  poseBuffer_.insert(timestamp_, pose);

  Eigen::Matrix4f inv_pose = pose.inverse();

  GLfloat cc[4];
  GLint vp[4];
  GLfloat psize;

  glGetIntegerv(GL_VIEWPORT, vp);
  glGetFloatv(GL_COLOR_CLEAR_VALUE, cc);
  glGetFloatv(GL_POINT_SIZE, &psize);

  glPointSize(1.0f);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glClearColor(0, 0, 0, 0);

  // bind texture once for all processing steps:
  glActiveTexture(GL_TEXTURE0);
  frame.vertex_map.bind();
  glActiveTexture(GL_TEXTURE1);
  frame.normal_map.bind();
  glActiveTexture(GL_TEXTURE2);
  radiusConfidenceMap_.bind();
  glActiveTexture(GL_TEXTURE3);
  indexMap_.bind();
  glActiveTexture(GL_TEXTURE4);
  measurementIntegrated_.bind();
  glActiveTexture(GL_TEXTURE5);
  poseTexture_.bind();
  glActiveTexture(GL_TEXTURE6);
  frame.semantic_map.bind();
  glActiveTexture(GL_TEXTURE7);
  color_map_1dtex_.bind();
  glActiveTexture(GL_TEXTURE8);
  newMapFrame_->semantic_map.bind();
  renderIndexmap(pose, inv_pose);

  sampler_.bind(0);
  sampler_.bind(1);
  sampler_.bind(2);
  sampler_.bind(3);
  sampler_.bind(4);
  sampler_.bind(5);
  sampler_.bind(6);

  generateDataSurfels(frame);

  updateSurfels(pose, inv_pose, frame);

  copySurfels();

  updateActiveSubmaps(pose);

  glActiveTexture(GL_TEXTURE0);
  frame.vertex_map.release();
  glActiveTexture(GL_TEXTURE1);
  frame.normal_map.release();
  glActiveTexture(GL_TEXTURE2);
  radiusConfidenceMap_.release();
  glActiveTexture(GL_TEXTURE3);
  indexMap_.release();
  glActiveTexture(GL_TEXTURE4);
  measurementIntegrated_.release();
  glActiveTexture(GL_TEXTURE5);
  poseTexture_.release();
  glActiveTexture(GL_TEXTURE6);
  frame.semantic_map.release();
  glActiveTexture(GL_TEXTURE7);
  color_map_1dtex_.bind();
  glActiveTexture(GL_TEXTURE8);
  oldMapFrame_->semantic_map.release();

  sampler_.release(0);
  sampler_.release(1);
  sampler_.release(2);
  sampler_.release(3);
  sampler_.release(4);
  sampler_.release(5);
  sampler_.release(6);

  timestamp_ += 1;

  // restore settings.
  glClearColor(cc[0], cc[1], cc[2], cc[3]);
  glViewport(vp[0], vp[1], vp[2], vp[3]);
  glPointSize(psize);

  CheckGlError();
}

void SurfelMap::renderIndexmap(const Eigen::Matrix4f& pose, Eigen::Matrix4f& inv_pose) {
  glViewport(0, 0, indexMapFramebuffer_.width(), indexMapFramebuffer_.height());
  // (0.) generate depth-buffered indexmap for updating the surfels: This tells us, which
  // surfels are currently seen...

  indexMapFramebuffer_.bind();
  indexMap_program_.bind();
  indexMap_program_.setUniform(GlUniform<Eigen::Matrix4f>("pose", pose));
  indexMap_program_.setUniform(GlUniform<Eigen::Matrix4f>("inv_pose", inv_pose));
  indexMap_program_.setUniform(GlUniform<int32_t>("timestamp", timestamp_));

  vao_surfels_.bind();
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glDrawArrays(GL_POINTS, 0, surfels_.size());
  vao_surfels_.release();

  indexMap_program_.release();
  indexMapFramebuffer_.release();
}

void SurfelMap::generateDataSurfels(Frame& frame) {
  glViewport(0, 0, radiusConfidenceFramebuffer_.width(), radiusConfidenceFramebuffer_.height());

  radConf_program_.bind();
  radiusConfidenceFramebuffer_.bind();

  vao_img_coords_.bind();
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glDrawArrays(GL_POINTS, 0, vbo_img_coords_.size());
  vao_img_coords_.release();

  radiusConfidenceFramebuffer_.release();
  radConf_program_.release();
}

void SurfelMap::updateSurfels(const Eigen::Matrix4f& pose, Eigen::Matrix4f& inv_pose, Frame& frame) {
  // (1.) update surfels with measurements & generate index map...

  glViewport(0, 0, updateFramebuffer_.width(), updateFramebuffer_.height());

  update_program_.bind();
  update_program_.setUniform(GlUniform<Eigen::Matrix4f>("pose", pose));
  update_program_.setUniform(GlUniform<Eigen::Matrix4f>("inv_pose", inv_pose));
  update_program_.setUniform(GlUniform<int32_t>("timestamp", timestamp_));

  vao_surfels_.bind();
  update_program_.bind();
  update_feedback_.bind();
  updateFramebuffer_.bind();
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  update_feedback_.begin(TransformFeedbackMode::POINTS);
  glDrawArrays(GL_POINTS, 0, surfels_.size());
  updated_surfels_.resize(update_feedback_.end());

  update_feedback_.release();
  update_program_.release();
  vao_surfels_.release();
  updateFramebuffer_.release();

  // (2.) generate surfels and determine associated surfels in model...

  glDisable(GL_DEPTH_TEST);
  initialize_feedback_.bind();
  initialize_program_.bind();
  initialize_program_.setUniform(GlUniform<Eigen::Matrix4f>("pose", pose));
  initialize_program_.setUniform(GlUniform<Eigen::Matrix4f>("inv_pose", inv_pose));
  initialize_program_.setUniform(GlUniform<int32_t>("timestamp", timestamp_));
  vao_img_coords_.bind();

  glEnable(GL_RASTERIZER_DISCARD);

  initialize_feedback_.begin(TransformFeedbackMode::POINTS);
  glDrawArrays(GL_POINTS, 0, vbo_img_coords_.size());
  data_surfels_.resize(initialize_feedback_.end());

  vao_img_coords_.release();
  initialize_program_.release();
  initialize_feedback_.release();
}

void SurfelMap::copySurfels() {
  // (4.) finally add old & new surfels.
  copy_feedback_.bind();
  copy_program_.bind();

  // set active area:
  copy_program_.setUniform(GlUniform<vec2>("submap_center", submapIndex2center(submap_origin_)));
  float extent = 2.0f * submap_dim_ * submap_extent_ + submap_extent_;

  // as we possibly postpone the extraction, we just increase the extent by one:
  if (partial_extraction_ && extraction_buffer_.size() > 0) extent += 2.0f * submap_extent_;

  copy_program_.setUniform(GlUniform<float>("submap_extent", extent));

  copy_feedback_.begin(TransformFeedbackMode::POINTS);

  // first, copy old updated and active map surfels...
  vao_updated_surfels_.bind();
  glDrawArrays(GL_POINTS, 0, updated_surfels_.size());
  vao_updated_surfels_.release();

  // secondly, copy new surfels...
  vao_data_surfels_.bind();
  glDrawArrays(GL_POINTS, 0, data_surfels_.size());
  vao_data_surfels_.release();

  surfels_.resize(copy_feedback_.end());
  copy_program_.release();
  copy_feedback_.release();

  glDisable(GL_RASTERIZER_DISCARD);
}

uint32_t SurfelMap::offset2index(int32_t i, int32_t j) {
  return (i + submap_dim_) + (j + submap_dim_) * submap_size_;
}

vec2 SurfelMap::submapIndex2center(const SubmapIndex& idx) {
  return vec2(2.0 * idx.i * submap_extent_, 2.0 * idx.j * submap_extent_);
}

void SurfelMap::extractSurfels(bool partially) {
  glEnable(GL_RASTERIZER_DISCARD);

  extractProgram_.bind();
  extractFeedback_.bind();
  vao_surfels_.bind();

  while (!extraction_buffer_.empty()) {
    SubmapIndex idx = extraction_buffer_.back();
    extraction_buffer_.pop_back();

    vec2 ctr = submapIndex2center(idx);
    extractProgram_.setUniform(GlUniform<vec2>("submap_center", ctr));
    extractProgram_.setUniform(GlUniform<float>("submap_extent", submap_extent_));

    extractFeedback_.begin(TransformFeedbackMode::POINTS);
    glDrawArrays(GL_POINTS, 0, surfels_.size());
    uint32_t extractedSize = extractFeedback_.end();
    if (extractedSize >= extractBuffer_.capacity())
      std::cout << "Warning: potential lost of information!!!" << std::endl;

    extractBuffer_.resize(extractedSize);

    SubmapCache& cachedSubmap = submapCache_[idx];
    extractBuffer_.get(cachedSubmap.surfels);

    if (partially) break;
  }

  vao_surfels_.release();
  extractFeedback_.release();
  extractProgram_.release();

  glDisable(GL_RASTERIZER_DISCARD);
}

void SurfelMap::updateActiveSubmaps(const Eigen::Matrix4f& pose) {
  // check if we have to update the active submaps.
  vec2 current_pos = vec2(pose(0, 3), pose(1, 3));
  vec2 submap_center = submapIndex2center(submap_origin_);

  float changex = current_pos.x - submap_center.x;  // forward.
  float changey = current_pos.y - submap_center.y;  // left right.
  float factor = 1.1;

  if (std::abs(changex) > factor * submap_extent_ || std::abs(changey) > factor * submap_extent_) {
    // remove stuff before next update, otherwise data loss possible.
    // FIXME:    if (!extraction_buffer_.empty()) extractSurfels(false);

    if (std::abs(changex) > factor * submap_extent_) {
      int32_t dir = direction(changex);
      // (1) trigger download relevant data.
      for (int32_t c = -submap_dim_; c <= submap_dim_; ++c) {
        SubmapIndex idx(submap_origin_.i - dir * submap_dim_, submap_origin_.j + c);

        extraction_buffer_.push_back(idx);
      }

      // (2) shift map.
      submap_origin_.i += dir;

      // (3) upload data in single chunk.
      old_surfel_cache_.clear();
      for (int32_t c = -submap_dim_; c <= submap_dim_; ++c) {
        SubmapIndex idx(submap_origin_.i + dir * submap_dim_, submap_origin_.j + c);
        SubmapCache& cachedSubmap = submapCache_[idx];

        old_surfel_cache_.insert(old_surfel_cache_.end(), cachedSubmap.surfels.begin(), cachedSubmap.surfels.end());
      }

      uint32_t old_size = surfels_.size();
      surfels_.resize(surfels_.size() + old_surfel_cache_.size());
      surfels_.replace(old_size, old_surfel_cache_);
    }

    if (std::abs(changey) > factor * submap_extent_) {
      int32_t dir = direction(changey);

      // (1) trigger download of relevant data.
      for (int32_t r = -submap_dim_; r <= submap_dim_; ++r) {
        SubmapIndex idx(submap_origin_.i + r, submap_origin_.j - dir * submap_dim_);
        extraction_buffer_.push_back(idx);
      }

      // (2) shift map.
      submap_origin_.j += dir;

      // (3) upload data in single chunk.
      old_surfel_cache_.clear();
      for (int32_t r = -submap_dim_; r <= submap_dim_; ++r) {
        SubmapIndex idx(submap_origin_.i + r, submap_origin_.j + dir * submap_dim_);
        SubmapCache& cachedSubmap = submapCache_[idx];

        old_surfel_cache_.insert(old_surfel_cache_.end(), cachedSubmap.surfels.begin(), cachedSubmap.surfels.end());
      }

      uint32_t old_size = surfels_.size();
      surfels_.resize(surfels_.size() + old_surfel_cache_.size());
      surfels_.replace(old_size, old_surfel_cache_);
    }

    std::vector<vec2> centers(submap_size_ * submap_size_);
    for (int32_t i = -submap_dim_; i <= submap_dim_; ++i) {
      for (int32_t j = -submap_dim_; j <= submap_dim_; ++j) {
        SubmapIndex idx = submap_origin_;
        idx += SubmapIndex(i, j);
        centers[offset2index(i, j)] = submapIndex2center(idx);
      }
    }

    vbo_submap_centers_.replace(0, centers);

    glFinish();
  }

  if (!extraction_buffer_.empty()) extractSurfels(partial_extraction_);
}

int32_t SurfelMap::direction(float a) {
  if (a < 0) return -1;
  return 1;
}

std::shared_ptr<Frame>& SurfelMap::oldMapFrame() {
  return oldMapFrame_;
}

std::shared_ptr<Frame>& SurfelMap::newMapFrame() {
  return newMapFrame_;
}

std::shared_ptr<Frame>& SurfelMap::composedFrame() {
  return composedFrame_;
}

void SurfelMap::render(const Eigen::Matrix4f& pose, Frame& frame, float confidence_threshold) {
  render(pose, pose, frame, confidence_threshold);
}

void SurfelMap::render(const Eigen::Matrix4f& pose_old, const Eigen::Matrix4f& pose_new, Frame& frame,
                       float confidence_threshold) {
  GLfloat cc[4];
  GLint vp[4];
  //  GLenum dt;

  if (composeRendering_) {
    glGetIntegerv(GL_VIEWPORT, vp);
    glGetFloatv(GL_COLOR_CLEAR_VALUE, cc);

    glEnable(GL_DEPTH_TEST);
    glClearColor(0, 0, 0, 0);
    glDepthFunc(GL_LESS);
    glViewport(0, 0, renderFramebuffer_.width(), renderFramebuffer_.height());

    glActiveTexture(GL_TEXTURE5);
    poseTexture_.bind();

    renderFramebuffer_.bind();
    render_program_.bind();
    vao_surfels_.bind();

    // FIXME: can we avoid rendering of all surfels by just keeping track of currently old surfels?

    // -- render old map parts.
    render_program_.setUniform(GlUniform<float>("conf_threshold", confidence_threshold));
    render_program_.setUniform(GlUniform<int>("timestamp_threshold", timestamp_ - composeSurfelAge_));

    render_program_.setUniform(GlUniform<Eigen::Matrix4f>("inv_pose", pose_old.inverse()));
    render_program_.setUniform(GlUniform<bool>("render_old_surfels", true));

    renderFramebuffer_.attach(FramebufferAttachment::COLOR0, oldMapFrame_->vertex_map);
    renderFramebuffer_.attach(FramebufferAttachment::COLOR1, oldMapFrame_->normal_map);
    renderFramebuffer_.attach(FramebufferAttachment::COLOR2, oldMapFrame_->semantic_map); // for semantic map
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDrawArrays(GL_POINTS, 0, surfels_.size());

    render_program_.setUniform(GlUniform<Eigen::Matrix4f>("inv_pose", pose_new.inverse()));
    render_program_.setUniform(GlUniform<bool>("render_old_surfels", false));

    renderFramebuffer_.attach(FramebufferAttachment::COLOR0, newMapFrame_->vertex_map);
    renderFramebuffer_.attach(FramebufferAttachment::COLOR1, newMapFrame_->normal_map);
    renderFramebuffer_.attach(FramebufferAttachment::COLOR2, newMapFrame_->semantic_map); // for semantic map
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDrawArrays(GL_POINTS, 0, surfels_.size());

    render_program_.setUniform(GlUniform<Eigen::Matrix4f>("inv_pose", pose_old.inverse()));
    render_program_.setUniform(GlUniform<bool>("render_old_surfels", true));

    // compose both views, take just nearest surfels (FIXME: avoid rendering two times!)
    renderFramebuffer_.attach(FramebufferAttachment::COLOR0, composedFrame_->vertex_map);
    renderFramebuffer_.attach(FramebufferAttachment::COLOR1, composedFrame_->normal_map);
    renderFramebuffer_.attach(FramebufferAttachment::COLOR2, composedFrame_->semantic_map); // for semantic map
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDrawArrays(GL_POINTS, 0, surfels_.size());

    render_program_.setUniform(GlUniform<Eigen::Matrix4f>("inv_pose", pose_new.inverse()));
    render_program_.setUniform(GlUniform<bool>("render_old_surfels", false));
    // Note: not clearing!
    glDrawArrays(GL_POINTS, 0, surfels_.size());

    vao_surfels_.release();
    render_program_.release();

    compose_program_.bind();
    vao_no_points_.bind();
    renderFramebuffer_.attach(FramebufferAttachment::COLOR0, frame.vertex_map);
    renderFramebuffer_.attach(FramebufferAttachment::COLOR1, frame.normal_map);
    renderFramebuffer_.attach(FramebufferAttachment::COLOR2, frame.semantic_map); // for semantic map

    glActiveTexture(GL_TEXTURE0);
    oldMapFrame_->vertex_map.bind();
    glActiveTexture(GL_TEXTURE1);
    oldMapFrame_->normal_map.bind();

    glActiveTexture(GL_TEXTURE2);
    newMapFrame_->vertex_map.bind();
    glActiveTexture(GL_TEXTURE3);
    newMapFrame_->normal_map.bind();

    glActiveTexture(GL_TEXTURE4);
    newMapFrame_->semantic_map.bind();
    glActiveTexture(GL_TEXTURE6);
    oldMapFrame_->semantic_map.bind();

    sampler_.bind(0);
    sampler_.bind(1);
    sampler_.bind(2);
    sampler_.bind(3);
    sampler_.bind(4);
    sampler_.bind(6);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDrawArrays(GL_POINTS, 0, 1);

    glActiveTexture(GL_TEXTURE0);
    oldMapFrame_->vertex_map.release();
    glActiveTexture(GL_TEXTURE1);
    oldMapFrame_->normal_map.release();

    glActiveTexture(GL_TEXTURE2);
    newMapFrame_->vertex_map.release();
    glActiveTexture(GL_TEXTURE3);
    newMapFrame_->normal_map.release();

    glActiveTexture(GL_TEXTURE4);
    newMapFrame_->semantic_map.release();
    glActiveTexture(GL_TEXTURE6);
    oldMapFrame_->semantic_map.release();

    glActiveTexture(GL_TEXTURE5);
    poseTexture_.release();

    vao_no_points_.release();
    compose_program_.release();
    renderFramebuffer_.release();

    sampler_.release(0);
    sampler_.release(1);
    sampler_.release(2);
    sampler_.release(3);
    sampler_.release(4);
    sampler_.release(6);

    glViewport(vp[0], vp[1], vp[2], vp[3]);
    glClearColor(cc[0], cc[1], cc[2], cc[3]);

    oldMapFrame_->valid = true;
    newMapFrame_->valid = true;
  } else {
    glGetIntegerv(GL_VIEWPORT, vp);
    glGetFloatv(GL_COLOR_CLEAR_VALUE, cc);

    render_program_.setUniform(GlUniform<Eigen::Matrix4f>("inv_pose", pose_old.inverse()));
    render_program_.setUniform(GlUniform<float>("conf_threshold", confidence_threshold));
    render_program_.setUniform(GlUniform<bool>("render_old_surfels", false));
    render_program_.setUniform(GlUniform<int>("timestamp_threshold", 0));

    renderFramebuffer_.attach(FramebufferAttachment::COLOR0, frame.vertex_map);
    renderFramebuffer_.attach(FramebufferAttachment::COLOR1, frame.normal_map);
    renderFramebuffer_.attach(FramebufferAttachment::COLOR2, frame.semantic_map);

    glEnable(GL_DEPTH_TEST);
    glClearColor(0, 0, 0, 0);
    glDepthFunc(GL_LESS);
    glViewport(0, 0, renderFramebuffer_.width(), renderFramebuffer_.height());

    renderFramebuffer_.bind();
    render_program_.bind();
    vao_surfels_.bind();

    glActiveTexture(GL_TEXTURE5);
    poseTexture_.bind();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDrawArrays(GL_POINTS, 0, surfels_.size());

    glActiveTexture(GL_TEXTURE5);
    poseTexture_.release();

    vao_surfels_.release();
    render_program_.release();
    renderFramebuffer_.release();

    glViewport(vp[0], vp[1], vp[2], vp[3]);
    glClearColor(cc[0], cc[1], cc[2], cc[3]);
    CheckGlError();

    // copy new, oldmap frame.
    newMapFrame_->copy(frame);
    oldMapFrame_->copy(frame);
  }

  CheckGlError();
}

void SurfelMap::render_active(const Eigen::Matrix4f& pose, float confidence_threshold) {
  GLfloat cc[4];
  GLint vp[4];

  glGetIntegerv(GL_VIEWPORT, vp);
  glGetFloatv(GL_COLOR_CLEAR_VALUE, cc);

  glEnable(GL_DEPTH_TEST);
  glClearColor(0, 0, 0, 0);
  glDepthFunc(GL_LESS);
  glViewport(0, 0, renderFramebuffer_.width(), renderFramebuffer_.height());

  glActiveTexture(GL_TEXTURE5);
  poseTexture_.bind();

  renderFramebuffer_.bind();
  render_program_.bind();
  vao_surfels_.bind();

  render_program_.setUniform(GlUniform<int>("timestamp_threshold", timestamp_ - composeSurfelAge_));
  render_program_.setUniform(GlUniform<float>("conf_threshold", confidence_threshold));
  render_program_.setUniform(GlUniform<Eigen::Matrix4f>("inv_pose", pose.inverse()));
  render_program_.setUniform(GlUniform<bool>("render_old_surfels", false));

  renderFramebuffer_.attach(FramebufferAttachment::COLOR0, newMapFrame_->vertex_map);
  renderFramebuffer_.attach(FramebufferAttachment::COLOR1, newMapFrame_->normal_map);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glDrawArrays(GL_POINTS, 0, surfels_.size());

  vao_surfels_.release();
  render_program_.release();

  glActiveTexture(GL_TEXTURE5);
  poseTexture_.release();
  renderFramebuffer_.release();

  sampler_.release(0);
  sampler_.release(1);
  sampler_.release(2);
  sampler_.release(3);
  sampler_.release(6); // for semantic map

  glViewport(vp[0], vp[1], vp[2], vp[3]);
  glClearColor(cc[0], cc[1], cc[2], cc[3]);

  newMapFrame_->valid = true;
}

void SurfelMap::render_inactive(const Eigen::Matrix4f& pose, float confidence_threshold) {
  GLfloat cc[4];
  GLint vp[4];

  glGetIntegerv(GL_VIEWPORT, vp);
  glGetFloatv(GL_COLOR_CLEAR_VALUE, cc);

  glEnable(GL_DEPTH_TEST);
  glClearColor(0, 0, 0, 0);
  glDepthFunc(GL_LESS);
  glViewport(0, 0, renderFramebuffer_.width(), renderFramebuffer_.height());

  glActiveTexture(GL_TEXTURE5);
  poseTexture_.bind();

  renderFramebuffer_.bind();
  render_program_.bind();
  vao_surfels_.bind();

  // -- render old map parts.
  render_program_.setUniform(GlUniform<float>("conf_threshold", confidence_threshold));
  render_program_.setUniform(GlUniform<int>("timestamp_threshold", timestamp_ - composeSurfelAge_));

  render_program_.setUniform(GlUniform<Eigen::Matrix4f>("inv_pose", pose.inverse()));
  render_program_.setUniform(GlUniform<bool>("render_old_surfels", true));

  renderFramebuffer_.attach(FramebufferAttachment::COLOR0, oldMapFrame_->vertex_map);
  renderFramebuffer_.attach(FramebufferAttachment::COLOR1, oldMapFrame_->normal_map);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glDrawArrays(GL_POINTS, 0, surfels_.size());

  vao_surfels_.release();
  render_program_.release();

  glActiveTexture(GL_TEXTURE5);
  poseTexture_.release();

  renderFramebuffer_.release();

  glViewport(vp[0], vp[1], vp[2], vp[3]);
  glClearColor(cc[0], cc[1], cc[2], cc[3]);

  oldMapFrame_->valid = true;
}

void SurfelMap::render_composed(const Eigen::Matrix4f& pose_old, const Eigen::Matrix4f& pose_new,
                                float confidence_threshold) {
  GLfloat cc[4];
  GLint vp[4];

  glGetIntegerv(GL_VIEWPORT, vp);
  glGetFloatv(GL_COLOR_CLEAR_VALUE, cc);

  glEnable(GL_DEPTH_TEST);
  glClearColor(0, 0, 0, 0);
  glDepthFunc(GL_LEQUAL);
  glViewport(0, 0, renderFramebuffer_.width(), renderFramebuffer_.height());

  glActiveTexture(GL_TEXTURE5);
  poseTexture_.bind();

  renderFramebuffer_.bind();
  render_program_.bind();
  vao_surfels_.bind();

  render_program_.setUniform(GlUniform<float>("conf_threshold", confidence_threshold));
  render_program_.setUniform(GlUniform<int>("timestamp_threshold", timestamp_ - composeSurfelAge_));

  // compose both views, take just nearest surfels (FIXME: avoid rendering two times!)
  renderFramebuffer_.attach(FramebufferAttachment::COLOR0, composedFrame_->vertex_map);
  renderFramebuffer_.attach(FramebufferAttachment::COLOR1, composedFrame_->normal_map);

  render_program_.setUniform(GlUniform<Eigen::Matrix4f>("inv_pose", pose_old.inverse()));
  render_program_.setUniform(GlUniform<bool>("render_old_surfels", true));

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glDrawArrays(GL_POINTS, 0, surfels_.size());

  render_program_.setUniform(GlUniform<Eigen::Matrix4f>("inv_pose", pose_new.inverse()));
  render_program_.setUniform(GlUniform<bool>("render_old_surfels", false));
  // Note: not clearing!
  glDrawArrays(GL_POINTS, 0, surfels_.size());

  vao_surfels_.release();
  render_program_.release();
  renderFramebuffer_.release();

  glActiveTexture(GL_TEXTURE5);
  poseTexture_.release();

  glViewport(vp[0], vp[1], vp[2], vp[3]);
  glClearColor(cc[0], cc[1], cc[2], cc[3]);

  composedFrame_->valid = true;
}

void SurfelMap::draw(const Eigen::Matrix4f& modelview, const SurfelMapVisualOptions& params) {
  // render current surfels and complete (active) map.

  float psize;
  glGetFloatv(GL_POINT_SIZE, &psize);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  // drawing surfels.

  GlProgram* draw_program = &draw_surfels_;
  if (params.drawPoints) {
    draw_program = &draw_surfelPoints_;
    glPointSize(2.0f * psize);
  }

  draw_program->bind();
  CheckGlError();

  draw_program->setUniform(GlUniform<Eigen::Matrix4f>("mvp", modelview));
  draw_program->setUniform(GlUniform<int32_t>("colorMode", params.colorMode));
  draw_program->setUniform(GlUniform<float>("conf_threshold", params.confidence_threshold));
  draw_program->setUniform(GlUniform<vec3>("view_pos", params.view_pos));
  draw_program->setUniform(GlUniform<int32_t>("timestamp", timestamp_ - 1));
  draw_program->setUniform(GlUniform<bool>("drawCurrentSurfelsOnly", false));
  draw_program->setUniform(GlUniform<bool>("backface_culling", params.backfaceCulling));

  glActiveTexture(GL_TEXTURE5);
  poseTexture_.bind();
  glActiveTexture(GL_TEXTURE7);
  color_map_1dtex_.bind();

  if (params.drawUpdatedSurfels) {
    draw_program->setUniform(GlUniform<bool>("drawCurrentSurfelsOnly", true));
    vao_updated_surfels_.bind();
    glDrawArrays(GL_POINTS, 0, updated_surfels_.size());
    vao_updated_surfels_.release();
  } else {
    vao_surfels_.bind();
    glDrawArrays(GL_POINTS, 0, surfels_.size());
    vao_surfels_.release();
  }

  draw_program->release();

  glDisable(GL_DEPTH_TEST);
  if (params.drawSubmaps) {
    //    std::cout << vbo_submap_centers_.size() << std::endl;
    vao_submap_centers_.bind();
    drawSubmaps_.bind();
    drawSubmaps_.setUniform(GlUniform<Eigen::Matrix4f>("mvp", modelview));
    glDrawArrays(GL_POINTS, 0, vbo_submap_centers_.size());
    drawSubmaps_.release();
    vao_submap_centers_.release();
  }
  glEnable(GL_DEPTH_TEST);

  glActiveTexture(GL_TEXTURE5);
  poseTexture_.release();
  glActiveTexture(GL_TEXTURE7);
  color_map_1dtex_.release();

  glPointSize(psize);
}

std::vector<Surfel> SurfelMap::getAllSurfels()
{
    std::vector<Surfel> all_surfels;
    surfels_.get(all_surfels);
    return all_surfels;
}

void SurfelMap::setColorMap(std::map<uint32_t, semantic_color> colormap) {
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
