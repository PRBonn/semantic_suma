#ifndef INCLUDE_CORE_FRAME_H_
#define INCLUDE_CORE_FRAME_H_

#include <cassert>

#include <glow/GlBuffer.h>
#include <glow/GlTexture.h>
#include <glow/GlTextureRectangle.h>
#include <eigen3/Eigen/Dense>
#include <rv/geometry.h>
#include <memory>

class SurfelMap;

/** \brief data class holding all information for a specific timestep.
 *
 *  \author behley
 *  \author Xieyuanli Chen
 */

class Frame {
 public:
  typedef std::shared_ptr<Frame> Ptr;

  Frame(uint32_t w, uint32_t h)
      : valid(false),
        width(w),
        height(h),
        vertex_map(width, height, glow::TextureFormat::RGBA_FLOAT),
        normal_map(width, height, glow::TextureFormat::RGBA_FLOAT),
        semantic_map(width, height, glow::TextureFormat::RGBA_FLOAT), // for semantic map
  residual_map(width, height, glow::TextureFormat::RGBA_FLOAT)
  {
    points.reserve(150000);

    // interpolation parameters.
    vertex_map.setMinifyingOperation(glow::TexRectMinOp::NEAREST);
    vertex_map.setMagnifyingOperation(glow::TexRectMagOp::NEAREST);
    vertex_map.setWrapOperation(glow::TexRectWrapOp::CLAMP_TO_BORDER, glow::TexRectWrapOp::CLAMP_TO_BORDER);
    normal_map.setMinifyingOperation(glow::TexRectMinOp::NEAREST);
    normal_map.setMagnifyingOperation(glow::TexRectMagOp::NEAREST);
    normal_map.setWrapOperation(glow::TexRectWrapOp::CLAMP_TO_BORDER, glow::TexRectWrapOp::CLAMP_TO_BORDER);
    semantic_map.setMinifyingOperation(glow::TexRectMinOp::NEAREST);
    semantic_map.setMagnifyingOperation(glow::TexRectMagOp::NEAREST);
    semantic_map.setWrapOperation(glow::TexRectWrapOp::CLAMP_TO_BORDER, glow::TexRectWrapOp::CLAMP_TO_BORDER);
    CheckGlError();
  }

  void copy(const Frame& other) {
    assert(width == other.width && height == other.height && "Frame dimensions must match.");

    valid = other.valid;
    vertex_map.copy(other.vertex_map);
    normal_map.copy(other.normal_map);
    points.assign(other.points);
    residual_map.copy(other.residual_map);
    semantic_map.copy(other.semantic_map);
    map = other.map;

    pose = other.pose;
  }

  bool valid;
  uint32_t width, height;

  glow::GlTextureRectangle vertex_map;  // (x,y,z) & w encodes validity
  glow::GlTextureRectangle normal_map;  // (x,y,z) & w encodes validity
  glow::GlTextureRectangle semantic_map; // for semantic map

  glow::GlBuffer<rv::Point3f> points{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlBuffer<float> labels{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlBuffer<float> probs{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};

  Eigen::Matrix4f pose{Eigen::Matrix4f::Identity()};  // estimated pose of that frame.

  std::shared_ptr<SurfelMap> map;

  glow::GlTextureRectangle residual_map;
};

#endif /* INCLUDE_CORE_FRAME_H_ */
