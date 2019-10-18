#ifndef SRC_CORE_PREPROCESSING_H_
#define SRC_CORE_PREPROCESSING_H_

#include <rv/ParameterList.h>
#include <glow/GlBuffer.h>
#include <glow/GlFramebuffer.h>
#include <glow/GlTextureRectangle.h>
#include <glow/GlSampler.h>
#include <glow/GlProgram.h>
#include <glow/GlVertexArray.h>
#include <vector>
#include <rv/geometry.h>

#include <map>

#include "core/Frame.h"
#include "util/kitti_utils.h"

/** \brief Preprocessing of the data.
 *
 *  We essentially take the point cloud, convert it to polar coordinates and generate from
 *  this a vertex map. We furthermore
 *
 *  Point cloud's coordinate system should be right-handed coordinate system: x - forward, y - left, z - up.
 *  The generated vertex map and normal map has normalized yaw/azimuth angles on the x-axis(right) and the normalized
 *  pitch angles on the y-axis (up). Yaw angles ranges from [-pi, pi], where 0 is in the middle of the image.
 *  The pitch angle ranges from [0, |fov_up| + |fov_down|] and starts at the upper border of the depth image.
 *
 *  the coordinate system therefore looks something like this:
 *      ---------------------------
 *      |<--- -yaw- (0,0) -+yaw-->|
 *      y             |           |
 *      |             v           |
 *      O--x-->--------------------
 *
 *  The vertex map and normal map are represented by an OpenGl texture and therefore have their origin in the lower
 *left.
 *
 *  The vertex map and normal map store in the fourth coordinate if the point is valid. If the vertex or
 *  normal is valid, the fourth coordinate is 1; otherwise 0.
 *
 *  \author behley
 *
 *  \author Xieyuanli Chen
 **/

class Preprocessing {
 public:
  Preprocessing(const rv::ParameterList& params);

  void setParameters(const rv::ParameterList& params);

  /** \brief pre-process the point cloud (vertex map, normal map, semantic_map, ... etc.)
   * and store results in frame. **/
  void process(glow::GlBuffer<rv::Point3f>& points, Frame& frame, glow::GlBuffer<float>& labels,
               glow::GlBuffer<float>& probs, uint32_t timestamp_);

  void setCalibration(const KITTICalibration& calib); // for semantic map

 protected:
  uint32_t width_, height_;
  glow::GlProgram depth_program_, normal_program_, bilateral_program_, avg_program_, floodfill_program_;
  glow::GlVertexArray vao_points_;     // laser points
  glow::GlVertexArray vao_no_points_;  // no points.
  glow::GlVertexArray vao_img_verts_;  // for each image coordinate, we have an point.
  glow::GlSampler sampler_;
  glow::GlFramebuffer framebuffer_;
  glow::GlFramebuffer semanticbuffer_;
  glow::GlTextureRectangle temp_vertices_;

  bool filterVertexmap_{false};
  bool useFilteredVertexmap_{true};
  bool avgVertexmap_{false};

  KITTICalibration calib_; // for semantic map
  Eigen::Matrix4f Tr;
  Eigen::Matrix4f Tr_inv;
  Eigen::Matrix4f P2;
};

#endif /* SRC_CORE_PREPROCESSING_H_ */
