#ifndef INCLUDE_CORE_SURFELMAP_H_
#define INCLUDE_CORE_SURFELMAP_H_

#include <glow/GlFramebuffer.h>
#include <glow/GlProgram.h>
#include <glow/GlSampler.h>
#include <glow/GlTextureBuffer.h>
#include <glow/GlTransformFeedback.h>
#include <glow/GlVertexArray.h>
#include <glow/glutil.h>
#include <rv/ParameterList.h>
#include <unordered_map>
#include "Frame.h"
#include "Surfel.h"

typedef std::tuple< u_char, u_char, u_char> semantic_color;

/** \brief Parameters for rendering the map.
 *  \author behley
 *
 *  \author Xieyuanli Chen
**/
struct SurfelMapVisualOptions {
 public:
  uint32_t colorMode{5};
  float confidence_threshold{10.0f};
  bool drawUpdatedSurfels{false};
  bool drawRenderedSurfels{false};
  glow::vec3 view_pos;
  bool backfaceCulling{false};
  bool drawSubmaps{false};
  bool drawPoints{false};
};

/** \brief surfel-based map representation. **/
class SurfelMap {
 public:
  SurfelMap(const rv::ParameterList& params);

  void setParameters(const rv::ParameterList& params);

  /** \brief remove all prior information from the map **/
  void reset();
  /** \brief integrate the given frame into the surfel-based map representation. **/
  void update(const Eigen::Matrix4f& pose, Frame& frame);

  /** \brief render the surfel map into the given frame. **/
  void render(const Eigen::Matrix4f& pose, Frame& frame, float confidence_threshold);
  void render(const Eigen::Matrix4f& pose_old, const Eigen::Matrix4f& pose_new, Frame& frame,
              float confidence_threshold);

  void render_active(const Eigen::Matrix4f& pose, float confidence_threshold);
  void render_inactive(const Eigen::Matrix4f& pose, float confidence_threshold);
  void render_composed(const Eigen::Matrix4f& pose_old, const Eigen::Matrix4f& pose_new, float confidence_threshold);

  /** \brief draw (or render) the surfel map with given modelview matrix. **/
  void draw(const Eigen::Matrix4f& modelview, const SurfelMapVisualOptions& params);

  std::shared_ptr<Frame>& oldMapFrame();
  std::shared_ptr<Frame>& newMapFrame();
  std::shared_ptr<Frame>& composedFrame();

  /** \brief get surfels estimated from current scans. **/
  glow::GlBuffer<Surfel> getDataSurfels() { return data_surfels_; }

  /** \brief get surfels currently inside the model. **/
  glow::GlBuffer<Surfel> getModelSurfels() { return surfels_; }

  uint32_t size() const { return surfels_.size(); }

  /** \brief update the poses of the integrated scans (maybe, due to loop closure) **/
  void updatePoses(const std::vector<Eigen::Matrix4f>& poses);

  /** \brief get all surfels in std::vector<Surfel> format **/
  std::vector<Surfel> getAllSurfels();

  /** \brief set the color map used for semantic mapping **/
  void setColorMap(std::map<uint32_t, semantic_color> colormap);

 protected:
  void initializeSubmaps();

  uint32_t timestamp_{0};

  uint32_t dataWidth_, dataHeight_;
  uint32_t modelWidth_, modelHeight_;
  uint32_t maxNumSurfels_{2048 * 2048};  //{4096 * 4096};  // 16.777.216 surfels.
  float timeDelta_{20};
  Eigen::Matrix4f currentPose_;

  glow::GlSampler sampler_;

  glow::GlVertexArray vao_img_coords_;
  glow::GlBuffer<glow::vec2> vbo_img_coords_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::STATIC_DRAW};

  glow::GlVertexArray vao_surfels_;  // rendering of surfels.
  glow::GlVertexArray vao_updated_surfels_;
  glow::GlVertexArray vao_data_surfels_;

  glow::GlBuffer<Surfel> surfels_{glow::BufferTarget::ARRAY_BUFFER,
                                  glow::BufferUsage::DYNAMIC_DRAW};  // feedback stores updated surfels inside surfels_.
  glow::GlBuffer<Surfel> updated_surfels_{
      glow::BufferTarget::ARRAY_BUFFER,
      glow::BufferUsage::DYNAMIC_DRAW};  // intermediate buffer needed for surfel update_.
  glow::GlBuffer<Surfel> data_surfels_{glow::BufferTarget::ARRAY_BUFFER,
                                       glow::BufferUsage::DYNAMIC_DRAW};  // surfels generate from the input data.

  glow::GlTransformFeedback initialize_feedback_;  // generate surfels from data...
  glow::GlTransformFeedback update_feedback_;      // generate updated surfels_ and discard surfels_
  glow::GlTransformFeedback copy_feedback_;        // updating surfels_ by integrating update_surfels_ and data_surfels_

  glow::GlFramebuffer indexMapFramebuffer_;
  glow::GlTextureRectangle indexMap_;
  glow::GlTextureRectangle indexVertexMap_;
  glow::GlTextureRectangle indexNormalMap_;

  glow::GlFramebuffer updateFramebuffer_;
  glow::GlTextureRectangle measurementIntegrated_;

  glow::GlFramebuffer renderFramebuffer_;
  glow::GlBuffer<Surfel> rendered_surfels_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlVertexArray vao_rendered_surfels_;

  glow::GlFramebuffer radiusConfidenceFramebuffer_;
  glow::GlTextureRectangle centerizedVertexMap_;
  glow::GlTextureRectangle filteredNormalMap_;
  glow::GlTextureRectangle radiusConfidenceMap_;

  glow::GlProgram indexMap_program_;
  glow::GlProgram initialize_program_;  // generate new surfels.
  glow::GlProgram update_program_;      // integrate measurement surfels.
  glow::GlProgram render_program_;      // render surfels to vertex, normal map.
  glow::GlProgram draw_surfels_;        // visualize surfels.
  glow::GlProgram draw_surfelPoints_;   // visualize surfels as points.
  glow::GlProgram copy_program_;        // copy generated and updated surfels to combined surfel buffer.
  glow::GlProgram radConf_program_;     // pre-compute radius, confidence for measurements... (bilateral filtering?)

  glow::GlProgram compose_program_;  // compose multiple surfel renderings
  std::shared_ptr<Frame> oldMapFrame_;
  std::shared_ptr<Frame> newMapFrame_;
  std::shared_ptr<Frame> composedFrame_;

  bool composeRendering_{false};
  uint32_t composeSurfelAge_{100};
  glow::GlVertexArray vao_no_points_;

  int32_t direction(float a);

  void updateActiveSubmaps(const Eigen::Matrix4f& pose);
  void copySurfels();
  void renderIndexmap(const Eigen::Matrix4f& pose, Eigen::Matrix4f& inv_pose);
  void generateDataSurfels(Frame& frame);
  void updateSurfels(const Eigen::Matrix4f& pose, Eigen::Matrix4f& inv_pose, Frame& frame);

  void extractSurfels(bool partially);

  class SubmapIndex {
   public:
    SubmapIndex() : i(0), j(0) {}
    SubmapIndex(int32_t i, int32_t j) : i(i), j(j) {}

    std::size_t operator()(SubmapIndex const& value) const {
      // simple hash function assuming that all values are in [-100000, 100000]
      return (i + 100000) + (j + 100000) * 200000;
    }

    SubmapIndex& operator+=(const SubmapIndex& rhs) {
      i += rhs.i;
      j += rhs.j;

      return *this;
    }

    bool operator==(const SubmapIndex& other) const { return (i == other.i && j == other.j); }

    int32_t i, j;
  };

  struct SubmapCache {
    std::vector<Surfel> surfels;
  };

  glow::vec2 submapIndex2center(const SubmapIndex& idx);
  uint32_t offset2index(int32_t i, int32_t j);

  std::unordered_map<SubmapIndex, SubmapCache, SubmapIndex> submapCache_;

  SubmapIndex submap_origin_;
  int32_t submap_dim_;
  float submap_extent_;  // half of submap's side length
  int32_t submap_size_;
  bool partial_extraction_{false};

  glow::GlBuffer<Surfel> extractBuffer_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlProgram extractProgram_;
  glow::GlTransformFeedback extractFeedback_;

  glow::GlVertexArray vao_submap_centers_;
  glow::GlBuffer<glow::vec2> vbo_submap_centers_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlProgram drawSubmaps_;

  std::vector<Surfel> old_surfel_cache_;
  std::vector<SubmapIndex> extraction_buffer_;

  uint32_t maxPoses_{10000};
  std::vector<Eigen::Matrix4f> poses_;
  glow::GlBuffer<Eigen::Matrix4f> poseBuffer_{glow::BufferTarget::TEXTURE_BUFFER, glow::BufferUsage::DYNAMIC_DRAW};
  glow::GlTextureBuffer poseTexture_;

  // for semantic map
  glow::GlTexture color_map_1dtex_;
  std::map<uint32_t, semantic_color> color_map_;

};

#endif /* INCLUDE_CORE_SURFELMAP_H_ */
