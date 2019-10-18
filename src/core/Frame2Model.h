#ifndef INCLUDE_CORE_FRAME2MODEL2_H_
#define INCLUDE_CORE_FRAME2MODEL2_H_

#include <glow/GlFramebuffer.h>
#include <glow/GlProgram.h>
#include <glow/GlQuery.h>
#include <glow/GlSampler.h>
#include <glow/GlVertexArray.h>
#include <glow/glutil.h>
#include <rv/ParameterList.h>

#include "Frame.h"
#include "core/Objective.h"

/** \brief Project point cloud, use projections to associate with model vertex, normal map.
 *
 *  This is essentially ProjectedCloud to frame, but with computation of weights for
 *  Iterative Re-weighted (Non-linear) Least Squares enabling to use different M-Estimators, like
 *  Huber weights and Turkey Bi-square weights.
 *
 *  This variant transforms the point cloud and tries to match this to the model frame rendered
 *  at the last position.
 *
 *  \author behley
 *
 *  \author Xieyuanli Chen
 */
class Frame2Model : public Objective {
 public:
  /** \brief setup objective. **/
  Frame2Model(const rv::ParameterList& params);

  /** \brief set single parameter to specific value. **/
  void setParameter(const rv::Parameter& param) override;

  void setData(const std::shared_ptr<Frame>& current, const std::shared_ptr<Frame>& last);

  /** \brief specify at which level the redisual/jacobian is computed. **/
  void setLevel(uint32_t lvl) override;

  /** \brief get maximum level (coarse-to-fine). **/
  uint32_t getMaxLevel() const override;

  uint32_t num_parameters() const;

  /** \brief compute residual for given increment. **/
  double residual(const Eigen::VectorXd& delta);

  /** \brief compute weighted JtJ and Jtf exploiting intermediate computations, return weighted objective F(x). */
  double jacobianProducts(Eigen::MatrixXd& JtJ, Eigen::MatrixXd& Jtf);

 protected:
  /** \brief updatable parameters, i.e., params that can be changed at runtime. **/
  void updateParameters();

  rv::ParameterList params_;

  std::shared_ptr<Frame> current_;
  std::shared_ptr<Frame> last_;  // model frame.

  uint32_t num_residuals_{0};

  glow::GlSampler sampler_;

  glow::GlVertexArray vao_img_coords_;
  glow::GlBuffer<glow::vec2> vbo_img_coords_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::STATIC_DRAW};

  glow::GlTexture JtJJtf_blend_;
  glow::GlFramebuffer fbo_blend_;
  glow::GlProgram program_blend_;

  uint32_t entriesPerKernel_{64};
};

#endif /* INCLUDE_CORE_FRAME2MODEL_H_ */
