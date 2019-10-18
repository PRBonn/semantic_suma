#ifndef INCLUDE_CORE_MODEL_H_
#define INCLUDE_CORE_MODEL_H_

#include "Mesh.h"
#include "MatrixStack.h"

/** \brief a model represents a collection of meshes.
 *
 *  \author behley
 */
class Model {
 public:
  /** \brief attach a mesh to the model. **/
  void attach(const Mesh& mesh);

  /** \brief attach a mesh to the model with given local transformation. **/
  void attach(const Eigen::Matrix4f& transform, const Mesh& mesh);

  /** \brief draw meshes with given program... **/
  void draw(glow::GlProgram& program, const MatrixStack& matrices) const;

 protected:
  std::vector<Eigen::Matrix4f> model_matrices_;
  std::vector<Eigen::Matrix4f> normal_matrices_;

  std::vector<Mesh> meshes_;
};

#endif /* INCLUDE_CORE_MODEL_H_ */
