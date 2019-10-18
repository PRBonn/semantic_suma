#include "Model.h"

using namespace glow;

void Model::attach(const Mesh& mesh) {
  model_matrices_.push_back(Eigen::Matrix4f::Identity());
  normal_matrices_.push_back(Eigen::Matrix4f::Identity());
  meshes_.push_back(mesh);
}

void Model::attach(const Eigen::Matrix4f& transform, const Mesh& mesh) {
  model_matrices_.push_back(transform);
  normal_matrices_.push_back(transform.inverse().transpose());
  meshes_.push_back(mesh);
}

void Model::draw(glow::GlProgram& program, const MatrixStack& matrices) const {
  GlUniform<Eigen::Matrix4f> mvp("mvp", matrices.mvp);
  GlUniform<Eigen::Matrix4f> model_mat("model_mat", matrices.model);
  GlUniform<Eigen::Matrix4f> normal_mat("normal_mat", matrices.normal);

  //  glEnable(GL_BLEND);
  //  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  for (uint32_t i = 0; i < meshes_.size(); ++i) {
    // apply local transformations to matrix stack.
    mvp = matrices.mvp * model_matrices_[i];  // mvp = p * v * m * l
    model_mat = matrices.model * model_matrices_[i];
    normal_mat = matrices.normal * normal_matrices_[i];

    program.setUniform(mvp);
    program.setUniform(model_mat);
    program.setUniform(normal_mat);

    meshes_[i].draw(program);
  }

  //  glDisable(GL_BLEND);
}
