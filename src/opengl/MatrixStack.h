/*
 * MatrixStack.h
 *
 *  Created on: Jun 28, 2016
 *      Author: behley
 */

#ifndef INCLUDE_CORE_MATRIXSTACK_H_
#define INCLUDE_CORE_MATRIXSTACK_H_

#include <eigen3/Eigen/Dense>

/** \brief All matrices needed for correct rendering and shading.
 *
 *  For correct (phong) shading, we have to determine the angles between model
 *  normals and the light directions. Therefore, we have to first transform all
 *  model vertices in world coordinates and also transform the normals. Thus, it
 *  is handy to not only have the model-view-projection matrix, but also the model
 *  matrix and the normal matrix to apply local transformations.
 *
 *  \author behley
 */
struct MatrixStack {
 public:
  Eigen::Matrix4f mvp;     // model-view-projection matrix.
  Eigen::Matrix4f model;   // model matrix for transformation in world coordinates
  Eigen::Matrix4f normal;  // normal matrix for transformation in world coordinates.
};

#endif /* INCLUDE_CORE_MATRIXSTACK_H_ */
