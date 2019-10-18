#version 330

/**
 *  \author behley
 **/

layout (location = 0) in vec4 position_in;
layout (location = 1) in vec4 normal_in;

uniform mat4 model_mat;   // model matrix.
uniform mat4 normal_mat;  // transformation of normals to world coordinates.
uniform mat4 mvp;         // model-view-projection matrix. (applied p*v*m)

out vec4 pos;
out vec4 normal;

void main()
{
  gl_Position = mvp * position_in;
  pos = model_mat * position_in;
  normal = normal_mat * normal_in;
}
