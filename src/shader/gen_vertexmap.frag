#version 330 core

/**
 *  \author behley
 *
 *  \author Xieyuanli Chen
 **/

in vec4 vertex_coord;
in float vert_label;
in float vert_label_prob;

layout (location = 0) out vec4 color;
layout (location = 1) out vec4 semantic_map;

void main()
{
  color = vertex_coord;

  float vert_label_ = vert_label / 255.0f;

  semantic_map = vec4(vert_label_, vert_label_, vert_label_, vert_label_prob);
}
