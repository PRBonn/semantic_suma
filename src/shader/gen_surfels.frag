#version 330 core

/**
 *  \author behley
 *
 *  \author Xieyuanli Chen
 **/

layout (location = 0) out vec4 upd_vertex_radius;
layout (location = 1) out vec4 upd_normal_confidence;

in vec4 sfl_position_radius;
in vec4 sfl_normal_confidence;
in int sfl_timestamp;

void main()
{
  // store information for update.
  upd_vertex_radius = sfl_position_radius;
  upd_normal_confidence = sfl_normal_confidence;
}
