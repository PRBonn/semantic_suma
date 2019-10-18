#version 330 core

/**
 *  \author behley
 *
 *  \author Xieyuanli Chen
 **/

layout(points) in;
layout(points, max_vertices = 1) out;

in SURFEL
{
  bool valid;
  vec4 position_radius;
  vec4 normal_confidence;
  int timestamp;
  vec3 color_weight_count;
  vec4 semantic_map;
} gs_in[];

out vec4 sfl_position_radius;
out vec4 sfl_normal_confidence;
out int sfl_timestamp;
out vec3 sfl_color_weight_count;
out vec4 sfl_semantic_map;

void main()
{
  if(gs_in[0].valid)
  {
    sfl_position_radius = gs_in[0].position_radius;
    sfl_normal_confidence = gs_in[0].normal_confidence;
    sfl_timestamp = gs_in[0].timestamp;
    sfl_color_weight_count = gs_in[0].color_weight_count;
    sfl_semantic_map = gs_in[0].semantic_map;

    EmitVertex();
    EndPrimitive();
  }
}
