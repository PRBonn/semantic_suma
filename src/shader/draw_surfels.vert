#version 330 core

/**
 *  \author behley
 *
 *  \author Xieyuanli Chen
 **/

layout (location = 0) in vec4 position_radius;
layout (location = 1) in vec4 normal_confidence;
layout (location = 2) in int timestamp;
layout (location = 3) in vec3 surfel_color_weight_count;
layout (location = 4) in vec4 surfel_semantic_map;

uniform mat4 mvp;
uniform mat4 mvp_inv_t;

uniform samplerBuffer poseBuffer;

out SURFEL
{
  float radius;
  vec4 normal;
  float confidence;
  int timestamp;
  vec3 color;
  vec4 semantic_map;
} vs_out;

#include "shader/color.glsl"

mat4 get_pose(int timestamp)
{
  int offset = 4 * timestamp;
  return mat4(texelFetch(poseBuffer, offset), texelFetch(poseBuffer, offset + 1),
              texelFetch(poseBuffer, offset + 2), texelFetch(poseBuffer, offset+3));
}

void main()
{
  mat4 surfelPose = get_pose(int(surfel_color_weight_count.z));

  // projection etc is applied in geometry shader.
  gl_Position = surfelPose * vec4(position_radius.xyz, 1.0f);
  vs_out.radius = position_radius.w;
  vs_out.normal = surfelPose * vec4(normal_confidence.xyz, 0.0f);
  vs_out.confidence = normal_confidence.w;
  vs_out.timestamp = timestamp;
  vs_out.color = unpack(surfel_color_weight_count.x);
  vs_out.semantic_map = surfel_semantic_map;
}
