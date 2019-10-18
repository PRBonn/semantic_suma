#version 330 core

/**
 *  \author behley
 *
 *  \author Xieyuanli Chen
 **/

// \brief essentially the same as drawing, but with projected coordinates.

// Replicate surfels on borders?

layout (location = 0) in vec4 position_radius;
layout (location = 1) in vec4 normal_confidence;
layout (location = 2) in int timestamp;
layout (location = 3) in vec3 surfel_color_weight_count;
layout (location = 4) in vec4 surfel_semantic_map;

uniform mat4 inv_pose;

uniform samplerBuffer poseBuffer;

out SURFEL
{
  vec4 position;
  float radius;
  vec4 normal;
  float confidence;
  int index;
  int timestamp;
  int creation_timestamp;
  vec4 semantic_map;
} vs_out;

mat4 get_pose(int t)
{
  int offset = 4 * t;
  return mat4(texelFetch(poseBuffer, offset), texelFetch(poseBuffer, offset + 1),
              texelFetch(poseBuffer, offset + 2), texelFetch(poseBuffer, offset+3));
}

void main()
{
  mat4 surfelPose = get_pose(int(surfel_color_weight_count.z));

  vs_out.position = inv_pose * surfelPose * vec4(position_radius.xyz, 1.0f);
  vs_out.radius = position_radius.w;
  vs_out.normal = inv_pose * surfelPose * vec4(normal_confidence.xyz, 0.0f);
  vs_out.confidence = normal_confidence.w;
  vs_out.index = gl_VertexID;
  vs_out.timestamp = timestamp;
  vs_out.creation_timestamp = int(surfel_color_weight_count.z);
  vs_out.semantic_map = surfel_semantic_map;
}
