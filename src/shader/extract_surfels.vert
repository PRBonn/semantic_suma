#version 330 core

/**
 *  \author behley
 *
 *  \author Xieyuanli Chen
 **/

layout (location = 0) in vec4 position_radius;
layout (location = 1) in vec4 normal_confidence;
layout (location = 2) in int in_timestamp;
layout (location = 3) in vec3 surfel_color_weight_count;
layout (location = 4) in vec4 sfl_semantic_map;

uniform samplerBuffer poseBuffer;
uniform vec2 submap_center;
uniform float submap_extent;

out SURFEL
{
  bool valid;
  vec4 position_radius;
  vec4 normal_confidence;
  int timestamp;
  vec3 color_weight_count;
  vec4 semantic_map;
} vs_out;

mat4 get_pose(int timestamp)
{
  int offset = 4 * timestamp;
  return mat4(texelFetch(poseBuffer, offset), texelFetch(poseBuffer, offset + 1),
              texelFetch(poseBuffer, offset + 2), texelFetch(poseBuffer, offset+3));
}

mat4 inverse_pose(mat4 pose)
{
  mat3 R = transpose(mat3(pose));
  vec3 t = vec3(pose[3]);
  mat4 result = mat4(R);
  result[3] = vec4(-R*t, 1);

  return result;
}

void main()
{
  mat4 surfelPose = get_pose(int(surfel_color_weight_count.z));
  vec4 position = surfelPose * vec4(position_radius.xyz, 1.0);

  if(abs(position.x - submap_center.x) > submap_extent || abs(position.y - submap_center.y) > submap_extent)
  {
    vs_out.valid = false;
  }
  else
  {
    vs_out.valid = true;
    vs_out.position_radius = position_radius;
    vs_out.normal_confidence = normal_confidence;
    vs_out.timestamp = in_timestamp;
    vs_out.color_weight_count = surfel_color_weight_count;
    vs_out.semantic_map = sfl_semantic_map;
  }
}
