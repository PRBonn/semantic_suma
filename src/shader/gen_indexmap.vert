#version 330 core

/**
 *  \author behley
 *
 *  \author Xieyuanli Chen
 **/

layout (location = 0) in vec4 surfel_position_radius;
layout (location = 1) in vec4 surfel_normal_confidence;
layout (location = 2) in int  surfel_timestamp;
layout (location = 3) in vec3 surfel_color_weight_count;
layout (location = 4) in vec4 sfl_semantic_map;

flat out float index;
out vec4 vertex;
out vec4 normal;

const vec2 halfpix = vec2(0.5f, 0.5f);
const float sqrt2 = 1.41421356237f;
const float pi = 3.14159265358979323846f;
const float inv_pi = 0.31830988618379067154f;
const float pi_2 = 1.57079632679;

uniform mat4 pose;
uniform mat4 inv_pose;

uniform float fov_up;
uniform float fov_down;
uniform float max_depth;
uniform float min_depth;
uniform float width;
uniform float height;

uniform samplerBuffer poseBuffer;

vec3 projectSpherical(vec4 position)
{
  float fov = abs(fov_up) + abs(fov_down);
  float depth = length(position.xyz);
  float yaw = atan(position.y, position.x);
  float pitch = -asin(position.z / depth);

  float x = 0.5 * ((-yaw * inv_pi) + 1.0); // in [0, 1]
  float y = (1.0 - (degrees(pitch) + fov_up) / fov); // in [0, 1]
  float z = (depth - min_depth) / (max_depth - min_depth); // in [0, 1]

  x = (floor(x * width) + 0.5) / width;
  y = (floor(y * height) + 0.5) / height;

  return vec3(x, y, z);
}

mat4 get_pose(int timestamp)
{
  int offset = 4 * timestamp;
  return mat4(texelFetch(poseBuffer, offset), texelFetch(poseBuffer, offset + 1),
              texelFetch(poseBuffer, offset + 2), texelFetch(poseBuffer, offset+3));
}

/** \brief depth-buffered index map.  **/
void main()
{

  mat4 surfelPose = get_pose(int(surfel_color_weight_count.z));

  vertex = inv_pose * surfelPose * vec4(surfel_position_radius.xyz, 1.0);
  normal = inv_pose * surfelPose * vec4(surfel_normal_confidence.xyz, 0.0);

  vec4 p = vertex;
  vec4 n = normal;

  gl_Position = vec4(-10.0);

  // visible from current position.
  if(dot(n.xyz, -p.xyz/length(p.xyz)) > 0.01)
  {
    gl_Position = vec4(2.0 * projectSpherical(p) - vec3(1.0), 1.0);
    index = gl_VertexID + 1; // initially all index pixels are 0.
  }
}
