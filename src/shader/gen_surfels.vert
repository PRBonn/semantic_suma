#version 330 core

/**
 *  \author behley
 *
 *  \author Xieyuanli Chen
 **/

#include "shader/color.glsl"

layout (location = 0) in vec2 img_coords; // image coords = (x, y) in [0, w] x [0, h]

// centerized vertex map, normal map and precomputed radius/confidence map.
uniform sampler2DRect vertex_map;
uniform sampler2DRect normal_map;
uniform sampler2DRect measurementIntegrated_map;
uniform sampler2DRect radiusConfidence_map;

uniform int timestamp;
uniform mat4 pose; // current pose of laser scanner.
uniform float log_prior;

uniform float width;
uniform float height;
uniform float pixel_size;

uniform float fov_up;
uniform float fov_down;
uniform float max_depth;
uniform float min_depth;


out SURFEL {
  bool valid;
  vec2 img_coords;
} vs_out;

void main()
{
  vec3 vertex = texture(vertex_map, img_coords).xyz;
  vec3 normal = texture(normal_map, img_coords).xyz;

  bool invalid = (texture(vertex_map, img_coords).w < 1.0f) || (texture(normal_map, img_coords).w < 1.0f);
  invalid = invalid || (texture(radiusConfidence_map, img_coords).w < 0.5f);
  bool integrated = texture(measurementIntegrated_map, img_coords).x > 0.5;

  vec3 view_dir = -vertex / length(vertex);

  // vertex/normal invalid or already integrated or normal pointing in the wrong direction.
  vs_out.valid = (!invalid && !integrated && (dot(normal, view_dir) > 0.01));
  vs_out.img_coords = img_coords;
}
