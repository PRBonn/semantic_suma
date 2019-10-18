#version 330 core

/**
 *  \author behley
 **/

layout (location = 0) in vec2 img_coords; // image coords = (x, y) in [0, w] x [0, h]

uniform sampler2DRect vertex_map;
uniform sampler2DRect normal_map;

uniform float fov_up;
uniform float fov_down;
uniform float max_depth;
uniform float min_depth;
uniform float pixel_size;
uniform int confidence_mode;
uniform float min_radius;
uniform float max_radius;
uniform float angle_thresh;


const vec2 halfpix = vec2(0.5f, 0.5f);
const float sqrt2 = 1.41421356237f;
const float pi = 3.14159265358979323846f;
const float inv_pi = 0.31830988618379067154f;
const float pi_2 = 1.57079632679;

out float valid;
out vec4 centerized_vertex;
out float radius;
out float confidence;

float get_radius(vec4 vertex, vec4 normal, float pixel_size)
{
  float d = length(vertex.xyz);

  return  1.41 * d * pixel_size / clamp(dot(normal.xyz, -vertex.xyz / d), 0.5, 1.0);
}

void main()
{
  vec2 tex_dim = textureSize(vertex_map);
  
  gl_Position = vec4(2.0 * img_coords / tex_dim - vec2(1.0), 0, 1);
  valid = 0.0;
  radius = 0.0;
  confidence = 0.0;
  
  vec4 vertex = texture(vertex_map, img_coords);
  vec4 normal = texture(normal_map, img_coords);
  vec3 view_dir = -vertex.xyz / length(vertex.xyz);
  
  float angle = dot(normal.xyz, view_dir);
  
  if(vertex.w > 0.5 && normal.w > 0.5 && angle > angle_thresh)
  {   
    valid = 1.0;
   
    float angle = 1.0 - clamp(dot(normal.xyz, view_dir), 0, 1);
    
    float confidence = 1.0f;

    centerized_vertex = vertex;
    radius = get_radius(vertex, normal, pixel_size);
    radius = min(max(radius, min_radius), max_radius);
  }    
}
