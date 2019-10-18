#version 400 core

/**
 *  \author behley
 *
 *  \author Xieyuanli Chen
 **/

#include "shader/color.glsl"

layout(points) in;
layout(points, max_vertices = 1) out;

uniform int timestamp;

in SURFEL {
  bool valid;
  vec2 img_coords;
} gs_in[];

out vec4 sfl_position_radius;
out vec4 sfl_normal_confidence;
out int sfl_timestamp;
out vec3  sfl_color_weight_count;
out vec4  sfl_semantic_map;

uniform sampler2DRect vertex_map;
uniform sampler2DRect normal_map;
uniform sampler2DRect radiusConfidence_map;

uniform sampler2DRect semantic_map;
uniform sampler2DRect model_semantic_map;
// uniform sampler1D prior_map;

uniform float removed_class;

uniform mat4 pose; // current pose of laser scanner.
uniform mat4 inv_pose; // current pose of laser scanner.
uniform float log_prior;

uniform float width;
uniform float height;
uniform float pixel_size;

uniform float fov_up;
uniform float fov_down;
uniform float max_depth;
uniform float min_depth;

const float sqrt2 = 1.41421356237f;
const float pi = 3.14159265358979323846f;
const float inv_pi = 0.31830988618379067154f;
const float pi_2 = 1.57079632679;

// semantic color_map
  vec4 unlabeled = vec4(0, 0, 0, 0);
  vec4 outlier = vec4(0, 0, 255, 1);
  vec4 car = vec4(245, 150, 100, 10);
  vec4 bicycle = vec4(245, 230, 100, 11);
  vec4 bus = vec4(250, 80, 100, 13);
  vec4 motorcycle = vec4(150, 60, 30,15);
  vec4 on_rails = vec4(255, 0, 0, 16);
  vec4 truck = vec4(180, 30, 80, 18);
  vec4 other_vehicle = vec4(255, 0, 0, 20);
  vec4 person = vec4(30, 30, 255, 30);
  vec4 bicyclist = vec4(200, 40, 255, 31);
  vec4 motorcyclist = vec4(90, 30, 150, 32);
  vec4 road = vec4(255, 0, 255, 40);
  vec4 parking = vec4(255, 150, 255, 44);
  vec4 sidewalk = vec4(75, 0, 75, 48);
  vec4 other_ground = vec4(75, 0, 175, 49);
  vec4 building = vec4(0, 200, 255, 50);
  vec4 fence = vec4(50, 120, 255, 51);
  vec4 other_structure = vec4(0, 150, 255, 52);
  vec4 lane_marking = vec4(170, 255, 150, 60);
  vec4 vegetation = vec4(0, 175, 0, 70);
  vec4 trunk = vec4(0, 60, 135, 71);
  vec4 terrain = vec4(80, 240, 150, 72);
  vec4 pole = vec4(150, 240, 255, 80);
  vec4 traffic_sign = vec4(0, 0, 255, 81);
  vec4 other_object = vec4(255, 255, 50, 99);
  vec4 moving_car = vec4(245, 150, 100, 252);
  vec4 moving_bicyclist = vec4(255, 0, 0, 256);
  vec4 moving_person = vec4(200, 40, 255, 253);
  vec4 moving_motorcyclist = vec4(30, 30, 255, 254);
  vec4 moving_on_rails = vec4(90, 30, 150, 255);
  vec4 moving_bus = vec4(250, 80, 100, 257);
  vec4 moving_truck = vec4(180, 30, 80, 258);
  vec4 moving_other_vehicle = vec4(255, 0, 0, 259);

vec3 projectSpherical(vec3 position)
{
  float fov = abs(fov_up) + abs(fov_down);
  float depth = length(position.xyz);
  float yaw = atan(position.y, position.x);
  float pitch = -asin(position.z / depth);

  float x = 0.5 * ((-yaw * inv_pi) + 1.0); // in [0, 1]
  float y = (1.0 - (degrees(pitch) + fov_up) / fov); // in [0, 1]
  float z = (depth - min_depth) / (max_depth - min_depth); // in [0, 1]

  // half-pixel coordinates.
  x = (floor(x * width) + 0.5);
  y = (floor(y * height) + 0.5);

  return vec3(x, y, z);
}

void main()
{
  if(gs_in[0].valid)
  {
    vec2 img_coords = gs_in[0].img_coords;

    vec4 v = vec4(texture(vertex_map, img_coords).xyz, 1.0);
    vec4 n = vec4(texture(normal_map, img_coords).xyz, 0.0);
    vec4 v_global = v;
    vec4 n_global = normalize(n);
    float radius = texture(radiusConfidence_map, img_coords).x;

    float weight = 1.0f;

    // all surfels are outputed to the transform feedback buffer.
    sfl_position_radius = vec4(v_global.xyz, radius);
    sfl_normal_confidence = vec4(n_global.xyz, log_prior);
    sfl_timestamp = timestamp;
    sfl_color_weight_count = vec3(pack(vec3(0,0,1)), weight, timestamp);

    // detect and remove outliers
    sfl_semantic_map = texture(semantic_map, gs_in[0].img_coords);
    float semantic_label = sfl_semantic_map.x * 255.0;
    float model_label = texture(model_semantic_map, img_coords.xy).x * 255.0;

    // only dealing with movable objects
    if(semantic_label == car.w || semantic_label == bicycle.w ||
      semantic_label == bus.w || semantic_label == motorcycle.w||
      semantic_label == truck.w|| semantic_label == other_vehicle.w||
      semantic_label == person.w|| semantic_label == bicyclist.w ||
      semantic_label == motorcyclist.w)
      sfl_normal_confidence = vec4(n_global.xyz, log_prior - 0.5);

    EmitVertex();
    EndPrimitive();
  }
}
