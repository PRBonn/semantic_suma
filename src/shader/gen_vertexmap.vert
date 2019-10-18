#version 330 core
#pragma glsl

/**
 *  \author behley
 *
 *  \author Xieyuanli Chen
 **/

layout (location = 0) in vec4 position; // x, y, z, 1
layout (location = 1) in float label; // x
layout (location = 2) in float prob; // x

out vec4 vertex_coord; // coordinate of the vertex producing this pixel.
out float vert_label;
out float vert_label_prob;

const float pi = 3.14159265358979323846f;
const float inv_pi = 0.31830988618379067154f;
const float pi_2 = 1.57079632679;

uniform float width;
uniform float height;
uniform float fov_up;
uniform float fov_down;
uniform float min_depth;
uniform float max_depth;
uniform float removed_class;

uniform mat4 Tr;
uniform mat4 Tr_inv;
uniform mat4 P2;

const vec4 invalid = vec4(0.0, 0.0, 0.0, 0.0);
uniform bool isfirst;

// semantic color map (B, G, R, label)
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

void main()
{
  vert_label = label;
  vert_label_prob = prob;

  float fov = abs(fov_up) + abs(fov_down);
  float depth = length(position.xyz);
  float yaw = atan(position.y, position.x);
  float pitch = -asin(position.z / depth); // angle = acos((0,0,1) * p/||p||) - pi/2 = pi/2 - asin(x) + pi/2

  float x = (-yaw * inv_pi); // in [-1, 1]
  float y = (1.0 - 2.0 * (degrees(pitch) + fov_up) / fov); // in [-1, 1]
  float z = 2.0f * ((depth - min_depth) / (max_depth - min_depth)) - 1.0f; // in [-1, 1]

  // force that each point lies exactly inside the texel enabling reproducible results.
  x = 2.0f * ((floor(0.5f * (x + 1.0f) * width ) + 0.5f ) / width) - 1.0;
  y = 2.0f * ((floor(0.5f * (y + 1.0f) * height ) + 0.5f ) / height) - 1.0;

  gl_Position = vec4(x, y, z, 1.0);
  vertex_coord = vec4(position.xyz, 1.0);

  // remove all movable objects during initialization period
  if(isfirst){
    if(vert_label == car.w || vert_label == bicycle.w ||
      vert_label == bus.w || vert_label == motorcycle.w||
      vert_label == truck.w|| vert_label == other_vehicle.w||
      vert_label == person.w||
      vert_label == bicyclist.w || vert_label == motorcyclist.w)
      vertex_coord = invalid;
  }
}
