#version 330 core

/**
 *  \author behley
 *
 *  \author Xieyuanli Chen
 **/

layout(points) in;
layout(triangle_strip, max_vertices = 6) out;

in SURFEL
{
  vec4 position;
  float radius;
  vec4 normal;
  float confidence;
  int index;
  int timestamp;
  int creation_timestamp;
  vec4 semantic_map;
} gs_in[];

out vec2 texCoords;
out vec4 normal;
out vec4 vertex;
out vec4 semantic;
out float confidence;

uniform float fov_up;
uniform float fov_down;
uniform float max_depth;
uniform float min_depth;

uniform float conf_threshold;
uniform int timestamp_threshold;
uniform bool render_old_surfels;
uniform bool use_stability;

const float pi = 3.14159265358979323846f;
const float inv_pi = 0.31830988618379067154f;
const float pi_2 = 1.57079632679;


vec3 project2model(vec4 position)
{
  float fov = abs(fov_up) + abs(fov_down);
  float depth = length(position.xyz);
  float yaw = atan(position.y, position.x);
  float pitch = -asin(position.z / depth);

  float x = 0.5 * ((-yaw * inv_pi) + 1.0); // in [0, 1]
  float y = (1.0 - (degrees(pitch) + fov_up) / fov); // in [0, 1]
  float z = (depth - min_depth) / (max_depth - min_depth); // in [0, 1]

  return vec3(x, y, z);
}

vec3 project2model(vec4 position, vec3 center_pixel)
{
  float fov = abs(fov_up) + abs(fov_down);
  float depth = length(position.xyz);
  float yaw = atan(position.y, position.x);
  float pitch = -asin(position.z / depth);

  float x = 0.5 * ((-yaw * inv_pi) + 1.0); // in [0, 1]
  // FIXME: find better solution for wrap around:
  if(center_pixel.x - x > 0.5) x += 1.0;
  if(x - center_pixel.x > 0.5) x -= 1.0;
  float y = (1.0 - (degrees(pitch) + fov_up) / fov); // in [0, 1]
  float z = (depth - min_depth) / (max_depth - min_depth); // in [0, 1]

  return vec3(x, y, z);
}

void main()
{
  vec4 p = gs_in[0].position;
  vec4 n = gs_in[0].normal;
  vec4 u = normalize(vec4(n.y - n.z, -n.x, n.x, 0.0f));
  vec4 v = vec4(normalize(cross(n.xyz, u.xyz)), 0.0f);
  float r = gs_in[0].radius;

  bool visible = (dot(n.xyz, -p.xyz / length(p.xyz)) > 0.01);

  vec3 pp = project2model(p);
  if(visible && all(greaterThanEqual(pp, vec3(0))) &&  all(lessThan(pp, vec3(1))) && (!use_stability || gs_in[0].confidence > conf_threshold))
  {
    //p_stable = 1.0 - 1.0 / (1.0 + exp(gs_in[0].confidence));
    bool valid = (render_old_surfels && (gs_in[0].creation_timestamp <  timestamp_threshold));
    valid = valid || (!render_old_surfels && (gs_in[0].creation_timestamp >=  timestamp_threshold || gs_in[0].timestamp >= timestamp_threshold));

    if(valid)
    {
      vertex = vec4(p.xyz, 1.0f);
      normal = vec4(n.xyz, 1.0f);
      semantic = gs_in[0].semantic_map;
      //index = gs_in[0].index + 1;
      confidence = gs_in[0].confidence;

      vec4 ru = r * u;
      vec4 rv = r * v;

      gl_Position = vec4(2.0 * project2model(p - ru - rv, pp) - 1.0, 1.0f);
      texCoords = vec2(-1.0, -1.0);
      EmitVertex();

      gl_Position = vec4(2.0 * project2model(p + ru - rv, pp) - 1.0, 1.0f);
      texCoords = vec2(1.0, -1.0);
      EmitVertex();

      gl_Position = vec4(2.0 * project2model(p - ru + rv, pp) - 1.0, 1.0f);
      texCoords = vec2(-1.0, 1.0);
      EmitVertex();

      gl_Position = vec4(2.0 * project2model(p + ru + rv, pp) - 1.0, 1.0f);
      texCoords = vec2(1.0, 1.0);

      EmitVertex();
      EndPrimitive();
    }
  }
}
