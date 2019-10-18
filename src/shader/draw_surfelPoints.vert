#version 330 core

/**
 *  \author behley
 *
 *  \author Xieyuanli Chen
 **/

layout (location = 0) in vec4 position_radius;
layout (location = 1) in vec4 normal_confidence;
layout (location = 2) in int surfel_timestamp;
layout (location = 3) in vec3 surfel_color_weight_count;
layout (location = 4) in vec4 sfl_semantic_map;

uniform mat4 mvp;
uniform mat4 mvp_inv_t;

uniform int colorMode; // 0 - phong, 1 - normal shading, 2 - normal color, 3 - confidence
uniform float conf_threshold;

uniform vec3 view_pos;
uniform bool drawCurrentSurfelsOnly;
uniform int timestamp;
uniform bool backface_culling;

uniform samplerBuffer poseBuffer;
uniform sampler1D color_map;

out vec4 color;

#include "shader/color.glsl"

const vec4 rviridis = vec4(2.90912735, -2.14404531, 0.04439198,  0.29390206);
const vec4 gviridis = vec4(-0.17293242, -0.16906214,  1.24131122,  0.01871256);
const vec4 bviridis = vec4(0.17848859, -1.72405244,  1.23042564,  0.34479632);

// approximate version of viridis colormap.
vec3 viridis(float t)
{
  vec4 tt = vec4(t*t*t, t*t, t, 1.0);
  return vec3(dot(tt, rviridis), dot(tt, gviridis), dot(tt, bviridis));
}

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
  gl_Position = mvp * surfelPose * vec4(position_radius.xyz, 1.0f);
  vec4 n = surfelPose * vec4(normal_confidence.xyz, 0.0f);
  float c = normal_confidence.w;

  vec4 p =  vec4(position_radius.xyz, 1.0f);
  vec3 unpacked_color = unpack(surfel_color_weight_count.x);

  bool valid = (c > conf_threshold);
  vec3 view_dir = normalize((view_pos.xyz - p.xyz));

  if(colorMode == 1)
  {
   color = vec4((vec3(.5f, .5f, .5f) * abs(dot(n.xyz, vec3(1.0, 1.0, 1.0)))) + vec3(0.1f, 0.1f, 0.1f), 1.0f);
  }
  else if(colorMode == 2)
  {
    color = vec4(abs(n.xyz), 1.0f);
  }
  else if(colorMode == 3)
  {
    valid = true;
    color = vec4(viridis(1.0-1.0/(1.0 + exp(c))), 1.0);
  }
  else if(colorMode == 5)
  {
    vec4 semantic_label = vec4(texture(color_map, sfl_semantic_map.x * 255.0f / 259.0f).rgb, 1.0);
    if (sfl_semantic_map.x != 0) color = semantic_label;
    else valid = false;
  }
  else
  {
    vec3 surfel_color = vec3(0.8f, 0.75f, 0.65f);
    float alpha = 1.0f;

    if (colorMode == 4)
    {
      surfel_color = unpacked_color;
      valid = true;
      alpha =  1.0 - clamp(conf_threshold - c, 0.1, 1);
    }

    color = vec4(surfel_color, alpha);
  }

  valid = valid && (!backface_culling || (dot(view_dir, n.xyz) > 0));

  if(drawCurrentSurfelsOnly) valid = (timestamp == surfel_timestamp);

  if(!valid) gl_Position = vec4(-10, -10, -10, 1.0f);
}
