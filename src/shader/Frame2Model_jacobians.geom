#version 330 core

/**
 *  \author behley
 *
 *  \author Xieyuanli Chen
 **/

layout(points) in;
layout(points, max_vertices = 16) out;

uniform sampler2DRect vertex_model;
uniform sampler2DRect normal_model;

uniform sampler2DRect vertex_data;
uniform sampler2DRect normal_data;

uniform sampler2DRect semantic_model; // [0,width]
uniform sampler2DRect semantic_data;

uniform float distance_thresh;
uniform float angle_thresh;

uniform mat4 pose;
uniform float fov_up;
uniform float fov_down;
uniform int entries_per_kernel;

const float pi = 3.14159265358979323846f;
const float inv_pi = 0.31830988618379067154f;
const float pi_2 = 1.57079632679;

uniform int iteration;
uniform int weight_function; // 0 - none, 1 - huber, 2 - turkey, 3 - stability.
uniform float factor;

uniform float cutoff_threshold;

#include "shader/color_map.glsl"

in VS_OUT {
  vec2 texCoords;
} gs_in[];

out vec3 values;

vec2 mat_coords(int i, int j)
{
  // keep in mind that we only have OpenGLs coord system...
  return vec2(2.0 * (j + 0.5) / 2.0 - 1.0, 2.0 * (i + 0.5) / 8.0 - 1.0);
}

vec2 project2model(vec4 vertex)
{
  vec2 tex_dim = textureSize(vertex_model);
  float fov = abs(fov_up) + abs(fov_down);
  float depth = length(vertex.xyz);
  float yaw = atan(vertex.y, vertex.x);
  float pitch = -asin(vertex.z / depth); // angle = acos((0,0,1) * p/||p||) - pi/2 = pi/2 - asin(x) + pi/2

  float x = 0.5 * ((-yaw * inv_pi) + 1.0); // in [0, 1]
  float y = 1.0 - (degrees(pitch) + fov_up) / fov; // in [0, 1]

  return vec2(x, y)*tex_dim; // [0, w],  [0, h]
}

void main()
{
  vec2 tex_dim = textureSize(vertex_data);
  vec2 model_dim = textureSize(vertex_model);

  bool has_inlier = false;

  // Idea: compute aggregated values, then submit values.

  // store Jacobian in column-major order; store J^T*f in last row.

  vec3 temp[16];
  for(int i = 0; i < 16; ++i) temp[i] = vec3(0);

  for(int e = 0; e < entries_per_kernel; ++e)
  {
    vec2 texCoords = gs_in[0].texCoords + vec2(e, 0);
    if(texCoords.x >= tex_dim.x || texCoords.y >= tex_dim.y) continue;

    vec2 img_coords = texCoords.xy / tex_dim;

    float e_d = texture(vertex_data, texCoords).w + texture(normal_data, texCoords).w;
    vec4 v_d = pose * vec4(texture(vertex_data, texCoords).xyz, 1.0);
    vec4 n_d = pose * vec4(texture(normal_data, texCoords).xyz, 0.0); // assuming non-scaling transform

    bool inlier = false;

    vec2 idx = project2model(v_d);
    if(idx.x < 0 || idx.x >= model_dim.x || idx.y < 0 || idx.y >= model_dim.y)
    {
      e_d = 0.0f;
    }

    float e_m = texture(vertex_model, idx).w + texture(normal_model, idx).w;
    vec3 v_m = texture(vertex_model, idx).xyz;
    vec3 n_m = texture(normal_model, idx).xyz;

    // Reminder: use ifs instead of if(...) return; to avoid problemns with Intel cpu.
    if((e_m > 1.5f) && (e_d > 1.5f))
    {
      has_inlier = true;

      bool inlier = true;

      if(length(v_m.xyz - v_d.xyz) > distance_thresh) inlier = false;
      if(dot(n_m.xyz, n_d.xyz) < angle_thresh) inlier = false;

      float residual = (dot(n_m.xyz, (v_d.xyz - v_m.xyz)));
      vec3 n = n_m;
      vec3 cp = cross(v_d.xyz, n_m.xyz);

      float weight = 1.0;

      if((weight_function == 4 || weight_function == 1))
      {
        // huber weighting.
        if(abs(residual) > factor)
        {
          weight = factor / abs(residual);
        }
      }
      else if(weight_function == 2 && iteration > 0)
      {
        // turkey bi-squared weighting:
        if(abs(residual) > factor)
        {
          weight = 0;
        }
        else
        {
          float alpha = residual / factor;
          weight = (1.0  - alpha * alpha);
          weight = weight * weight;
        }
      }

      // use semantic information during ICP
      float data_label = texture(semantic_data, texCoords).x * 255.0;
      float data_label_prob = texture(semantic_data, texCoords).w;
      float model_label = texture(semantic_model, idx).x * 255.0;

      if( model_label == car.w || model_label == bicycle.w ||
          model_label == bus.w || model_label == motorcycle.w||
          model_label == truck.w|| model_label == other_vehicle.w||
          model_label == person.w||
          model_label == bicyclist.w || model_label == motorcyclist.w)
      {
        if(round(data_label) != round(model_label))
           weight *= (1 - data_label_prob);
        else
           weight *= data_label_prob;
      }

      if(inlier)
      {

        temp[0] += weight * n.x * n;
        temp[1] += weight * n.y * n;
        temp[2] += weight * n.z * n;
        temp[3] += weight * cp.x * n;
        temp[4] += weight * cp.y * n;
        temp[5] += weight * cp.z * n;

        temp[6] += weight * n.x * cp;
        temp[7] += weight * n.y * cp;
        temp[8] += weight * n.z * cp;
        temp[9] += weight * cp.x * cp;
        temp[10] += weight * cp.y * cp;
        temp[11] += weight * cp.z * cp;

        // compute J^T * W * f => 2 vertices

        temp[12] += weight * residual * n;

        temp[13] += weight * residual * cp;

        temp[14].x += 1.0f; // terms in error function (inlier + outlier)
        temp[14].y += weight * residual * residual; // residual

        temp[15].x += weight * residual * residual; // inlier residual

      }
      else
      {
        // was cut-off due to gross outlier rejection.
        temp[14].x += 1.0f; // terms.
        temp[14].y += weight * residual * residual; // "outlier" residual.
        temp[14].z += 1.0f;// num_outliers.
      }
    }
    else
    {
      temp[15].y += 1.0; // invalid count.
    }

  }

  // SUBMISSION:

  if(has_inlier)
  {
    for(int i = 0; i < 6; ++i)
    {
      gl_Position = vec4(mat_coords(i, 0), 0.0, 1.0);
      values = temp[i];

      EmitVertex();
      EndPrimitive();

      gl_Position = vec4(mat_coords(i, 1), 0.0, 1.0);
      values = temp[i + 6];

      EmitVertex();
      EndPrimitive();
    }
  }

  gl_Position = vec4(mat_coords(6, 0), 0.0, 1.0);
  values = temp[12];

  EmitVertex();
  EndPrimitive();

  gl_Position = vec4(mat_coords(6, 1), 0.0, 1.0);
  values = temp[13];

  EmitVertex();
  EndPrimitive();

  gl_Position = vec4(mat_coords(7, 0), 0.0, 1.0);
  values = temp[14];

  EmitVertex();
  EndPrimitive();

  gl_Position = vec4(mat_coords(7, 1), 0.0, 1.0);
  values = temp[15];

  EmitVertex();
  EndPrimitive();
}
