#version 330 core

/**
 *  \author behley
 *
 *  \author Xieyuanli Chen
 **/

#include "shader/color.glsl"
#include "shader/color_map.glsl"

layout (location = 0) in vec4 surfel_position_radius;
layout (location = 1) in vec4 surfel_normal_confidence;
layout (location = 2) in int  surfel_timestamp;
layout (location = 3) in vec3 surfel_color_weight_count;
layout (location = 4) in vec4 surfel_semantic_map;

uniform mat4 pose;
uniform mat4 inv_pose;
uniform int  timestamp;

uniform float distance_thresh;
uniform float angle_thresh;

uniform sampler2DRect vertex_map;
uniform sampler2DRect normal_map;
uniform sampler2DRect radiusConfidence_map;
uniform sampler2DRect index_map;
uniform sampler2DRect semantic_map_in;

uniform samplerBuffer poseBuffer;

uniform float width;
uniform float height;
uniform float pixel_size;

uniform float fov_up;
uniform float fov_down;
uniform float max_depth;
uniform float min_depth;
uniform int unstable_age;
uniform float confidence_threshold;
uniform int confidence_mode;

uniform float p_stable;
uniform float p_unstable;
uniform float p_prior;
uniform float log_unstable; // log odds of p_unstable log(p_unstable/(1-p_unstable));
uniform float log_prior;    // log odds of p_prior.
uniform float sigma_angle;
uniform float sigma_distance;
uniform float min_radius;
uniform bool update_always;
uniform bool use_stability;

uniform int weighting_scheme; // 0 - exponential, 1 - cumulative, 2 - cumulative/weighted
uniform float max_weight;
uniform int averaging_scheme; // 0 - normal, 1 - advanced/push?

const float sqrt2 = 1.41421356237f;
const float pi = 3.14159265358979323846f;
const float inv_pi = 0.31830988618379067154f;
const float pi_2 = 1.57079632679;

uniform int active_timestamps;

out SURFEL
{
  bool valid;
  vec4 position_radius;
  vec4 normal_confidence;
  int timestamp;
  vec3 color_weight_count;
  vec4 semantic_map; // for semantic map
} vs_out;

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

vec4 centerize(vec2 img_coords, vec3 position, vec3 normal, vec2 dim)
{
  float fov = abs(fov_up) + abs(fov_down);

  float x05 = (floor(img_coords.x) + 0.5f) / dim.x;
  float y05 = (floor(img_coords.y) + 0.5f) / dim.y;

  float theta = radians((1.0f - y05) * fov - fov_up) + pi_2;
  float phi = -(2.0 * x05 - 1.0f) * pi;

  vec3 nu = vec3(sin(theta)*cos(phi), sin(theta)*sin(phi), cos(theta));

  float r = dot(normal, position) / dot(normal, nu);

  return vec4(r * nu, 1.0);
}


vec3 slerp(vec3 v0, vec3 v1, float weight)
{

  float omega = acos(dot(normalize(v0), normalize(v1)));

  // weight is actually (1.0 - t), therefore inverted.
  float eta = 1.0 / sin(omega);
  float w0 = eta * sin(weight * omega);
  float w1 = eta * sin((1.-weight)* omega);

  return w0 * v0 + w1 * v1;
}

float get_radius(vec4 vertex, vec4 normal, float pixel_size)
{
  float d = length(vertex.xyz);

  return  1.41 * d * pixel_size / clamp(dot(normal.xyz, -vertex.xyz / d), 0.5, 1.0);
}

mat4 get_pose(int timestamp)
{
  int offset = 4 * timestamp;
  return mat4(texelFetch(poseBuffer, offset), texelFetch(poseBuffer, offset + 1),
              texelFetch(poseBuffer, offset + 2), texelFetch(poseBuffer, offset + 3));
}

void main()
{
  const float upper_stability_bound = 20.0;
  int surfel_age = timestamp - surfel_timestamp;

  int creation_timestamp = int(surfel_color_weight_count.z);
  // read column-wise surfel pose from texture buffer:
  mat4 surfelPose = get_pose(creation_timestamp);


  vec3 old_position = (surfelPose * vec4(surfel_position_radius.xyz, 1)).xyz;
  vec3 old_normal = (surfelPose* vec4(surfel_normal_confidence.xyz, 0)).xyz;
  float old_radius = surfel_position_radius.w;
  float old_confidence = surfel_normal_confidence.w;
  float old_weight = surfel_color_weight_count.y;

  vs_out.valid = true;
  if(old_confidence < confidence_threshold && use_stability) vs_out.valid = (surfel_age < unstable_age);
  vs_out.position_radius = surfel_position_radius;
  vs_out.normal_confidence = surfel_normal_confidence;
  vs_out.timestamp = surfel_timestamp;
  vs_out.color_weight_count = surfel_color_weight_count;
  vs_out.color_weight_count.x = pack(vec3(0.3, 0.3, 0.3));

  vs_out.semantic_map = surfel_semantic_map; // for semantic map

  vec4 vertex = inv_pose * vec4(old_position, 1.0);
  vec4 normal = normalize(inv_pose * vec4(old_normal, 0.0));

  bool visible = (dot(normal.xyz, -vertex.xyz / length(vertex.xyz)) > 0.00);
  vec3 img_coords = projectSpherical(vertex.xyz);
  vec3 dim = vec3(width, height, 1);
  gl_Position = vec4(-10.0);
  bool valid = (texture(vertex_map, img_coords.xy).w > 0.5f) && (texture(normal_map, img_coords.xy).w > 0.5f);
  bool inside = all(lessThan(img_coords, dim)) && !(all(lessThan(img_coords, vec3(0))));

  float penalty = 0.0;

  float update_confidence = log_prior;

  if(valid && inside && visible)
  {
    // detect and renalize outliers
    float data_label = texture(semantic_map_in, img_coords.xy).x * 255.0;
    float data_prob = texture(semantic_map_in, img_coords.xy).w;
    float model_label = surfel_semantic_map.x * 255.0;
    float model_prob = surfel_semantic_map.w;
    if(round(data_label) != round(model_label))
    {
      if( model_label == car.w || model_label == bicycle.w ||
          model_label == bus.w || model_label == motorcycle.w||
          model_label == truck.w|| model_label == other_vehicle.w||
          model_label == person.w||
          model_label == bicyclist.w || model_label == motorcyclist.w)
          penalty = 1.0;
    }

    mat4 surfelPose_inv = inverse(surfelPose);

    // Check if surfel and measurment are compatible.
    vec3 v = texture(vertex_map, img_coords.xy).xyz;
    vec3 n = texture(normal_map, img_coords.xy).xyz;
    vec4 v_global = pose * vec4(v, 1.0f);
    // Note: We assume non-scaling transformation, det(pose) = 1.0. Therefore Rot(pose^T^-1) = Rot(pose).
    vec4 n_global = normalize(pose * vec4(n, 0.0f));

    float depth = length(v);

    vec3 view_dir = -v/length(v);

    float distance = abs(dot(old_normal.xyz, v_global.xyz - old_position.xyz)); // distance(v_global.xyz, old_position.xyz);
    float angle = length(cross(n_global.xyz, old_normal.xyz));

    float new_radius = texture(radiusConfidence_map, img_coords.xy).x;
    float new_confidence = texture(radiusConfidence_map, img_coords.xy).y;

    // measurement is compatible, update conf & timestamp, but integrate measurment only if new radius is smaller!
    if((distance < distance_thresh) && (angle < angle_thresh))
    {
      gl_Position = vec4(2.0 * img_coords / dim - 1.0f, 1.0f); // measurement integrated: no need to generate surfel.
      float confidence = old_confidence + new_confidence;
      // always update confidence and timestamp:
      vs_out.normal_confidence.w = confidence;
      vs_out.timestamp = timestamp;
      float avg_radius = min(new_radius, old_radius);
      avg_radius = max(avg_radius, min_radius);
      vs_out.position_radius.w = avg_radius;
      vs_out.valid = true;

      vs_out.color_weight_count.x = pack(vec3(0.0, 0.7, 0.0f)); // green
      vs_out.color_weight_count.z = creation_timestamp;

      float r = (depth - min_depth) / (max_depth - min_depth);
      float a = angle;
      float d = distance;

      float p = p_stable;

      if(confidence_mode == 1 || confidence_mode == 3) p *= exp(-a*a / (sigma_angle*sigma_angle));
      if(confidence_mode == 2 || confidence_mode == 3) p *= exp(-d*d / (sigma_distance*sigma_distance));

      p = clamp(p, p_unstable, 1.0);

      update_confidence = log(p / (1.0 - p));

      if((new_radius < old_radius && timestamp - creation_timestamp < active_timestamps) || update_always)
      {
        float w1 = 0.9;
        float w2 = 0.1;

        if(weighting_scheme > 0)
        {
          w1 = old_weight;
          w2 = 1;
          if(weighting_scheme == 2) w2 = dot(n, view_dir);
          vs_out.color_weight_count.y = min(max_weight, w1 + w2);

          float sum = w1 + w2;
          w1 /= sum;
          w2 /= sum;
        }


        vec3 avg_position = w1 * old_position.xyz + w2 * v_global.xyz;
        // vec3 avg_normal = normalize(w1 * old_normal.xyz + w2 * n_global.xyz);
        vec3 avg_normal = slerp(old_normal.xyz, n_global.xyz, w1);

        // update semantic probability
        float avg_prob = 0;
        if(round(data_label) != round(model_label))
          avg_prob = w1 * model_prob + w2 * (1 - data_prob);
        else
          avg_prob = w1 * model_prob + w2 * data_prob;
        vs_out.semantic_map.w = avg_prob;

        // re-center the vertex such that it should cover the pixel again completely.
        // vec3 v_s =  (inv_pose * vec4(avg_position,1.0)).xyz;
        // vec3 n_s =  (inv_pose * vec4(avg_normal,0.0)).xyz;

        // vec4 cpos = centerize(img_coords.xy, v_s, n_s, textureSize(vertex_map));
        // avg_position = (pose * cpos).xyz;
        // avg_position = centerize(img_coords.xy, avg_position, avg_normal, textureSize(vertex_map));

        if(averaging_scheme == 1)
        {
          // move surfel along the normal towards measurment.
          avg_position = old_position.xyz + w2 * distance * old_normal.xyz;
          avg_normal = slerp(old_normal.xyz, n_global.xyz, w1);
        }

        avg_normal = normalize(avg_normal);
        avg_position = (surfelPose_inv * vec4(avg_position.xyz, 1)).xyz;
        avg_normal = (surfelPose_inv * vec4(avg_normal.xyz, 0)).xyz;

        vs_out.position_radius = vec4(avg_position.xyz, avg_radius);
        vs_out.normal_confidence = vec4(avg_normal.xyz, confidence);

        vs_out.color_weight_count.x = pack(vec3(1.0, 0.0, 1.0f)); // magenta: measurement integrated.
      }
    }
    else
    {
      int idx = int(texture(index_map, img_coords.xy)) - 1;
      if(idx == gl_VertexID) // ensure that surfel is closest visible surfel.
      {
        // if not matching; reduce confidence...
        update_confidence = log(p_unstable/(1.0 - p_unstable));
        vs_out.color_weight_count.x = pack(vec3(0.0, 1.0, 1.0f));
      }
    }
  }
  else if(visible && inside && false)
  {
    update_confidence = log_unstable;
    vs_out.color_weight_count.x = pack(vec3(1.0, 0.0, 0.0f));

    // we have a visible surfel without associated measurment => adjust radius to cover pixel
    int idx = int(texture(index_map, img_coords.xy)) - 1;
    if(idx == gl_VertexID && img_coords.y > 10) // ensure that surfel is closest visible surfel.
    {
      //vs_out.position_radius.w = min(old_radius, get_radius(vertex, normal, pixel_size));
    }
  }

  update_confidence = update_confidence - penalty;

  // static state bayes filter (see Thrun et al., Probabilistic Robotics, p. 286):
  if(use_stability)
    vs_out.normal_confidence.w = min(old_confidence + update_confidence - log_prior, upper_stability_bound);
  else
    vs_out.normal_confidence.w = old_confidence;

  if(vs_out.normal_confidence.w < log_unstable && use_stability) vs_out.valid = false;

}
