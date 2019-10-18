#version 330 core

/**
 *  \author behley
 *
 *  \author Xieyuanli Chen
 **/

#include "shader/color.glsl"

layout(points) in;
layout(points, max_vertices = 2) out;

uniform int timestamp;

in SURFEL {
  bool valid;
  vec2 img_coords;
} gs_in[];

out vec4 sfl_position_radius;
out vec4 sfl_normal_confidence;
out int sfl_timestamp;
out vec3 sfl_color_weight_count;
out vec4 sfl_semantic_map; // for semantic map

uniform sampler2DRect vertex_map;
uniform sampler2DRect normal_map;
uniform sampler2DRect radiusConfidence_map;
uniform sampler2DRect semantic_map; // for semantic maps

uniform mat4 pose; // current pose of laser scanner.
uniform float log_prior;
uniform float min_radius;
uniform float max_radius;

// fill-in for OpenGL 3.30: replacing one texture gather with 4 texture accesses.
vec4 textureGather(sampler2DRect sampler, vec2 coords, int comp)
{
	vec4 result;
	// Note: documentation is somehow incoherent here, but ARB shows this order:
	if(comp == 0)
	{
		result.x = texture(sampler, vec2(coords.x, coords.y)).x;
		result.y = texture(sampler, vec2(coords.x + 1, coords.y)).x;
		result.z = texture(sampler, vec2(coords.x + 1, coords.y - 1)).x;
		result.w = texture(sampler, vec2(coords.x, coords.y - 1)).x;
	}
	else if(comp == 1)
	{
		result.x = texture(sampler, vec2(coords.x, coords.y)).y;
		result.y = texture(sampler, vec2(coords.x + 1, coords.y)).y;
		result.z = texture(sampler, vec2(coords.x + 1, coords.y - 1)).y;
		result.w = texture(sampler, vec2(coords.x, coords.y - 1)).y;
	}
	else if(comp == 2)
	{
		result.x = texture(sampler, vec2(coords.x, coords.y)).z;
		result.y = texture(sampler, vec2(coords.x + 1, coords.y)).z;
		result.z = texture(sampler, vec2(coords.x + 1, coords.y - 1)).z;
		result.w = texture(sampler, vec2(coords.x, coords.y - 1)).z;
	}
	else if(comp == 3)
	{
		result.x = texture(sampler, vec2(coords.x, coords.y)).w;
		result.y = texture(sampler, vec2(coords.x + 1, coords.y)).w;
		result.z = texture(sampler, vec2(coords.x + 1, coords.y - 1)).w;
		result.w = texture(sampler, vec2(coords.x, coords.y - 1)).w;
	}

	return result;
}

void main()
{
  // filter invalid surfels.
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

    EmitVertex();
    EndPrimitive();
  }
}
