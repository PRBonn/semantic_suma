#version 330 core

/**
 *  \author behley
 *
 *  \author Xieyuanli Chen
 **/

in vec2 texCoords;

layout (location = 0) out vec4 vertexmap;
layout (location = 1) out vec4 normalmap;
layout (location = 2) out vec4 semanticmap;


uniform sampler2DRect old_vertexmap;
uniform sampler2DRect old_normalmap;
uniform sampler2DRect new_vertexmap;
uniform sampler2DRect new_normalmap;
uniform sampler2DRect old_semanticmap;
uniform sampler2DRect new_semanticmap;

uniform float max_distance; // maximum distance of old vertices to consider.


void main()
{
  vec2 dim = textureSize(new_vertexmap);
  vec2 img_coords = texCoords * dim;

  vertexmap = texture(new_vertexmap, img_coords);
  normalmap = texture(new_normalmap, img_coords);
  semanticmap = texture(new_semanticmap, img_coords);

  vec4 old_vertex = texture(old_vertexmap, img_coords);
  vec4 old_normal = texture(old_normalmap, img_coords);
  vec4 old_semantic = texture(old_semanticmap, img_coords);

  bool valid = (old_vertex.w > 0.5f && old_normal.w > 0.5f);
  bool new_valid = (vertexmap.w > 0.5f && normalmap.w > 0.5f);

  if(!new_valid && valid && (vertexmap.w < 0.5f || length(vertexmap.xyz - old_vertex.xyz) < max_distance))
  {
     vertexmap = old_vertex;
     normalmap = old_normal;
     semanticmap = old_semantic;
  }
}
