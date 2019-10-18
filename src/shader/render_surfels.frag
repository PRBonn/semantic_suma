#version 330 core

/**
 *  \author behley
 *
 *  \author Xieyuanli Chen
 **/

in vec2 texCoords;
in vec4 vertex;
in vec4 normal;
in vec4 semantic;
in float confidence;

layout (location = 0) out vec4 vertexmap;
layout (location = 1) out vec4 normalmap;
layout (location = 2) out vec4 semantic_rangemap;

void main()
{

 if(dot(texCoords, texCoords) > 1.0f)
 {
  vertexmap = vec4(0);
  normalmap = vec4(0);
  semantic_rangemap = vec4(0);
  discard;
 }

 vertexmap = vec4(vertex.xyz, 1.0);
 normalmap = normal;
 semantic_rangemap = semantic;
}
