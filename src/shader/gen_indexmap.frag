#version 330 core

/**
 *  \author behley
 **/

flat in float index;
in vec4 vertex;
in vec4 normal;

layout (location = 0) out float indexmap;
layout (location = 1) out vec4 vertexmap;
layout (location = 2) out vec4 normalmap;

void main()
{
  indexmap = index;
  vertexmap = vertex;
  normalmap = normal;
}
