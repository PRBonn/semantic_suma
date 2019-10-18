#version 330 core

/**
 *  \author behley
 **/

in float valid;
in vec4 centerized_vertex;
in float radius;
in float confidence;

layout (location = 0) out vec4 centerized_vertex_map;
layout (location = 1) out vec4 radius_confidence_map;


void main()
{
  centerized_vertex_map = centerized_vertex;
  radius_confidence_map = vec4(radius, confidence, 0, valid);
}
