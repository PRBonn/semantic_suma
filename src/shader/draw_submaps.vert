#version 330 core

/**
 *  \author behley
 **/

layout (location = 0) in vec2 submap_centers;

uniform float submap_extent;

out SUBMAP 
{
  vec2 center;
  float extent;  
} vs_out;


void main()
{
  vs_out.center = submap_centers;
  vs_out.extent = submap_extent;
}
