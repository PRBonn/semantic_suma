#version 330 core

/**
 *  \author behley
 **/

layout(points) in;
layout(line_strip, max_vertices = 6) out;

in SURFEL 
{
  float radius;
  vec4 normal;
  float confidence;
} gs_in[];

uniform mat4 mvp;
uniform mat4 mvp_inv_t;

out vec4 color;

#include "shader/color.glsl"

void main()
{
  
  vec4 p = mvp * gl_in[0].gl_Position;
  vec4 n = gs_in[0].normal; 
  vec4 u = normalize(vec4(n.y - n.z, -n.x, n.x, 0.0f));
  vec4 v = vec4(normalize(cross(n.xyz, u.xyz)), 0.0f); 
  
  p += 0.01 * mvp_inv_t * n; // shift slightly along the normal to avoid "z-fighting"
  
  color = RED;
  gl_Position = p;
  EmitVertex();
    
  gl_Position = p + 0.05 * mvp_inv_t * u;
  EmitVertex();
  
  EndPrimitive();
  
  color = GREEN;
  gl_Position = p ;
  EmitVertex();
    
  gl_Position = p + 0.05 * mvp_inv_t * v;
  EmitVertex();
  
  EndPrimitive();
  
  
  color = BLUE;
  gl_Position = p;
  EmitVertex();
    
  gl_Position = p + 0.05 * mvp_inv_t * n;
  EmitVertex();
  
  EndPrimitive();
  

}
