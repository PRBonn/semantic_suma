#version 330 core

/**
 *  \author behley
 **/

layout(points) in;
layout(line_strip, max_vertices = 2) out;

in VS_OUT {
  bool valid;
  vec4 normal;
  vec4 color;
} gs_in[];

out vec4 color;

void main()
{
  if(gs_in[0].valid)
  {
    color = gs_in[0].color;
    gl_Position = gl_in[0].gl_Position;
    EmitVertex();
    
    gl_Position = gl_in[0].gl_Position + 0.1 * gs_in[0].normal;
    EmitVertex();
    
    EndPrimitive();
  }
}
