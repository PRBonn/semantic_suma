#version 330 core

/**
 *  \author behley
 **/

layout(points) in;
layout(line_strip, max_vertices = 4) out;

uniform mat4 from;
uniform mat4 to;
uniform mat4 measurement;

uniform mat4 mvp;
uniform mat4 mvp_inv_t;

out vec4 color;

// better solution would be to use instancing, but for now this hack will do it.

void main()
{
  // edge connecting the  poses:
  vec4 origin = vec4(0,0,0,1);
  color = vec4(0.5, 0, 0.95, 1.0);
  gl_Position = mvp * from * origin;
  EmitVertex();
    
  gl_Position = mvp * to * origin;
  EmitVertex();   
  
  EndPrimitive();
  
  color = vec4(1.0, 0.0, 0.0, 1.0);
  vec4 tip = from * measurement * origin;
      
  gl_Position = mvp * tip;
  EmitVertex();    
  
  color = vec4(0.0, 1.0, 0.0, 1.0);

  gl_Position = mvp * (from * measurement * vec4(1,0,0,0) + tip);
  EmitVertex();
  EndPrimitive();
  
  
}
