#version 330 core

/**
 *  \author behley
 **/

layout(points) in;
layout(line_strip, max_vertices = 5) out;

in SUBMAP 
{
  vec2 center;
  float extent;  
} gs_in[];

uniform mat4 mvp;

out vec4 vertexColor;
uniform float submap_extent;

void main()
{
    float e = submap_extent;
    vertexColor = vec4(0,0,0,1);
    
    gl_Position = mvp * vec4(gs_in[0].center + vec2(e, e), 0, 1);
    EmitVertex();
    
    gl_Position = mvp * vec4(gs_in[0].center + vec2(e, -e), 0, 1);
    EmitVertex();
    
    gl_Position = mvp * vec4(gs_in[0].center + vec2(-e, -e), 0, 1);
    EmitVertex();
    
    gl_Position = mvp * vec4(gs_in[0].center + vec2(-e, e), 0, 1);
    EmitVertex();
    
    gl_Position = mvp * vec4(gs_in[0].center + vec2(e, e), 0, 1);
    EmitVertex();
    
    EndPrimitive();
}
