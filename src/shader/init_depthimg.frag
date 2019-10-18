#version 330 core

/**
 *  \author behley
 **/

flat in vec2 depth_index;
flat in vec3 vertex_coord;

layout (location = 0) out float depth;  // depth image.
layout (location = 1) out int index;    // index map.
layout (location = 2) out vec3 vertex;  // vertex map.
layout (location = 3) out int valid; // validity map.

void main()
{
    depth = -1.0;
    index = -1;
    vertex = vec3(1./0.);
    valid = 0;
} 
