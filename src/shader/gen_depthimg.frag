#version 330 core

/**
 *  \author behley
 **/

flat in vec2 depth_index;
flat in vec4 vertex_coord;
flat in vec4 image_coord;

layout (location = 0) out float color;  // depth image.
layout (location = 1) out int index;    // index map.
layout (location = 2) out vec4 vertex;  // vertex map.
layout (location = 3) out int valid;  // validity map.
layout (location = 4) out vec4 out_img_coord; // image coordinates

void main()
{
    color = depth_index.x;
    index = int(depth_index.y);
    vertex = vertex_coord;
    valid = 1;
    out_img_coord = image_coord;
} 
