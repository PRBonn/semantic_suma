#version 330 core

/**
 *  \author behley
 **/

in vec4 point_color;
out vec4 color;

void main()
{
    color = point_color;
} 
