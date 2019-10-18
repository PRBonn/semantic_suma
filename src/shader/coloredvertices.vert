#version 330 core
/**
 *  \author behley
 **/

layout (location = 0) in vec4 position_color;

uniform mat4 mvp;

out vec4 vertexColor;

uniform bool useCustomColor;
uniform vec4 customColor;

vec4 decodeColor(float c)
{
    vec4 col;
    col.x = float(int(c) >> 16 & 0xFF) / 255.0f;
    col.y = float(int(c) >> 8 & 0xFF) / 255.0f;
    col.z = float(int(c) & 0xFF) / 255.0f;
    col.w = 1.0f;
    
    return col;
}

void main()
{
    gl_Position = mvp * vec4(position_color.xyz, 1.0);
    vertexColor = decodeColor(position_color.w);
    if(useCustomColor) vertexColor = customColor;
}
