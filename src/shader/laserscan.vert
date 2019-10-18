#version 330 core

/**
 *  \author behley
 **/

layout (location = 0) in vec4 position;
layout (location = 1) in float remission;

uniform mat4 mvp;
uniform int pointColorMode; // 0 - black, 1 - remission, 2 - depth, 3 - custom color, 4 - remission in w coordinate.
uniform vec4 customColor;
uniform bool removeGround;

out vec4 point_color;

#include "shader/color.glsl"

void main()
{
    if(removeGround)
    {
      if(position.z < -1.0) gl_Position = vec4(-10, -10, -10, 1);
      else gl_Position = mvp * vec4(position.xyz, 1.0);
    }
    else
    {
      gl_Position = mvp * vec4(position.xyz, 1.0);
    }
    
    if(pointColorMode == 1) 
    {
      point_color = vec4(remission, remission, remission, 1.0f);
    }
    else if(pointColorMode == 2)
    {
      float depth = length(vec3(position.x, position.y, position.z));
      vec3 hsv = hsv2rgb(vec3(depth/50.0, 1.0, 1.0));
      point_color = vec4(hsv, 1.0);
    }
    else if(pointColorMode == 3)
    {
      //vec3 hsv = rgb2hsv(customColor.xyz);
      point_color = customColor;//vec4(hsv2rgb(vec3(hsv.x, 1.0, remission)), 1.0f);
    }
    else if(pointColorMode == 4)
    {
      point_color = vec4(position.w, position.w, position.w, 1.0f); 
    }
    else if(pointColorMode == 5)
    {
      vec3 hsv = rgb2hsv(customColor.xyz);
      point_color = vec4(hsv2rgb(vec3(hsv.x, 1.0, max(2*position.w, 0.5))), 1.0f);
    }
    else if(pointColorMode == 6)
    {
      vec3 hsv = rgb2hsv(customColor.xyz);
      point_color = vec4(hsv2rgb(vec3(hsv.x, 1.0, max(remission, 0.3))), 1.0f);
    }
    else if(pointColorMode == 7)
    {
      point_color = vec4(hsv2rgb(vec3(position.z / 10.0, 1.0, max(2*position.w, 0.5))), 1.0f);
    }
    else
    {
      point_color = vec4(0.0f, 0.0f, 0.0f, 1.0f);
    }
    
    
}
