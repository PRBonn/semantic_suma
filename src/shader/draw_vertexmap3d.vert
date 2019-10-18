#version 330 core

/**
 *  \author behley
 **/

layout (location = 0) in vec2 tex_position;

uniform sampler2DRect texVertexMap;
uniform sampler2DRect texResidualMap;
uniform mat4 mvp;

uniform int colorMode; // 0 - depth grey, 1 - hsv, 2 - red/depth, 3 - residual, 4 - outlier
uniform vec4 customColor;

out vec4 color;

// source: http://lolengine.net/blog/2013/07/27/rgb-to-hsv-in-glsl
vec3 hsv2rgb(vec3 c)
{
    vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
    vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);
    
    return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);
}

void main()
{
  vec4 coords = vec4(texture(texVertexMap, tex_position).rgb, 1.0);
  vec4 residuals = texture(texResidualMap, tex_position).rgba;
  float depth = length(coords.xyz)/50.0f; 
  
  gl_Position = mvp * coords;
  
  color = vec4(0.0f, 0.0f, 0.0f, 1.0f);
  
  if(colorMode == 0) color = vec4(depth, depth, depth, 1.0);
  else if(colorMode == 1) color = vec4(hsv2rgb(vec3(depth, 1.0, 1.0)), 1.0);
  else if(colorMode == 2) color = vec4(depth, 0, 0, 1.0);
  else if(colorMode == 3) color = vec4(hsv2rgb(vec3(abs(residuals.g), 1.0, 1.0)), 1.0);
  else if(colorMode == 4) {
    if(int(residuals.a) < 0.5) color = vec4(1,0,0,1);
    else color = vec4(0,1,0,1);
  }
  else if(colorMode == 5)
  {
    color = vec4(hsv2rgb(vec3(residuals.b, 1.0, 1.0)), 1.0);
  }
  else if(colorMode == 6)
  {
    if(residuals.a < -2.0) color = vec4(1,0,0,1);
    else color = vec4(0,1,0,1);
  }
}
