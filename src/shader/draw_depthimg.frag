#version 330 core

/**
 *  \author behley
 **/

in vec2 texCoords;
out vec4 color;

uniform int colorMode; // 0 - intensity, 1 - hsv, 2 - raw 
uniform sampler2DRect depthTexture;

uniform bool markInvalidDepthValues;
uniform bool filterDepth;
uniform float minDepth;
uniform float maxDepth;



// source: http://lolengine.net/blog/2013/07/27/rgb-to-hsv-in-glsl
vec3 hsv2rgb(vec3 c)
{
    vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
    vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);
    
    return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);
}

const vec4 rviridis = vec4(2.90912735, -2.14404531, 0.04439198,  0.29390206);
const vec4 gviridis = vec4(-0.17293242, -0.16906214,  1.24131122,  0.01871256);
const vec4 bviridis = vec4(0.17848859, -1.72405244,  1.23042564,  0.34479632);

// approximate version of viridis colormap.
vec3 viridis(float t)
{
  vec4 tt = vec4(t*t*t, t*t, t, 1.0);
  return vec3(dot(tt, rviridis), dot(tt, gviridis), dot(tt, bviridis)); 
}


void main()
{
  float depth = length(texture(depthTexture, texCoords * textureSize(depthTexture)).xyz);
  bool valid = bool(texture(depthTexture, texCoords * textureSize(depthTexture)).w);
  
  color = vec4(0.0, 0.0, 0.0, 1.0); // default color.
  
  if(!valid && markInvalidDepthValues)
  {
    color = vec4(1.0, 0.0, 0.0, 1.0);
  }
  else
  {
    if(filterDepth && (depth < minDepth || depth > maxDepth))
    {
      color = vec4(0.0, 0.0, 0.0, 1.0);
    }
    else
    {
      if(filterDepth && colorMode != 2) depth = (depth - minDepth) / (maxDepth-minDepth); 
      else if(colorMode != 2) depth = depth/50.0f;
  
      if(colorMode == 0) color = vec4(viridis(depth), 1.0);
      else if(colorMode == 1) color = vec4(hsv2rgb(vec3(depth, 1.0, 1.0)), 1.0);
      else if(colorMode == 2) color = vec4(depth, 0, 0, 1.0);
    }
  }
}
