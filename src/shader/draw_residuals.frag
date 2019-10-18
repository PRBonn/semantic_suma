#version 330 core

/**
 *  \author behley
 **/

in vec2 texCoords;
out vec4 color;


uniform sampler2DRect residualmap;

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
  vec4 residual = texture(residualmap, texCoords * textureSize(residualmap));
  //color = vec4(viridis(clamp(abs(residual.y) / 2.0, 0.0, 1.0)), 1.0);
  if(residual.w > 0.5 && residual.x > 0.5) color = vec4(0,1,0,1);
  else if(residual.w < -1.5)  color = vec4(1,0,0,1);
  else if(residual.w < -0.5) color = vec4(0,0,1,1);
  else if(residual.x < 0.5) color = vec4(0,0,0,1);
}
