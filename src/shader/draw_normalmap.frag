#version 330 core

/**
 *  \author behley
 **/

in vec2 texCoords;
out vec4 color;

uniform sampler2DRect screenTexture;
const vec4 invalid = vec4(0.0, 0.0, 0.0f, 1.0f);

void main()
{ 
  color = abs(texture(screenTexture, texCoords * textureSize(screenTexture)));
  if(color.w < 0.5f) color = invalid;
}
