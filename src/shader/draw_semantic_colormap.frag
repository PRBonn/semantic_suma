#version 330 core

/**
 *  \author Xieyuanli Chen
 **/

in vec2 texCoords;
out vec4 color;

uniform sampler2DRect screenTexture;
uniform sampler1D color_map; // for semantic maps
const vec4 invalid = vec4(0.0, 0.0, 0.0, 1.0);

void main()
{
  vec4 semantic_label = texture(screenTexture, texCoords * textureSize(screenTexture));

  if(semantic_label.x == 0.0 || semantic_label.w < 0.9) color = invalid;

  else
  {
    float textureSize1d = 259.0f;
    vec4 sfl_semantic = vec4(texture(color_map, semantic_label.x * 255.0f / textureSize1d).rgb, 1.0);
    color = sfl_semantic;
  }
}
