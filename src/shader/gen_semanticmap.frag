#version 330

/**
 *  \author Xieyuanli Chen
 **/

in vec2 texCoords;

uniform sampler2DRect color_mask;

layout (location = 0) out vec4 semantic_map;

const vec4 invalid = vec4(0.0, 0.0, 0.0f, 1.0f);

void main()
{
  semantic_map = texture(color_mask, vec2(texCoords.x, 1.0 - texCoords.y) * textureSize(color_mask));
}
