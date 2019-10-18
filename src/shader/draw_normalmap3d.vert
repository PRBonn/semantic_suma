#version 330 core

/**
 *  \author behley
 **/

layout (location = 0) in vec2 tex_position;

uniform sampler2DRect texVertexMap;
uniform sampler2DRect texNormalMap;

uniform mat4 mvp;
uniform mat4 mvp_inv_t;

out VS_OUT {
  bool valid;
  vec4 normal;
  vec4 color;
} vs_out;

void main()
{

  gl_Position = mvp * vec4(texture(texVertexMap, tex_position).rgb, 1.0);
  
  
  vs_out.valid = (texture(texNormalMap, tex_position).w > 0.5f);
  vs_out.normal = mvp_inv_t * vec4(texture(texNormalMap, tex_position).rgb, 0.0f);
  vs_out.color = vec4(abs(texture(texNormalMap, tex_position).rgb), 1.0f);
}
