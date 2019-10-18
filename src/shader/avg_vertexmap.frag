#version 330 core
/**
 *  \author behley
 **/

in vec2 texCoords;

uniform sampler2DRect in_vertexmap;

out vec4 out_vertexmap;

void main()
{
  vec2 dim = textureSize(in_vertexmap);
  vec2 p = vec2(int(texCoords.x * dim.x), int(texCoords.y * dim.y));
  
  out_vertexmap = texture(in_vertexmap, p);
  if(out_vertexmap.w > 0.5) out_vertexmap = out_vertexmap / out_vertexmap.w;
}
