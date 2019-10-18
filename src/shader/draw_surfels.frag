#version 330 core

/**
 *  \author behley
 **/

uniform int surfelDrawMethod; // 0 - texCoord-based fragement discard, 2 - hexagon based.

in vec2 texCoords;
in vec4 gs_color;
out vec4 color;
in float radius;

// phong shading?

void main()
{

  if (gs_color.w < 0.5) discard;

  if(surfelDrawMethod == 0)
  {
    if(dot(texCoords, texCoords) > 1.0f) discard;
    color = gs_color;
  }
  else
  {
    color = gs_color;
  }
}
