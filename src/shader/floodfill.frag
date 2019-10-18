#version 330

/**
 *  \author Xieyuanli Chen
 **/

in vec2 texCoords;

uniform int width;
uniform int height;

uniform sampler2DRect vertex_map;
uniform sampler2DRect semantic_map;

layout (location = 0) out vec4 refined_semantic_map;

float wrap(float x, float dim)
{
  float value = x;

  while(value >= dim) value = (value - dim);
  while(value < 0) value = (value + dim);

  return value;
}

bool valid(vec4 normal)
{
  return (normal.w > 0.5);
}

vec4 invalid = vec4(0.0, 0.0, 0.0, 1.0);

void main()
{
  float width = textureSize(semantic_map).x;
  vec2 pos = texCoords * textureSize(semantic_map);
  refined_semantic_map = texture(semantic_map, pos);

  int kernel_size = 3;

  // floodfill
  {
    for(int offset = 1; offset < kernel_size; offset++)
    {
      vec4 p = texture(vertex_map, pos);
      vec4 u = texture(vertex_map, vec2(wrap(pos.x + offset, width), pos.y));
      vec4 v = texture(vertex_map, vec2(pos.x, pos.y + offset));
      vec4 s = texture(vertex_map, vec2(wrap(pos.x - offset, width), pos.y));
      vec4 t = texture(vertex_map, vec2(pos.x, pos.y - offset));

      vec4 p_label = texture(semantic_map, pos);
      vec4 u_label = texture(semantic_map, vec2(wrap(pos.x + offset, width), pos.y));
      vec4 v_label = texture(semantic_map, vec2(pos.x, pos.y + offset));
      vec4 s_label = texture(semantic_map, vec2(wrap(pos.x - offset, width), pos.y));
      vec4 t_label = texture(semantic_map, vec2(pos.x, pos.y - offset));

      float threshold = 0.007;

      // use reciprocal decay

      if(p_label.x == 0.0 && u_label.x != 0 && abs(length(p.xyz) - length(u.xyz)) < threshold * length(p.xyz))
      {
        refined_semantic_map = vec4(u_label.xyz, u_label.w / (offset + 1));
        break;
      }
      else if(p_label.x == 0.0 && v_label.x != 0 && abs(length(p.xyz) - length(v.xyz)) < threshold * length(p.xyz))
      {
        refined_semantic_map = vec4(v_label.xyz, v_label.w / (offset + 1));
        break;
      }
      else if(p_label.x == 0.0 && s_label.x != 0 && abs(length(p.xyz) - length(s.xyz)) < threshold * length(p.xyz))
      {
        refined_semantic_map = vec4(s_label.xyz, s_label.w / (offset + 1));
        break;
      }
      else if(p_label.x == 0.0 && t_label.x != 0 && abs(length(p.xyz) - length(t.xyz)) < threshold * length(p.xyz))
      {
        refined_semantic_map = vec4(t_label.xyz, t_label.w / (offset + 1));
        break;
      }
    }
  }
}
