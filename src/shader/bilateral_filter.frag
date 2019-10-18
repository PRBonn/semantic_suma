#version 330 core

/**
 *  \author behley
 **/

in vec2 texCoords;

uniform sampler2DRect in_vertexmap;
uniform float width;
uniform float height;
uniform float sigma_space;
uniform float sigma_range;

out vec4 out_vertexmap;

float wrap(float x, float dim)
{
  float value = x;
  
  while(value >= dim) value = (value - dim);
  while(value < 0) value = (value + dim);
  
  return value;
}

void main()
{
  int x = int(texCoords.x * width);
  int y = int(texCoords.y * height);

  vec4 vertex = texture(in_vertexmap, vec2(x, y));
  
  out_vertexmap = vertex;
  
  if(vertex.w > 0.5)
  {
    float range = length(vertex.xyz);
    vec3 ray = vertex.xyz / range;
    
  
    float sigma_space_factor = -0.5 / (sigma_space * sigma_space); // elastic fusion: 0.024691358 => sigma = 4.5
    float sigma_range_factor = -0.5 / (sigma_range * sigma_range); // elastic fusion: 0.000555556 => sigma = 30
          
    const int R = 6; // make R dependent on sigma_space?
    const int D = R * 2 + 1;
      
    int tx = x - D / 2 + D;
    int ty = min(y - D / 2 + D, int(height));
  
    float sum1 = 0.0f;
    float sum2 = 0.0f;
      
    for(int cy = max(y - D / 2, 0); cy < ty; ++cy)
    {
        for(int cx = x - D / 2; cx < tx; ++cx)
        { 
          float xx = wrap(cx, width);
        
          vec4 tmp = texture(in_vertexmap, vec2(xx, cy));
          if(tmp.w < 0.5) continue;
          
          float tmp_range = length(tmp);
          
          
          float diff_space2 = (x-xx) * (x-xx) + (y-cy) * (y-cy); // dot((vertex.xyz - tmp.xyz) , (vertex.xyz - tmp.xyz));
          float diff_range2 = (range - tmp_range) * (range - tmp_range);

          float weight = exp(diff_space2 * sigma_space_factor + diff_range2 * sigma_range_factor);

          sum1 += tmp_range * weight;
          sum2 += weight;
        }
    }
    
    float filtered_range = sum1 / sum2; 
    
    out_vertexmap = vec4( filtered_range * ray, 1.0);
  }
}
