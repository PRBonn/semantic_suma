#version 330 core

/**
 *  \author behley
 *
 *  \author Xieyuanli Chen
 **/

layout(points) in;
layout(triangle_strip, max_vertices = 6) out;

in SURFEL
{
  float radius;
  vec4 normal;
  float confidence;
  int timestamp;
  vec3 color;
  vec4 semantic_map; // for semantic map
} gs_in[];


struct Light
{
  vec4 position; // directional lights have w == 0.

  vec3 ambient;
  vec3 diffuse;
  vec3 specular;
};

uniform int num_lights;
uniform Light lights[10];

struct Material
{
  vec3 ambient;
  vec3 diffuse;
  vec3 specular;
  vec3 emission;

  float shininess;
  float alpha;
};

uniform Material material;

uniform mat4 mvp;
uniform mat4 mvp_inv_t;
uniform int surfelDrawMethod; // 0 - texCoord-based fragement discard, 2 - hexagon based.
uniform int colorMode; // 0 - phong, 1 - normal shading, 2 - normal color, 3 - confidence
uniform float conf_threshold;

uniform vec3 view_pos;
uniform bool drawCurrentSurfelsOnly;
uniform int timestamp;
uniform bool backface_culling;
uniform bool use_stability;

uniform sampler1D color_map;

out vec4 gs_color;
out vec2 texCoords;
out float radius;

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
  vec4 p = gl_in[0].gl_Position;
  vec4 n = gs_in[0].normal;
  radius = gs_in[0].radius;
  vec4 u = normalize(vec4(n.y - n.z, -n.x, n.x, 0.0f));
  vec4 v = vec4(normalize(cross(n.xyz, u.xyz)), 0.0f);

  float c = gs_in[0].confidence;
  bool valid = (c > conf_threshold || !use_stability);
  vec3 view_dir = normalize((view_pos.xyz - p.xyz));

  if(colorMode == 1)
  {
   gs_color = vec4((vec3(.5f, .5f, .5f) * abs(dot(n.xyz, vec3(1.0, 1.0, 1.0)))) + vec3(0.1f, 0.1f, 0.1f), 1.0f);
  }
  else if(colorMode == 2)
  {
    gs_color = vec4(abs(n.xyz), 1.0f);
  }
  else if(colorMode == 3)
  {
    valid = true;
    gs_color = vec4(viridis(1.0-1.0/(1.0 + exp(c))), 1.0);
  }
  else if(colorMode == 5)
  {
    vec4 semantic_label = vec4(texture(color_map, gs_in[0].semantic_map.x * 255.0f / 259.0f).rgb, 1.0);
    if (gs_in[0].semantic_map.x != 0) gs_color = semantic_label;
    else valid = false;
  }
  else
  {
    vec3 norm = normalize(vec3(n));


    vec3 result = vec3(0);

    vec3 surfel_color = material.diffuse;
    float alpha = material.alpha;

    if (colorMode == 4)
    {
      surfel_color = gs_in[0].color;
      valid = true;
      alpha =  1.0 - clamp(conf_threshold - c, 0.1, 1);
      radius /= sqrt(2);
    }



    for(int i = 0; i < num_lights; ++i)
    {
      // ambient:
      vec3 ambient = lights[i].ambient * material.ambient;

      // diffuse:
      vec3 light_dir = normalize(lights[i].position.xyz - p.xyz);
      if(lights[i].position.w < 0.0001) light_dir = normalize(-lights[i].position.xyz); // directional light.
      float diff = abs(dot(norm, light_dir));
      vec3 diffuse = lights[i].diffuse * (diff * surfel_color);

      // specular:
      vec3 reflect_dir = reflect(-light_dir, norm);
      float spec = pow(max(dot(view_dir, reflect_dir), 0.0), material.shininess);
      vec3 specular =  lights[i].specular * (spec * material.specular);

      // emission (seems good to apply for each light the emission; with this the emission part doesn't just depend
      // on the number of lights:
      result += ambient + diffuse + specular + material.emission;
    }

     gs_color = vec4(result, alpha);
  }

  valid = valid && (!backface_culling || (dot(view_dir, n.xyz) > 0));

  if(drawCurrentSurfelsOnly) valid = (gs_in[0].timestamp == timestamp);

  if(surfelDrawMethod == 0 && valid)
  {
    vec4 ru = radius * u;
    vec4 rv = radius * v;

    gl_Position = mvp * (p - ru - rv);
    texCoords = vec2(-1.0, -1.0);
    EmitVertex();

    gl_Position =  mvp * (p + ru - rv);
    texCoords = vec2(1.0, -1.0);
    EmitVertex();

    gl_Position =  mvp * (p - ru + rv);
    texCoords = vec2(-1.0, 1.0);
    EmitVertex();

    gl_Position =  mvp * (p + ru + rv);
    texCoords = vec2(1.0, 1.0);

    EmitVertex();
    EndPrimitive();
  }

}
