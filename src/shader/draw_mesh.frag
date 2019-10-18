#version 330

/**
 *  \author behley
 **/

in vec4 pos;
in vec4 normal;

struct Light
{
  vec4 position; // directional lights have w == 0.
  
  vec3 ambient;
  vec3 diffuse;
  vec3 specular;
};

uniform int num_lights;
uniform Light lights[10];

uniform vec4 view_pos;

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

out vec4 color;

// see also http://learnopengl.com/#!Lighting/Materials

void main()
{
  vec3 norm = normalize(vec3(normal));
  vec3 view_dir = normalize((view_pos - pos).xyz);
    
  vec3 result = vec3(0);

  for(int i = 0; i < num_lights; ++i)
  {
    // ambient:
    vec3 ambient = lights[i].ambient * material.ambient;
    
    // diffuse:
    vec3 light_dir = normalize(lights[i].position.xyz - pos.xyz);
    if(lights[i].position.w < 0.0001) light_dir = normalize(-lights[i].position.xyz); // directional light.
    float diff = abs(dot(norm, light_dir));
    vec3 diffuse = lights[i].diffuse * (diff * material.diffuse); 
    
    // specular:
    vec3 reflect_dir = reflect(-light_dir, norm);
    float spec = pow(max(dot(view_dir, reflect_dir), 0.0), material.shininess);
    vec3 specular =  lights[i].specular * (spec * material.specular);
    
    // emission (seems good to apply for each light the emission; with this the emission part doesn't just depend
    // on the number of lights:
    result += ambient + diffuse + specular + material.emission;
  }

  color = vec4(result, material.alpha);
}
