#version 330

/** \brief Computation of residual & Jacobian from given model vertices & data vertices.
 *  
 *  Generating everything needed for computation of (weighted) Jacobian products. 
 *
 *  Computation happens in the geometry shader, since this turned out to be more productive.
 *   
 *
 *  \author behley
 **/

layout (location = 0) in vec2 texCoords;

out VS_OUT {
  vec2 texCoords;
} vs_out;
  
void main()
{
  gl_Position = vec4(0.5);
  vs_out.texCoords = texCoords;
}