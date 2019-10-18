#version 330

/**
 *  \author behley
 **/

in vec3 values;

// layout (location = 0) 
out vec3 result;

void main()
{
  // blending ensures that the sum is computed.
  result = values;
}
