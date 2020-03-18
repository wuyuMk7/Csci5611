#version 330 core

out vec4 outColor;

in vec4 sharedTriangleColor;

void main()
{
  outColor = sharedTriangleColor;
}
