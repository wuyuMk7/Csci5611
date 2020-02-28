#version 330 core
layout (location=0) in vec3 trianglePos;
layout (location=1) in vec3 triangleColor;
layout (location=2) in float radius;

out vec4 sharedTriangleColor;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
  gl_Position = projection * view * model * vec4(trianglePos, 1.0f);
  gl_PointSize = radius;
  sharedTriangleColor = vec4(triangleColor, 1.0f);
}
