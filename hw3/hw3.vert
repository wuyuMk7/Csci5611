#version 330 core
layout (location=0) in vec3 trianglePos;
layout (location=1) in vec3 triangleColor;
layout (location=2) in float radius;
layout (location=3) in vec2 texCoord;

out vec4 sharedTriangleColor;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
  gl_Position = projection * view * model * vec4(trianglePos, 1.0f);
  //gl_Position = vec4(0.5f, 0.5f, 0.0f, 1.0f);
  //gl_Position = vec4(trianglePos, 1.0f);
  gl_PointSize = 10.0f;
  sharedTriangleColor = vec4(triangleColor, 1.0f);
}
