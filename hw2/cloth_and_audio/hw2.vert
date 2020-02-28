#version 330 core
layout (location=0) in vec3 trianglePos;
layout (location=1) in vec3 triangleColor;
layout (location=2) in float radius;
layout (location=3) in vec2 texCoord;

out vec4 sharedTriangleColor;
out vec2 sharedTextureCoord;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
  //gl_Position = vec4(trianglePos.x + xOffset, trianglePos.y, trianglePos.z, trianglePos.w);
  gl_Position = projection * view * model * vec4(trianglePos, 1.0f);
  //gl_Position = trianglePos;
  gl_PointSize = radius;
  sharedTriangleColor = vec4(triangleColor, 1.0f);
  sharedTextureCoord = texCoord;
}
