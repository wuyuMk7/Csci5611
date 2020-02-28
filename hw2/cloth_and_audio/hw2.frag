#version 330 core

out vec4 outColor;

in vec4 sharedTriangleColor;
in vec2 sharedTextureCoord;

uniform sampler2D flagTexture;

void main()
{
  //vec2 pos = mod(gl_FragCoord.xy, vec2(50.0f)) - vec2(25.0f);
  //float dist_squared = dot(pos, pos);

  //outColor = sharedTriangleColor;
  //outColor = dist_squared < 400.0 ? sharedTriangleColor : vec4(.20f, .20f, .40f, 1.0f);
  //vec2 coord = gl_PointCoord - vec2(0.5);
  //outColor = length(coord) < 0.5 ? sharedTriangleColor : vec4(1.0f, 1.0f, 1.0f, 1.0f);
  //outColor = sharedTriangleColor;
  outColor = texture(flagTexture, sharedTextureCoord) * sharedTriangleColor;
}
