#version 450

layout(location = 0) in vec3 fragColor;
layout(location = 1) in vec2 fragTexCoord;
layout(location = 2) in vec3 fragNormal;
layout(location = 3) in vec3 fragLightDir;
layout(location = 4) in vec3 fragPosition;


layout(location = 0) out vec4 outColorShading;
layout(location = 1) out vec4 outColorPos;

void main() 
{
    outColorShading = vec4(fragColor.rgb, 1.0);
	
	outColorPos = vec4(fragPosition.rgb, 1.0);
}