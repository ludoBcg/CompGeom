#version 450

// INPUT 
layout(location = 0) in vec3 fragColor;
layout(location = 1) in vec2 fragTexCoord;
layout(location = 2) in vec3 fragNormal;
layout(location = 3) in vec3 fragLightDir;


// OUTPUT 
layout(location = 0) out vec4 outColorShading;

void main() 
{
    outColorShading = vec4(fragColor.rgb, 1.0);
}