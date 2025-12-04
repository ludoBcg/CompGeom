#version 450

layout(location = 0) in vec3 fragPosition;


layout(location = 0) out vec4 outColorPos;

void main() 
{
    outColorPos = vec4(fragPosition.rgb, 1.0);
}