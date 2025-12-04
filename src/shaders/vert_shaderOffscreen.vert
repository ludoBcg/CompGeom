#version 450


// UNIFORMS INPUT  (set = 0 is optionnal, only used in case of multiple descriptor sets)
layout(set = 0, binding = 0) uniform UniformBufferObject
{
    mat4 model;
    mat4 view;
    mat4 proj;
    vec3 lightPos;
} ubo;


// ATTRIBUTE INPUT (i.e., vertex buffer data)
layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec2 inTexCoord;
layout(location = 3) in vec3 inNormal;


// OUTPUT 
layout(location = 0) out vec3 fragPosition;

void main() 
{
    gl_Position = ubo.proj * ubo.view * ubo.model * vec4(inPosition, 1.0);

	fragPosition = inPosition;    
}
