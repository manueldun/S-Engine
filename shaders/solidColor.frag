#version 450

layout(location = 0) out vec4 outColor;

layout(binding = 0, set = 1) uniform colorUniform { vec3 color; }
color;

void main() { outColor = vec4(color.color, 1.0f); }
