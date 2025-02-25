#version 330 core
layout(location = 0) in vec3 aPos; // Vertex position

// Uniforms for transformation
uniform mat4 projection;
uniform mat4 view;

out vec3 FragPos; // Pass world-space position to fragment shader

void main()
{
    FragPos = aPos;  // Store world position for fragment calculations
    gl_Position = projection * view * vec4(aPos, 1.0);
}
