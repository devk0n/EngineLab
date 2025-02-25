#version 330 core
out vec4 FragColor;
uniform vec3 cameraPos;

void main() {
    vec3 gridColor = vec3(0.8, 0.8, 0.8);
    FragColor = vec4(gridColor, 1.0);
}
