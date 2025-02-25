#version 330 core
in vec3 FragPos;
out vec4 FragColor;

uniform vec3 cameraPos;

void main() {
    float gridSize = 1.0;        // Size of each grid cell
    float lineThickness = 0.02;  // Grid line thickness

    // Calculate grid lines using world coordinates
    vec2 gridUV = FragPos.xz / gridSize;
    vec2 gridLine = abs(fract(gridUV - 0.5) - 0.5) / fwidth(gridUV);

    float grid = min(gridLine.x, gridLine.y);
    float gridIntensity = 1.0 - smoothstep(0.0, lineThickness, grid);

    // Grid color (white lines on dark background)
    vec3 gridColor = mix(vec3(0.1), vec3(1.0), gridIntensity);

    // Apply fade effect based on distance from camera
    float fade = exp(-length(FragPos.xz - cameraPos.xz) * 0.02);
    FragColor = vec4(gridColor * fade, 1.0);
}
