#version 330 core
out vec4 FragColor;
uniform vec3 cameraPos;

void main() {
    vec2 worldPos = gl_FragCoord.xy;
    float majorLine = 10.0;
    float superMajorLine = 100.0;

    float lineWidth = 0.01;
    float majorLineWidth = 0.02;
    float superMajorLineWidth = 0.05;

    float gridX = mod(worldPos.x, 1.0) < lineWidth ? 1.0 : 0.0;
    float gridY = mod(worldPos.y, 1.0) < lineWidth ? 1.0 : 0.0;
    float majorGridX = mod(worldPos.x, majorLine) < majorLineWidth ? 1.0 : 0.0;
    float majorGridY = mod(worldPos.y, majorLine) < majorLineWidth ? 1.0 : 0.0;
    float superMajorGridX = mod(worldPos.x, superMajorLine) < superMajorLineWidth ? 1.0 : 0.0;
    float superMajorGridY = mod(worldPos.y, superMajorLine) < superMajorLineWidth ? 1.0 : 0.0;

    float gridIntensity = gridX + gridY;
    float majorGridIntensity = majorGridX + majorGridY;
    float superMajorGridIntensity = superMajorGridX + superMajorGridY;

    vec3 color = vec3(0.5, 0.5, 0.5) * gridIntensity;
    color += vec3(0.8, 0.8, 0.8) * majorGridIntensity;
    color += vec3(1.0, 1.0, 1.0) * superMajorGridIntensity;

    FragColor = vec4(color, 1.0);
}
