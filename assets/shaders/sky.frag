#version 460 core
out vec4 FragColor;
in vec3 TexCoords;

uniform vec3 sunDir = normalize(vec3(0.2, -1.0, 0.1));

void main() {
    // Use Z coordinate for vertical gradient
    float vertical = TexCoords.z; // Changed from .y to .z
    vec3 skyColor = mix(vec3(0.5, 0.7, 1.0), vec3(0.1, 0.1, 0.3),
         smoothstep(-0.2, 0.8, vertical));

    // Sun calculation (adjust for Z-up)
    vec3 rayDir = normalize(TexCoords);
    vec3 sunDir = normalize(vec3(0.2, 0.1, -1.0)); // Sun coming from +X, +Y, -Z
    float sun = smoothstep(0.999, 0.9997, dot(rayDir, sunDir));

    FragColor = vec4(skyColor + sun * vec3(1.0, 0.8, 0.6), 1.0);
}