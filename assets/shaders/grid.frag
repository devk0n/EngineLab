#version 460 core

out vec4 FragColor;

uniform mat4 view;
uniform mat4 projection;
uniform mat4 invVP;
uniform vec3 cameraPos;

in vec2 vPos;

void main() {
    // Calculate world position from NDC
    vec4 clipPos = vec4(vPos, 1.0, 1.0);
    vec4 worldPos = invVP * clipPos;
    worldPos /= worldPos.w;

    // Ray direction from camera to fragment
    vec3 rayDir = normalize(worldPos.xyz - cameraPos);

    // Intersect with XY plane (Z = 0)
    if (rayDir.z == 0.0) discard;
    float t = -cameraPos.z / rayDir.z;
    if (t < 0.0) discard;
    vec3 hitPoint = cameraPos + t * rayDir;

    // Grid parameters
    float majorGrid = 10.0;
    float minorGrid = 1.0;
    float minorWidth = 0.02;
    float majorWidth = 0.05;

     // Grid parameters (now using XY coordinates)
    vec2 coord = hitPoint.xy;
    vec2 minor = abs(fract(coord / minorGrid - 0.5) - 0.5);
    float minorLine = smoothstep(minorWidth, minorWidth + 0.001, min(minor.x, minor.y));

    // Major grid lines
    vec2 major = abs(fract(coord / majorGrid - 0.5) - 0.5);
    float majorLine = smoothstep(majorWidth, majorWidth + 0.001, min(major.x, major.y));

    // Combine lines
    float grid = max(minorLine, majorLine);

    // Fade with distance
    float distance = length(hitPoint - cameraPos);
    float fade = 1.0 - smoothstep(10.0, 100.0, distance);

    // Color and alpha
    vec3 color = vec3(0.5) * grid * fade;
    float alpha = grid * fade;

    FragColor = vec4(color, alpha * 0.5);
}