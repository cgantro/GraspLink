
#type vertex
#version 330 core

layout(location = 0) in vec3 a_Position;

uniform mat4 u_Model;
uniform mat4 u_View;
uniform mat4 u_Projection;
uniform mat4 u_LightSpaceMatrix;

out vec3 v_WorldPosition;
out vec4 v_LightSpacePosition;

void main()
{
    vec4 worldPosition = u_Model * vec4(a_Position, 1.0);

    v_WorldPosition = worldPosition.xyz;
    v_LightSpacePosition = u_LightSpaceMatrix * worldPosition;

    // 좌표 변환: World → camera → clip.
    gl_Position = u_Projection * u_View * worldPosition;
}

#type fragment
#version 330 core

in vec3 v_WorldPosition;
in vec4 v_LightSpacePosition;

uniform sampler2D u_ShadowMap;

out vec4 FragColor;

float GridLine(vec2 position, float scale)
{
    vec2 coord = position / scale;

    // 한 화면 pixel이 차지하는 World 간격으로 선 폭을 맞춰, 먼 격자의 깜빡임을 줄인다.
    vec2 derivative = fwidth(coord);

    vec2 grid =
        abs(fract(coord - 0.5) - 0.5) /
        max(derivative, vec2(0.00001));

    float line = min(grid.x, grid.y);

    return 1.0 - min(line, 1.0);
}

float CalculateShadow(vec4 lightSpacePosition)
{
    // 광원 Clip 좌표를 w로 나눠 texture 좌표와 저장된 깊이의 0..1 범위에 맞춘다.
    vec3 projCoords = lightSpacePosition.xyz / lightSpacePosition.w;

    projCoords = projCoords * 0.5 + 0.5;

    if (projCoords.z > 1.0)
        return 0.0;

    if (projCoords.x < 0.0 || projCoords.x > 1.0 ||
        projCoords.y < 0.0 || projCoords.y > 1.0)
        return 0.0;

    float currentDepth = projCoords.z;

    // 정규화된 깊이를 조금 당겨 동일 표면의 정밀도 오차로 생기는 자기 그림자를 줄인다.
    float bias = 0.0003;

    vec2 texelSize = 1.0 / vec2(textureSize(u_ShadowMap, 0));

    float shadow = 0.0;

    for (int x = -2; x <= 2; ++x)
    {
        for (int y = -2; y <= 2; ++y)
        {
            float closestDepth = texture(
                u_ShadowMap,
                projCoords.xy + vec2(x, y) * texelSize
            ).r;

            if (currentDepth - bias > closestDepth)
                shadow += 1.0;
        }
    }

    // 5x5 depth 비교 평균: 0=빛, 1=완전한 그림자.
    return shadow / 25.0;
}

void main()
{
    vec2 position = v_WorldPosition.xz;

    // World XZ 간격 [m]: 보조선 0.10, 주선 0.50.
    float minor = GridLine(position, 0.10);
    float major = GridLine(position, 0.50);

    vec3 baseColor = vec3(0.09);
    vec3 minorColor = vec3(0.16);
    vec3 majorColor = vec3(0.28);

    vec3 color = mix(
        baseColor,
        minorColor,
        minor * 0.6
    );

    color = mix(
        color,
        majorColor,
        major
    );

    float xAxis = 1.0 - smoothstep(
        0.0,
        0.008,
        abs(v_WorldPosition.z)
    );

    float zAxis = 1.0 - smoothstep(
        0.0,
        0.008,
        abs(v_WorldPosition.x)
    );

    color = mix(
        color,
        vec3(0.55, 0.10, 0.08),
        xAxis
    );

    color = mix(
        color,
        vec3(0.08, 0.20, 0.55),
        zAxis
    );

    float shadow = CalculateShadow(v_LightSpacePosition);

    color *= mix(1.0,0.68,shadow);

    color = pow(color, vec3(1.0 / 2.2));

    FragColor = vec4(color, 1.0);
}
