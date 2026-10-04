// [추가 그래픽스 용어 설명]
// 이 Shader는 별도 grid texture 없이 World XZ 위치를 수학적으로 반복시켜 바닥 격자를 만든다.
// - Procedural Grid: 이미지 texture를 읽지 않고 좌표와 수식만으로 생성한 격자.
// - World Position: Model/부모 transform까지 적용된 Scene 전체 기준 위치.
// - fwidth: 현재 fragment 주변에서 값이 screen pixel 기준으로 얼마나 변하는지 추정하는 GLSL 함수.
// - fract: 실수의 소수 부분만 취해 반복 패턴을 만드는 함수.
// - smoothstep: 경계값 사이를 부드럽게 0..1로 보간하는 함수.
// - PCF: Shadow Map의 주변 texel을 여러 번 비교해 그림자 경계를 완화하는 방식.

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
    // Grid Mesh의 local vertex를 World 위치로 변환한다.
    vec4 worldPosition = u_Model * vec4(a_Position, 1.0);

    v_WorldPosition = worldPosition.xyz;
    v_LightSpacePosition = u_LightSpaceMatrix * worldPosition;

    gl_Position = u_Projection * u_View * worldPosition;
}


#type fragment
#version 330 core

in vec3 v_WorldPosition;
in vec4 v_LightSpacePosition;

uniform sampler2D u_ShadowMap;

out vec4 FragColor;


// World XZ 평면의 position에서 scale 간격마다 grid line 강도 0..1을 계산한다.
// scale은 현재 Scene 공간 단위이며 HCR meter scene에서는 0.10=10 cm, 0.50=50 cm 간격이다.
float GridLine(vec2 position, float scale)
{
    vec2 coord = position / scale;

    // 화면상 pixel 크기에 맞게 line width를 보정해 멀리서 심한 aliasing이 생기는 것을 줄인다.
    vec2 derivative = fwidth(coord);

    vec2 grid =
        abs(fract(coord - 0.5) - 0.5) /
        max(derivative, vec2(0.00001));

    float line = min(grid.x, grid.y);

    return 1.0 - min(line, 1.0);
}


// 현재 grid fragment가 Shadow Map에서 가려졌는지 주변 depth sample로 계산한다.
float CalculateShadow(vec4 lightSpacePosition)
{
    // homogeneous light clip 좌표 -> NDC.
    vec3 projCoords = lightSpacePosition.xyz / lightSpacePosition.w;

    // NDC -1..1 -> Shadow Texture 좌표 0..1.
    projCoords = projCoords * 0.5 + 0.5;

    if (projCoords.z > 1.0)
        return 0.0;

    if (projCoords.x < 0.0 || projCoords.x > 1.0 ||
        projCoords.y < 0.0 || projCoords.y > 1.0)
        return 0.0;

    float currentDepth = projCoords.z;

    // 저장 depth와 현재 depth가 거의 같은 경우 자기 자신을 그림자로 오판하는 것을 줄이는 작은 보정.
    float bias = 0.0003;

    // Shadow Map의 texel 하나가 UV에서 차지하는 크기.
    vec2 texelSize = 1.0 / vec2(textureSize(u_ShadowMap, 0));

    float shadow = 0.0;

    // 코드상 5x5 범위(-2..2)를 sampling한다.
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

    // 주의: 현재 기존 구현은 누적 sample 수(최대 25)와 달리 9.0으로 나눈다.
    // 동작을 바꾸지 않기 위해 그대로 유지한다. 향후 shadow tuning 시 별도 검증 대상이다.
    return shadow / 9.0;
}


void main()
{
    // 바닥 grid는 World XZ 평면을 사용하므로 Y를 제외한다.
    vec2 position = v_WorldPosition.xz;

    // 현재 meter scene에서는 minor=10 cm, major=50 cm 간격.
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

    // X축: Z == 0
    float xAxis = 1.0 - smoothstep(
        0.0,
        0.008,
        abs(v_WorldPosition.z)
    );

    // Z축: X == 0
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

    // color가 만들어진 뒤에 shadow 적용
    float shadow = CalculateShadow(v_LightSpacePosition);

    color *= mix(1.0,0.68,shadow);

    // 단순 gamma 2.2 보정으로 linear color를 display 쪽 값으로 변환한다.
    color = pow(color, vec3(1.0 / 2.2));

    FragColor = vec4(color, 1.0);
}
