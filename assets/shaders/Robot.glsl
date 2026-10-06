
#type vertex
#version 330 core

// Mesh는 삼각형 정점과 연결 번호를 모은 형상이다. 정점에서 위치와 표면 수직 방향(법선)을 읽고 UV 이미지 좌표로 기본색 Texture의 픽셀을 찾는다.

layout(location = 0) in vec3 a_Position;
layout(location = 1) in vec3 a_Normal;
layout(location = 2) in vec2 a_TexCoord;

uniform mat4 u_Model;
uniform mat4 u_View;
uniform mat4 u_Projection;
uniform mat4 u_LightSpaceMatrix;

out vec3 v_Normal;
out vec3 v_WorldPosition;
out vec4 v_LightSpacePosition;
out vec2 v_TexCoord;

void main()
{
    // Local 좌표는 Mesh의 기준점에서 잰 위치이고 World 좌표는 장면 전체 기준 위치다. 모델 행렬로 Local을 World로 바꿔 표면과 광원을 같은 기준에서 비교한다.
    vec4 worldPosition = u_Model * vec4(a_Position, 1.0);

    v_WorldPosition = worldPosition.xyz;

    // 크기 배율이 축마다 다르면 방향만 모델 행렬로 바꾼 법선은 표면에 수직이 아니게 된다.
    // 역전치 행렬을 적용해 변환 뒤에도 표면에 수직인 방향을 유지한다.
    v_Normal = mat3(transpose(inverse(u_Model))) * a_Normal;

    v_LightSpacePosition = u_LightSpaceMatrix * worldPosition;
    v_TexCoord = a_TexCoord;

    // 좌표 변환은 점의 기준을 바꾸는 계산이다. Local에서 World, Camera, Clip 순서로 바꾸며 Camera 좌표는 카메라가 원점인 기준이고 Clip 좌표는 화면 자르기와 깊이 계산에 쓰이는 중간값이다.
    // Projection 원근 계산으로 먼 표면은 작아 보이고 Clip 좌표를 w로 나눈 뒤 GPU가 화면 위치와 앞뒤 깊이를 정한다.
    gl_Position = u_Projection * u_View * worldPosition;
}

#type fragment
#version 330 core

// GPU는 삼각형 안의 정점값을 픽셀 위치에 맞게 보간한다. 이 단계는 보간한 World 위치와 표면 방향으로 빛과 그림자를 계산한다.

in vec3 v_Normal;
in vec3 v_WorldPosition;
in vec4 v_LightSpacePosition;
in vec2 v_TexCoord;

uniform vec3 u_CameraPosition;
uniform vec3 u_LightDirection;

uniform vec4 u_BaseColorFactor;
uniform float u_MetallicFactor;
uniform float u_RoughnessFactor;
uniform sampler2D u_BaseColorTexture;
uniform int u_UseBaseColorTexture;

uniform sampler2D u_ShadowMap;

out vec4 FragColor;

vec3 ToneMapACES(vec3 color)
{
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;

    // 여러 조명을 더해 1보다 커진 선형 RGB를 화면에 표시할 수 있는 0~1 범위로 부드럽게 줄인다.
    return clamp(
        (color * (a * color + b)) /
        (color * (c * color + d) + e),
        0.0,
        1.0
    );
}

float CalculateShadow(vec4 lightSpacePosition, vec3 normal, vec3 lightDirection)
{
    // 광원 기준 Clip 좌표를 w로 나누고 0~1 범위로 바꿔 그림자 이미지 위치와 투영 깊이로 사용한다. 이 깊이는 실제 광원 거리값이 아니다.
    vec3 projCoords = lightSpacePosition.xyz / lightSpacePosition.w;

    projCoords = projCoords * 0.5 + 0.5;

    if (projCoords.z > 1.0)
        return 0.0;

    if (projCoords.x < 0.0 || projCoords.x > 1.0 ||
        projCoords.y < 0.0 || projCoords.y > 1.0)
        return 0.0;

    float currentDepth = projCoords.z;

    // 표면 깊이에 작은 여유값을 둔다. 광선을 비스듬히 받는 면은 깊이 오차가 커져 얼룩 그림자가 생기기 쉬우므로 여유를 늘린다.
    float bias = max(
        0.0008 * (1.0 - dot(normal, lightDirection)),
        0.00015
    );

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

    // 표면 주변 5×5 위치에서 가려진 비율을 평균한다. 0은 빛을 받음, 1은 완전히 가려짐이다.
    return shadow / 25.0;
}

void main()
{
    vec3 N = normalize(v_Normal);
    vec3 V = normalize(u_CameraPosition - v_WorldPosition);
    vec3 L = normalize(u_LightDirection);

    float NdotL = max(dot(N, L), 0.0);

    float upFactor = N.y * 0.5 + 0.5;

    vec3 groundAmbient = vec3(0.10, 0.105, 0.11);
    vec3 skyAmbient = vec3(0.24, 0.25, 0.27);
    vec3 ambientLight = mix(groundAmbient, skyAmbient, upFactor);

    vec4 sampledBaseColor = vec4(1.0);

    if (u_UseBaseColorTexture == 1)
    {
        sampledBaseColor = texture(u_BaseColorTexture,v_TexCoord);
    }

    vec3 baseColor = sampledBaseColor.rgb * u_BaseColorFactor.rgb;

    float baseAlpha = sampledBaseColor.a * u_BaseColorFactor.a;

    // 금속성·거칠기 이미지는 연결하지 않으며 glTF가 정한 전체 물리 기반 반사식도 구현하지 않았다.
    // 재질에 저장된 두 숫자만 읽어 아래의 간단한 빛 계산에 사용한다.
    float metallic = clamp(u_MetallicFactor, 0.0, 1.0);
    float roughness = clamp(u_RoughnessFactor, 0.05, 1.0);

    // 금속 계수가 커져도 표면 색이 모두 사라지지 않도록 기본색 조명을 최대 35% 남기는 단순 근사다.
    vec3 diffuseColor = baseColor * mix(1.0, 0.35, metallic);

    // 표면에서 빛과 카메라를 향하는 두 방향의 중간을 이용해 반짝임을 계산한다. 거칠수록 넓고 약해진다.
    vec3 H = normalize(L + V);
    float NdotH = max(dot(N, H), 0.0);

    float shininess = mix(48.0, 5.0, roughness);
    float specularStrength = pow(NdotH, shininess);

    if (NdotL <= 0.0)
        specularStrength = 0.0;

    // 표면을 정면에서 볼 때의 반사색을 비금속의 회색 값 0.04와 금속의 기본색 사이에서 정한다.
    vec3 F0 = mix(vec3(0.04), baseColor, metallic);
    vec3 specular = F0 * specularStrength * 0.35;

    float shadow = CalculateShadow(v_LightSpacePosition, N, L);

    vec3 directLight =(diffuseColor * NdotL + specular) *1.35 *(1.0 - shadow * 0.65);

    vec3 fillDirection = normalize(vec3(0.65, 0.35, -0.65));
    float fillAmount = max(dot(N, fillDirection), 0.0);
    vec3 fillLight = diffuseColor * fillAmount * 0.42;

    vec3 color =
        baseColor * ambientLight +
        directLight +
        fillLight;

    // ACES 곡선은 조명 결과를 화면 밝기 범위로 압축한다. 이어 1/2.2 거듭제곱 감마 보정으로 모니터 표시용 RGB 값에 맞춘다.
    color = ToneMapACES(color);

    color = pow(color, vec3(1.0 / 2.2));

    FragColor =vec4(color,baseAlpha);
}
