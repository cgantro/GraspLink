
#type vertex
#version 330 core

// 입력 위치와 법선은 Mesh의 vertex layout, UV는 base-color texture의 좌표다.

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
    // Local → World: 조명 계산은 모든 위치와 광원 행렬을 같은 World 좌표에서 수행한다.
    vec4 worldPosition = u_Model * vec4(a_Position, 1.0);

    v_WorldPosition = worldPosition.xyz;

    // 법선은 방향 벡터지만 비균일 크기 변환에서 표면에 수직인 관계를 잃으므로 모델 행렬의 역전치를 쓴다.
    v_Normal = mat3(transpose(inverse(u_Model))) * a_Normal;

    v_LightSpacePosition = u_LightSpaceMatrix * worldPosition;
    v_TexCoord = a_TexCoord;

    // 좌표 변환: Local → World → camera(View) → clip(Projection). 마지막 clip 좌표는 래스터화에 사용된다.
    gl_Position = u_Projection * u_View * worldPosition;
}

#type fragment
#version 330 core

// vertex stage에서 보간된 World 값으로 표면 조명과 광원 깊이 비교를 수행한다.

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

    // 조명 합산으로 1을 넘은 선형 RGB를 화면 범위에 부드럽게 압축한다.
    return clamp(
        (color * (a * color + b)) /
        (color * (c * color + d) + e),
        0.0,
        1.0
    );
}

float CalculateShadow(vec4 lightSpacePosition, vec3 normal, vec3 lightDirection)
{
    // 광원 Clip 좌표를 texture 좌표·깊이의 0..1 범위로 옮겨 저장된 표면 깊이와 비교한다.
    vec3 projCoords = lightSpacePosition.xyz / lightSpacePosition.w;

    projCoords = projCoords * 0.5 + 0.5;

    if (projCoords.z > 1.0)
        return 0.0;

    if (projCoords.x < 0.0 || projCoords.x > 1.0 ||
        projCoords.y < 0.0 || projCoords.y > 1.0)
        return 0.0;

    float currentDepth = projCoords.z;

    // u_LightDirection은 표면에서 광원 쪽을 향한다. 비스듬한 입사면일수록 bias를 키워 acne를 줄인다.
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

    // 5x5 depth 비교 평균: 0=빛, 1=완전한 그림자.
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

    // 제한: metallic/roughness는 material factor만 사용한다. 해당 채널 texture와 완전한 glTF PBR BRDF는 없다.
    float metallic = clamp(u_MetallicFactor, 0.0, 1.0);
    float roughness = clamp(u_RoughnessFactor, 0.05, 1.0);

    // 금속도 완전히 검게 사라지지 않도록 diffuse를 35%까지 남기는 간략 근사다.
    vec3 diffuseColor = baseColor * mix(1.0, 0.35, metallic);

    // Blinn-Phong 형태의 간단한 정반사 항. roughness가 클수록 지수와 하이라이트가 낮아진다.
    vec3 H = normalize(L + V);
    float NdotH = max(dot(N, H), 0.0);

    float shininess = mix(48.0, 5.0, roughness);
    float specularStrength = pow(NdotH, shininess);

    if (NdotL <= 0.0)
        specularStrength = 0.0;

    // 정면 반사율 F0를 비금속의 0.04에서 금속의 baseColor로 보간한다.
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

    // 선형 조명 결과를 ACES 곡선으로 범위 압축한 뒤 디스플레이 감마 공간으로 변환한다.
    color = ToneMapACES(color);

    color = pow(color, vec3(1.0 / 2.2));

    FragColor =vec4(color,baseAlpha);
}
