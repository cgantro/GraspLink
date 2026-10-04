// [추가 그래픽스 용어 설명]
// 이 파일은 하나의 .glsl 안에 vertex/fragment shader를 함께 두고 #type으로 구분하는 프로젝트 형식이다.
// - Vertex Shader: Mesh의 각 정점을 Model -> World -> View -> Projection 순서로 변환한다.
// - Fragment Shader: rasterization 뒤 생성된 fragment마다 조명/Material/Shadow를 계산해 최종 색을 만든다.
// - Attribute: Mesh가 정점마다 제공하는 입력값. location 0=position, 1=normal, 2=UV.
// - Uniform: 한 draw 동안 여러 정점/fragment가 공통으로 읽는 CPU에서 전달한 값.
// - Varying(out/in): Vertex Shader가 계산해 Fragment Shader로 보간되어 전달되는 값.
// - Model Matrix: Mesh local 좌표 -> World 좌표.
// - View Matrix: World 좌표 -> Camera 좌표.
// - Projection Matrix: Camera 좌표 -> clip 좌표.
// - Normal Matrix: non-uniform scale이 있어도 normal 방향을 올바르게 변환하기 위한 inverse-transpose 행렬.
// - Light-space: Shadow Map을 만든 광원 관점의 좌표계.

#type vertex
#version 330 core

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
    // Local vertex position을 World 공간으로 변환한다.
    vec4 worldPosition = u_Model * vec4(a_Position, 1.0);

    v_WorldPosition = worldPosition.xyz;

    // Normal은 위치처럼 translation을 적용하면 안 되므로 model matrix의 inverse-transpose 3x3을 사용한다.
    v_Normal = mat3(transpose(inverse(u_Model))) * a_Normal;

    // 같은 World 위치를 light 관점으로 변환해 fragment 단계의 shadow 비교에 사용한다.
    v_LightSpacePosition = u_LightSpaceMatrix * worldPosition;
    v_TexCoord = a_TexCoord;

    // OpenGL이 rasterization에 사용할 최종 clip-space 위치.
    gl_Position = u_Projection * u_View * worldPosition;
}


#type fragment
#version 330 core

// [추가 fragment 용어]
// - N: 표면 normal 방향.
// - V: 표면에서 camera를 향하는 view 방향.
// - L: 표면에서 light를 향하는 방향으로 사용하는 조명 방향.
// - H: L과 V 사이의 half vector. Blinn-Phong specular 계산에 사용한다.
// - dot(a,b): 두 단위벡터가 얼마나 같은 방향인지 나타내는 내적(-1..1).
// - PCF: 주변 shadow texel 여러 개를 비교해 그림자 경계를 부드럽게 하는 필터링.
// - Bias: 자기 자신을 shadow로 잘못 판단하는 shadow acne를 줄이기 위한 작은 depth 보정값.
// - Tone Mapping: HDR처럼 큰 밝기 범위를 화면 0..1 범위로 압축하는 과정.
// - Gamma correction: linear lighting 결과를 모니터 표시 특성에 맞는 값으로 변환하는 과정.

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


// ACES 계열 근사식을 사용해 큰 linear color 값을 표시 가능한 0..1 범위로 압축한다.
vec3 ToneMapACES(vec3 color)
{
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;

    return clamp(
        (color * (a * color + b)) /
        (color * (c * color + d) + e),
        0.0,
        1.0
    );
}


// Light-space 위치를 Shadow Map UV/depth로 바꾸고 5x5 PCF 비교 결과를 0(no shadow)..1(shadow)로 반환한다.
float CalculateShadow(vec4 lightSpacePosition, vec3 normal, vec3 lightDirection)
{
    // Perspective divide: homogeneous clip 좌표를 NDC 좌표로 바꾼다.
    vec3 projCoords = lightSpacePosition.xyz / lightSpacePosition.w;

    // OpenGL NDC -1..1 범위를 texture UV/depth 0..1 범위로 매핑한다.
    projCoords = projCoords * 0.5 + 0.5;

    if (projCoords.z > 1.0)
        return 0.0;

    if (projCoords.x < 0.0 || projCoords.x > 1.0 ||
        projCoords.y < 0.0 || projCoords.y > 1.0)
        return 0.0;

    float currentDepth = projCoords.z;

    // 표면이 light와 비스듬할수록 조금 더 큰 bias를 사용해 self-shadow artifact를 줄인다.
    float bias = max(
        0.0008 * (1.0 - dot(normal, lightDirection)),
        0.00015
    );

    // Shadow texture에서 정확히 한 texel만큼 이동하기 위한 UV 크기.
    vec2 texelSize = 1.0 / vec2(textureSize(u_ShadowMap, 0));

    float shadow = 0.0;

    // 현재 pixel 주변 5x5=25개 depth sample을 비교하는 PCF.
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

    return shadow / 25.0;
}


void main()
{
    // 보간된 방향벡터는 길이가 1이라는 보장이 없으므로 조명 계산 전에 normalize한다.
    vec3 N = normalize(v_Normal);
    vec3 V = normalize(u_CameraPosition - v_WorldPosition);
    vec3 L = normalize(u_LightDirection);

    // Lambert diffuse의 핵심 값. 표면이 light를 정면으로 볼수록 1에 가까워진다.
    float NdotL = max(dot(N, L), 0.0);

    // Hemisphere ambient
    float upFactor = N.y * 0.5 + 0.5;

    vec3 groundAmbient = vec3(0.10, 0.105, 0.11);
    vec3 skyAmbient = vec3(0.24, 0.25, 0.27);
    vec3 ambientLight = mix(groundAmbient, skyAmbient, upFactor);

    // Material

    vec4 sampledBaseColor = vec4(1.0);

    if (u_UseBaseColorTexture == 1)
    {
        sampledBaseColor = texture(u_BaseColorTexture,v_TexCoord);
    }

    // Texture 색과 Material factor를 곱해 실제 base color를 만든다.
    vec3 baseColor = sampledBaseColor.rgb * u_BaseColorFactor.rgb;

    float baseAlpha = sampledBaseColor.a * u_BaseColorFactor.a;

    // 외부에서 이상한 값이 들어와도 현재 lighting 식의 기대 범위 안에서 사용한다.
    float metallic = clamp(u_MetallicFactor, 0.0, 1.0);
    float roughness = clamp(u_RoughnessFactor, 0.05, 1.0);

    vec3 diffuseColor = baseColor * mix(1.0, 0.35, metallic);

    // Blinn-Phong specular
    vec3 H = normalize(L + V);
    float NdotH = max(dot(N, H), 0.0);

    // roughness가 커질수록 highlight가 넓고 흐려지도록 shininess를 낮춘다.
    float shininess = mix(48.0, 5.0, roughness);
    float specularStrength = pow(NdotH, shininess);

    if (NdotL <= 0.0)
        specularStrength = 0.0;

    // 비금속은 약 4% 반사색을, 금속은 baseColor 쪽을 F0로 사용하도록 보간한다.
    vec3 F0 = mix(vec3(0.04), baseColor, metallic);
    vec3 specular = F0 * specularStrength * 0.35;

    // Shadow
    float shadow = CalculateShadow(v_LightSpacePosition, N, L);

    vec3 directLight =(diffuseColor * NdotL + specular) *1.35 *(1.0 - shadow * 0.65);

    // 반대쪽 보조광
    vec3 fillDirection = normalize(vec3(0.65, 0.35, -0.65));
    float fillAmount = max(dot(N, fillDirection), 0.0);
    vec3 fillLight = diffuseColor * fillAmount * 0.42;

    vec3 color =
        baseColor * ambientLight +
        directLight +
        fillLight;

    color = ToneMapACES(color);

    // 단순 gamma 2.2 보정: linear color -> display 쪽 값.
    color = pow(color, vec3(1.0 / 2.2));

    FragColor =vec4(color,baseAlpha);
}
