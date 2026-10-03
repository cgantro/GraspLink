#type vertex
#version 330 core

layout(location = 0) in vec3 a_Position;
layout(location = 1) in vec3 a_Normal;
layout(location = 2) in vec2 a_TexCoord;

uniform mat4 u_Model;
uniform mat4 u_View;
uniform mat4 u_Projection;

out vec3 v_Normal;
out vec3 v_WorldPosition;

void main()
{
    vec4 worldPosition =
        u_Model * vec4(a_Position, 1.0);

    v_WorldPosition =
        worldPosition.xyz;

    v_Normal =
        mat3(transpose(inverse(u_Model))) *
        a_Normal;

    gl_Position =
        u_Projection *
        u_View *
        worldPosition;
}


#type fragment
#version 330 core

in vec3 v_Normal;
in vec3 v_WorldPosition;

uniform vec3 u_CameraPosition;

uniform vec4 u_BaseColorFactor;
uniform float u_MetallicFactor;
uniform float u_RoughnessFactor;

out vec4 FragColor;


// ------------------------------------------------------------
// 간단한 ACES 계열 Tone Mapping
// ------------------------------------------------------------

vec3 ToneMapACES(vec3 color)
{
    float a = 2.51;
    float b = 0.03;
    float c = 2.43;
    float d = 0.59;
    float e = 0.14;

    return clamp(
        (color * (a * color + b)) /
        (color * (c * color + d) + e),
        0.0,
        1.0);
}


void main()
{
    vec3 N =
        normalize(v_Normal);

    vec3 V =
        normalize(
            u_CameraPosition -
            v_WorldPosition);


    // --------------------------------------------------------
    // Main directional light
    // surface -> light 방향
    // --------------------------------------------------------

    vec3 L =
        normalize(
            vec3(-0.45, 0.85, 0.35));

    float NdotL =
        max(
            dot(N, L),
            0.0);


    // --------------------------------------------------------
    // Hemisphere ambient
    //
    // 위쪽 면은 약간 밝게,
    // 아래쪽 면은 약간 어둡게.
    // 상수 ambient보다 입체감이 훨씬 좋다.
    // --------------------------------------------------------

    float upFactor =
        N.y * 0.5 + 0.5;

    vec3 groundAmbient =
        vec3(0.035, 0.04, 0.045);

    vec3 skyAmbient =
        vec3(0.16, 0.18, 0.20);

    vec3 ambientLight =
        mix(
            groundAmbient,
            skyAmbient,
            upFactor);


    // --------------------------------------------------------
    // Material
    // --------------------------------------------------------

    vec3 baseColor =
        u_BaseColorFactor.rgb;

    float metallic =
        clamp(
            u_MetallicFactor,
            0.0,
            1.0);

    float roughness =
        clamp(
            u_RoughnessFactor,
            0.05,
            1.0);


    // metallic 재질은 diffuse가 감소한다.
    vec3 diffuseColor =
        baseColor *
        (1.0 - metallic);


    // --------------------------------------------------------
    // Blinn-Phong 기반 임시 Specular
    //
    // 완전한 PBR은 아니지만
    // 현재 Debug Shader보다 훨씬 보기 좋다.
    // --------------------------------------------------------

    vec3 H =
        normalize(L + V);

    float NdotH =
        max(
            dot(N, H),
            0.0);

    float shininess =
        mix(
            128.0,
            8.0,
            roughness);

    float specularStrength =
        pow(
            NdotH,
            shininess);


    // 비금속 F0 ≈ 0.04
    // 금속은 baseColor가 반사색에 영향을 준다.
    vec3 F0 =
        mix(
            vec3(0.04),
            baseColor,
            metallic);

    vec3 specular =
        F0 *
        specularStrength;


    // --------------------------------------------------------
    // Main Light
    // --------------------------------------------------------

    vec3 directLight =
        (
            diffuseColor * NdotL +
            specular
        ) * 1.7;


    // --------------------------------------------------------
    // 약한 Fill Light
    // --------------------------------------------------------

    vec3 fillDirection =
        normalize(
            vec3(0.65, 0.35, -0.65));

    float fillAmount =
        max(
            dot(N, fillDirection),
            0.0);

    vec3 fillLight =
        diffuseColor *
        fillAmount *
        0.18;


    // --------------------------------------------------------
    // Final
    // --------------------------------------------------------

    vec3 color =
        baseColor * ambientLight +
        directLight +
        fillLight;


    // HDR -> display range
    color =
        ToneMapACES(color);


    // 현재 framebuffer에서 수동 Gamma Correction
    color =
        pow(
            color,
            vec3(1.0 / 2.2));


    FragColor =
        vec4(
            color,
            u_BaseColorFactor.a);
}