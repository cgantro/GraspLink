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
    vec4 worldPosition = u_Model * vec4(a_Position, 1.0);

    v_WorldPosition = worldPosition.xyz;
    v_Normal = mat3(transpose(inverse(u_Model))) * a_Normal;
    v_LightSpacePosition = u_LightSpaceMatrix * worldPosition;
    v_TexCoord = a_TexCoord;
    gl_Position = u_Projection * u_View * worldPosition;
}


#type fragment
#version 330 core

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

    return clamp(
        (color * (a * color + b)) /
        (color * (c * color + d) + e),
        0.0,
        1.0
    );
}


float CalculateShadow(vec4 lightSpacePosition, vec3 normal, vec3 lightDirection)
{
    vec3 projCoords = lightSpacePosition.xyz / lightSpacePosition.w;
    projCoords = projCoords * 0.5 + 0.5;

    if (projCoords.z > 1.0)
        return 0.0;

    if (projCoords.x < 0.0 || projCoords.x > 1.0 ||
        projCoords.y < 0.0 || projCoords.y > 1.0)
        return 0.0;

    float currentDepth = projCoords.z;

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

    return shadow / 25.0;
}


void main()
{
    vec3 N = normalize(v_Normal);
    vec3 V = normalize(u_CameraPosition - v_WorldPosition);
    vec3 L = normalize(u_LightDirection);

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

    vec3 baseColor = sampledBaseColor.rgb * u_BaseColorFactor.rgb;

    float baseAlpha = sampledBaseColor.a * u_BaseColorFactor.a;

    float metallic = clamp(u_MetallicFactor, 0.0, 1.0);
    float roughness = clamp(u_RoughnessFactor, 0.05, 1.0);

    vec3 diffuseColor = baseColor * mix(1.0, 0.35, metallic);

    // Blinn-Phong specular
    vec3 H = normalize(L + V);
    float NdotH = max(dot(N, H), 0.0);

    float shininess = mix(48.0, 5.0, roughness);
    float specularStrength = pow(NdotH, shininess);

    if (NdotL <= 0.0)
        specularStrength = 0.0;

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
    color = pow(color, vec3(1.0 / 2.2));

    FragColor =vec4(color,baseAlpha);
}