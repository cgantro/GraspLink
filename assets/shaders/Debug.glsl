#type vertex
#version 330 core

layout(location = 0) in vec3 a_Position;
layout(location = 1) in vec3 a_Normal;
layout(location = 2) in vec2 a_TexCoord;

uniform mat4 u_Model;
uniform mat4 u_View;
uniform mat4 u_Projection;

out vec3 v_Normal;

void main()
{
    v_Normal =
        mat3(transpose(inverse(u_Model))) *
        a_Normal;

    gl_Position =
        u_Projection *
        u_View *
        u_Model *
        vec4(a_Position, 1.0);
}


#type fragment
#version 330 core

in vec3 v_Normal;

uniform vec4 u_BaseColorFactor;
uniform float u_MetallicFactor;
uniform float u_RoughnessFactor;

out vec4 FragColor;

void main()
{
    vec3 N =
        normalize(v_Normal);

    vec3 lightDirection =
        normalize(
            vec3(-0.4, 0.8, 0.6));

    float diffuse =
        max(
            dot(N, lightDirection),
            0.0);

    float ambient =
        0.40;

    float lighting =
        ambient +
        diffuse * 0.75;

    vec3 color =
        u_BaseColorFactor.rgb *
        lighting;

    color =
        pow(
            color,
            vec3(1.0 / 2.2));

    FragColor =
        vec4(
            color,
            u_BaseColorFactor.a);
}