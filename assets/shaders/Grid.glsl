#type vertex
#version 330 core

layout(location = 0) in vec3 a_Position;

uniform mat4 u_Model;
uniform mat4 u_View;
uniform mat4 u_Projection;
uniform mat4 u_LightSpaceMatrix;

out vec4 v_LightSpacePosition;
out vec3 v_WorldPosition;

void main()
{
    vec4 worldPosition =
        u_Model *
        vec4(a_Position, 1.0);

    v_WorldPosition =
        worldPosition.xyz;

    gl_Position =
        u_Projection *
        u_View *
        worldPosition;
    v_LightSpacePosition =
        u_LightSpaceMatrix *
        worldPosition;
}


#type fragment
#version 330 core

in vec3 v_WorldPosition;
in vec4 v_LightSpacePosition;

uniform sampler2D u_ShadowMap;
out vec4 FragColor;

float CalculateShadow(
    vec4 lightPosition)
{
    vec3 projCoords =
        lightPosition.xyz /
        lightPosition.w;

    projCoords =
        projCoords * 0.5 + 0.5;

    if (projCoords.z > 1.0)
    {
        return 0.0;
    }

    float currentDepth =
        projCoords.z;

    vec2 texelSize =
        1.0 /
        textureSize(
            u_ShadowMap,
            0);

    float shadow =
    CalculateShadow(
        v_LightSpacePosition);

    color *= mix(1.0,0.42,shadow);

    for (int x = -1;
         x <= 1;
         ++x)
    {
        for (int y = -1;
             y <= 1;
             ++y)
        {
            float depth =
                texture(
                    u_ShadowMap,
                    projCoords.xy +
                    vec2(x, y) *
                    texelSize)
                    .r;

            shadow +=
                currentDepth - 0.001 >
                depth
                    ? 1.0
                    : 0.0;
        }
    }

    return shadow / 9.0;
}

// scale = Grid 간격
float GridLine(
    vec2 position,
    float scale)
{
    vec2 coord =
        position / scale;

    vec2 derivative =
        fwidth(coord);

    vec2 grid =
        abs(
            fract(coord - 0.5) -
            0.5)
        / derivative;

    float line =
        min(
            grid.x,
            grid.y);

    return
        1.0 -
        min(line, 1.0);
}


void main()
{
    vec2 position =
        v_WorldPosition.xz;


    // 10cm
    float minor =
        GridLine(
            position,
            0.10);


    // 50cm
    float major =
        GridLine(
            position,
            0.50);


    vec3 baseColor =
        vec3(0.09);

    vec3 minorColor =
        vec3(0.16);

    vec3 majorColor =
        vec3(0.28);


    vec3 color =
        mix(
            baseColor,
            minorColor,
            minor * 0.6);

    color =
        mix(
            color,
            majorColor,
            major);


    // X axis
    float xAxis =
        1.0 -
        smoothstep(
            0.0,
            0.008,
            abs(v_WorldPosition.z));

    // Z axis
    float zAxis =
        1.0 -
        smoothstep(
            0.0,
            0.008,
            abs(v_WorldPosition.x));


    color =
        mix(
            color,
            vec3(0.55, 0.10, 0.08),
            xAxis);

    color =
        mix(
            color,
            vec3(0.08, 0.20, 0.55),
            zAxis);


    color =
        pow(
            color,
            vec3(1.0 / 2.2));


    FragColor =
        vec4(color, 1.0);
}