#type vertex
#version 330 core

layout(location = 0) in vec3 a_Position;

uniform mat4 u_Model;
uniform mat4 u_View;
uniform mat4 u_Projection;

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
}


#type fragment
#version 330 core

in vec3 v_WorldPosition;

out vec4 FragColor;


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