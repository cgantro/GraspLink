
#type vertex
#version 330 core

// Mesh는 삼각형 정점과 연결 번호를 모은 형상이다. 이 바닥 Mesh는 위치만 받아 장면 전체 기준 격자로 그린다.

layout(location = 0) in vec3 a_Position;

uniform mat4 u_Model;
uniform mat4 u_View;
uniform mat4 u_Projection;

out vec3 v_WorldPosition;

void main()
{
    vec4 worldPosition = u_Model * vec4(a_Position, 1.0);

    v_WorldPosition = worldPosition.xyz;

    // 격자는 장면의 XZ 평면에 고정한다. 화면에 그릴 때는 Robot Mesh와 같은 카메라 변환을 거친다.
    gl_Position = u_Projection * u_View * worldPosition;
}

#type fragment
#version 330 core


in vec3 v_WorldPosition;


out vec4 FragColor;

float GridLine(vec2 position, float scale)
{
    vec2 coord = position / scale;

    // 반복되는 셀 경계에서 선을 만든다. fwidth는 화면 한 픽셀에 해당하는 좌표 변화를 알려주므로 그 크기에 맞춰 선 가장자리를 부드럽게 해 멀리 있는 격자가 깜빡이는 현상을 줄인다.
    vec2 derivative = fwidth(coord);

    vec2 grid =
        abs(fract(coord - 0.5) - 0.5) /
        max(derivative, vec2(0.00001));

    float line = min(grid.x, grid.y);

    return 1.0 - min(line, 1.0);
}

void main()
{
    vec2 position = v_WorldPosition.xz;

    // 간격은 장면 좌표의 XZ 평면에서 재며 단위는 m다. 보조선은 0.10 m, 주선은 0.50 m마다 반복한다.
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

    color = pow(color, vec3(1.0 / 2.2));

    FragColor = vec4(color, 1.0);
}
