#type vertex
#version 330 core

// VAO의 location 0번 Vertex Attribute로부터
// 정점의 2D 위치를 전달받는다.
layout(location = 0) in vec2 a_Position;

void main()
{
    // OpenGL Vertex Shader는 최종 정점 위치를
    // vec4 형태의 gl_Position에 기록해야 한다.
    //
    // 현재는 별도의 Model/View/Projection 변환 없이
    // 입력된 좌표를 그대로 Clip Space 좌표로 사용한다.
    gl_Position = vec4(
        a_Position,
        0.0,
        1.0
    );
}


#type fragment
#version 330 core

// 현재 Fragment가 화면에 출력할 최종 RGBA 색상.
out vec4 FragColor;

void main()
{
    // 아직 Texture가 없으므로 고정 색상을 출력한다.
    FragColor = vec4(
        0.2,
        0.7,
        0.9,
        0.1
    );
}