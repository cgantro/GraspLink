#type vertex
#version 330 core

// Mesh의 location 0.
// 이제 position은 2D가 아니라 x, y, z를 가진 3D 좌표다.
layout(location = 0) in vec3 a_Position;

// Mesh의 location 1.
layout(location = 1) in vec2 a_TexCoord;

// CPU(Renderer)에서 전달받는 변환 행렬.
//
// Model      : 물체의 Local 좌표 -> World 좌표
// View       : World 좌표 -> Camera 기준 좌표
// Projection : Camera 좌표 -> 화면 투영을 위한 Clip 좌표
uniform mat4 u_Model;
uniform mat4 u_View;
uniform mat4 u_Projection;

// Fragment Shader로 전달할 Texture Coordinate.
out vec2 v_TexCoord;

void main()
{
    /*
        Vertex가 최종 화면에 도달하기까지:

        Local
          ↓ Model
        World
          ↓ View
        Camera
          ↓ Projection
        Clip Space

        행렬 곱은 오른쪽부터 적용된다.
    */
    gl_Position =
        u_Projection
        * u_View
        * u_Model
        * vec4(a_Position, 1.0);

    v_TexCoord = a_TexCoord;
}


#type fragment
#version 330 core

in vec2 v_TexCoord;

uniform sampler2D u_Texture;

out vec4 FragColor;

void main()
{
    // 보간되어 들어온 UV 좌표로 Texture의 Pixel 값을 읽는다.
    FragColor = texture(u_Texture, v_TexCoord);
}