#type vertex
#version 330 core


// VBO:
//
// [x y u v]
//  ↑ ↑
//
// Position
layout(location = 0) in vec2 a_Position;


// VBO:
//
// [x y u v]
//      ↑ ↑
//
// Texture Coordinate
layout(location = 1) in vec2 a_TexCoord;


// Fragment Shader로 전달할 Texture Coordinate.
//
// Vertex Shader의 out과
// Fragment Shader의 in이 연결된다.
out vec2 v_TexCoord;


void main()
{
    // Fullscreen Quad의 Position
    gl_Position = vec4(
        a_Position,
        0.0,
        1.0
    );


    // 현재 Vertex의 UV를 Fragment Shader 쪽으로 전달
    v_TexCoord = a_TexCoord;
}


#type fragment
#version 330 core


// Vertex Shader에서 전달받은 UV.
//
// Rasterizer가 각 Fragment 위치에 맞게
// 자동으로 보간한 값이다.
in vec2 v_TexCoord;


// 최종 화면 색상
out vec4 FragColor;


// Texture Sampler.
//
// 이것 자체가 Texture 객체는 아니다.
//
// 실제로는:
//
// u_Texture
//     ↓
// Texture Unit
//     ↓
// Texture Object
//
// 구조로 연결된다.
uniform sampler2D u_Texture;


void main()
{
    // 현재 UV 좌표에 해당하는 Texture Pixel을 읽는다.
    //
    // 반환 값은 vec4:
    //
    // RGBA
    FragColor = texture(
        u_Texture,
        v_TexCoord
    );
}