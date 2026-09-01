#include "Renderer.h"

#include "Shader.h"
#include "VertexBuffer.h"
#include "VertexArray.h"
#include "Texture.h"
#include "IndexBuffer.h"

#include <glad/glad.h>

#include <memory>

namespace PoseLink {

Renderer::Renderer() = default;
Renderer::~Renderer() = default;

void Renderer::Init() {
    // TODO: Initialize renderer-owned resources after an OpenGL context exists.
    const float vertices[] = {

        // Position      // UV

        /*
                        Vertex 0
            ┌────────────────────────────┐
            │  x  │  y  │  u  │  v      │
            └────────────────────────────┘
               ↑           ↑
            position       texture coord
            location 0     location 1

            3 ---------------- 2
            |                  |
            |                  |
            |                  |
            |                  |
            0 ---------------- 1
        */
         // 0: Left Bottom
        -1.0f, -1.0f,      0.0f, 0.0f,

        // 1: Right Bottom
         1.0f, -1.0f,      1.0f, 0.0f,

        // 2: Right Top
         1.0f,  1.0f,      1.0f, 1.0f,

        // 3: Left Top
        -1.0f,  1.0f,      0.0f, 1.0f
    };

    // Index 배열
    const uint32_t indices[] = {
        // Triangle 1
        0,1,2,
        // Triangle 2
        2,3,0
    };
    /*
    * 2×2 RGBA 테스트 Texture.
    *
    * 메모리는 1차원 배열이지만,
    * 논리적으로는 다음과 같은 2차원 이미지다.
    *
    * R = Red
    * G = Green
    * B = Blue
    * Y = Yellow
    *
    *     x →
    *
    *   R   G
    *
    *   B   Y
    *
    *
    * Pixel 하나:
    *
    * [R][G][B][A]
    *
    * 각 Channel은 unsigned char 1byte.
    */
    const unsigned char pixels[] = {

        // 첫 번째 Pixel - Red
        255,   0,   0, 255,
        // 두 번째 Pixel - Green
          0, 255,   0, 255,
        // 세 번째 Pixel - Blue
          0,   0, 255, 255,
        // 네 번째 Pixel - Yellow
        255, 255,   0, 255
    };

    // VAO 바인딩
    m_VertexArray = std::make_unique<VertexArray>();
    m_VertexArray->Bind();

    // VBO
    m_VertexBuffer = std::make_unique<VertexBuffer>(vertices,sizeof(vertices));
    m_VertexBuffer->Bind();

    // EBO
    m_IndexBuffer = std::make_unique<IndexBuffer>(indices,6);

    m_Texture = std::make_unique<Texture>(2,2);
    m_Texture->Update(pixels);

     /*
        location 0:
        Vertex의 position 영역을 어떻게 읽을지 설정.

        한 Vertex:

        [pos.x][pos.y][uv.x][uv.y]
         ↑
        여기서 시작

        stride = float 4개
    */
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(
        0,
        2,
        GL_FLOAT,
        GL_FALSE,
        4 * sizeof(float), // 다음 정점까지
        nullptr
    );

    /*
        location 1:
        Texture Coordinate 영역.

        같은 Vertex에서 float 2개를 건너뛴 위치부터 읽는다.

        [pos.x][pos.y][uv.x][uv.y]
                       ↑
                    여기서 시작
    */
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(
        1,
        2,
        GL_FLOAT,
        GL_FALSE,
        4*sizeof(float),
        reinterpret_cast<void*>(2 * sizeof(float))
    );

    m_Shader = Shader::Create("shaders/Quad.glsl");
    m_Shader->Bind();

    // uniform sampler2D u_Texture;
    // 아래의 0은 GL_TEXTURE0을 의미한다.
    m_Shader->SetInt("u_Texture", 0);

    m_VertexArray->UnBind();
    m_VertexBuffer->UnBind();
    m_Shader->UnBind();
}

void Renderer::BeginFrame() {
    
    // TODO: Clear framebuffer and prepare render state.

    glClearColor(
        0.1f,
        0.1f,
        0.1f,
        1.0f
    ); // 배경색

    glClear(
        GL_COLOR_BUFFER_BIT
    );
}

void Renderer::Render() {

    m_Shader->Bind();

    // Texture Unit에 Object 연결
    m_Texture->Bind(0);

    /*
        VAO를 Bind하면 Init()에서 저장해둔:

        - Vertex Attribute 설정
        - EBO Binding

        을 다시 사용할 수 있다.
    */
    m_VertexArray->Bind();

    
    // glDrawArrays(
    //     GL_TRIANGLES,
    //     0,
    //     6
    // );
    /*
        glDrawElements는 Vertex를 순서대로 읽지 않고
        EBO에 들어있는 Index를 먼저 읽는다.

        EBO:
            0, 1, 2,
            2, 3, 0

        따라서 VBO의 4개 Vertex를 이용해
        총 두 개의 Triangle을 그린다.
    */
    glDrawElements(
        GL_TRIANGLES,
        // EBO에서 읽을 Index 개수
        m_IndexBuffer->GetCount(), 
        // Index 하나의 자료형 32_t -> 4byte Uint
        GL_UNSIGNED_INT,
        nullptr // EBO 시작위치로부터 얼마나 떨어진 위치부터 읽나, 처음부터 읽음 -> nullptr = offset 0
    );
    m_VertexArray->UnBind();
    m_Texture->UnBind();
    m_Shader->UnBind();

}

void Renderer::EndFrame() {
    // TODO: Finalize the frame and present renderer output.
}

} // namespace PoseLink
