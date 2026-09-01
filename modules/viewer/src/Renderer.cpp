#include "Renderer.h"

#include "Shader.h"
#include "Texture.h"
#include "Mesh.h"

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


    /*
        Mesh가 VAO/VBO/EBO 생성과 Vertex Layout까지 책임
    */
    m_Mesh = std::make_unique<Mesh>(
        vertices,sizeof(vertices),
        indices,6
    );

    m_Texture = std::make_unique<Texture>(2,2);
    m_Texture->Update(pixels);

    m_Shader = Shader::Create("shaders/Quad.glsl");
    m_Shader->Bind();

    // uniform sampler2D u_Texture;
    // 아래의 0은 GL_TEXTURE0을 의미한다.
    m_Shader->SetInt("u_Texture", 0);
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
        Mesh::Bind()는 내부 VAO를 Bind
        VAO가 VBO의 Layout과, EBO 연결 상태를 기억하고 있음
    */
    m_Mesh->Bind();
    glDrawElements(
        GL_TRIANGLES,
        m_Mesh->GetIndexCount(),
        GL_UNSIGNED_INT,
        nullptr
    );
    m_Mesh->UnBind();
    m_Texture->UnBind();
    m_Shader->UnBind();
}

void Renderer::EndFrame() {
    // TODO: Finalize the frame and present renderer output.
}

} // namespace PoseLink
