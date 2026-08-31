#include "poselink/graphics/Renderer.h"

#include "poselink/graphics/Shader.h"
#include "poselink/graphics/VertexBuffer.h"
#include "poselink/graphics/VertexArray.h"
#include "poselink/graphics/Texture.h"
#include <glad/glad.h>

#include <memory>

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
        */
        // Triangle 1
        -1.0f, -1.0f,     0.0f, 0.0f,
         1.0f, -1.0f,     1.0f, 0.0f,
         1.0f,  1.0f,     1.0f, 1.0f,

        // Triangle 2
        -1.0f, -1.0f,     0.0f, 0.0f,
         1.0f,  1.0f,     1.0f, 1.0f,
        -1.0f,  1.0f,     0.0f, 1.0f
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

    m_VertexArray = std::make_unique<VertexArray>();
    m_VertexArray->Bind();

    m_VertexBuffer = std::make_unique<VertexBuffer>(vertices,sizeof(vertices));
    m_VertexBuffer->Bind();

    m_Texture = std::make_unique<Texture>(2,2);
    m_Texture->Update(pixels);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(
        0,
        2,
        GL_FLOAT,
        GL_FALSE,
        4 * sizeof(float), // 다음 정점까지
        nullptr
    );

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
    m_VertexArray->Bind();
    glDrawArrays(
        GL_TRIANGLES,
        0,
        6
    );
    m_VertexArray->UnBind();
    m_Texture->UnBind();
    m_Shader->UnBind();

}

void Renderer::EndFrame() {
    // TODO: Finalize the frame and present renderer output.
}
