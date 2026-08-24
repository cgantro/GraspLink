#include "graphics/Renderer.h"

#include "graphics/Shader.h"
#include "graphics/VertexBuffer.h"
#include "graphics/VertexArray.h"

#include <glad/glad.h>

#include <memory>

Renderer::Renderer() = default;
Renderer::~Renderer() = default;

void Renderer::Init() {
    // TODO: Initialize renderer-owned resources after an OpenGL context exists.
    const float vertices[] = {

        // 첫 번째 Triangle
        // 왼쪽 아래
        -1.0f, -1.0f,

        // 오른쪽 아래
         1.0f, -1.0f,

        // 오른쪽 위
         1.0f,  1.0f,


        // 두 번째 Triangle
        // 왼쪽 아래
        -1.0f, -1.0f,

        // 오른쪽 위
         1.0f,  1.0f,

        // 왼쪽 위
        -1.0f,  1.0f
    };

    m_VertexArray = std::make_unique<VertexArray>();
    m_VertexArray->Bind();

    m_VertexBuffer = std::make_unique<VertexBuffer>(vertices,sizeof(vertices));
    m_VertexBuffer->Bind();

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(
        0,
        2,
        GL_FLOAT,
        GL_FALSE,
        2*sizeof(float),
        nullptr
    );

    m_Shader = Shader::Create("assets/shaders/Quad.glsl");

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

void Renderer::Render(const Scene& scene) {
    // TODO: Render the Scene without exposing OpenGL details to EngineApp.
    (void)scene;

    m_Shader->Bind();
    m_VertexArray->Bind();
    glDrawArrays(
        GL_TRIANGLES,
        0,
        6
    );
    m_VertexArray->UnBind();
    m_Shader->UnBind();

}

void Renderer::EndFrame() {
    // TODO: Finalize the frame and present renderer output.
}
