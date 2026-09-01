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
   

    /*
        Mesh가 VAO/VBO/EBO 생성과 Vertex Layout까지 책임
    */
    m_Mesh = Mesh::CreateCube();

    const unsigned char pixels[] = {
    255,   0,   0, 255,
      0, 255,   0, 255,
      0,   0, 255, 255,
    255, 255,   0, 255
    };
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
