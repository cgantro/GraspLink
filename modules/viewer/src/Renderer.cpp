#include "Renderer.h"

#include "Shader.h"
#include "Texture.h"
#include "Mesh.h"
#include "Camera.h"

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <memory>

namespace PoseLink {

Renderer::Renderer() = default;
Renderer::~Renderer() = default;

void Renderer::Init() {
   

    /*
        Mesh가 VAO/VBO/EBO 생성과 Vertex Layout까지 책임
    */
    m_Mesh = Mesh::CreateCube();

    /*
        Camera를 World의 (2,2,3)에 놓고
        원점(0,0,0)을 바라보게 한다.
    */
    m_Camera = std::make_unique<Camera>(
        glm::vec3(2.0f,2.0f,3.0f),
        glm::vec3(0.0f,0.0f,0.0f),
        1280.f / 720.0f
    );
    const unsigned char pixels[] = {
        255,   0,   0, 255,
          0, 255,   0, 255,
          0,   0, 255, 255,
        255, 255,   0, 255
    };
    m_Texture = std::make_unique<Texture>(2,2);
    m_Texture->Update(pixels);


    // 기존 Quad Shader 대신 3D 변환을 지원하는 Cube Shader 사용.
    m_Shader = Shader::Create("shaders/Cube.glsl");
    m_Shader->Bind();

    // uniform sampler2D u_Texture;
    // 아래의 0은 GL_TEXTURE0을 의미한다.
    m_Shader->SetInt("u_Texture", 0);
    m_Shader->UnBind();

    /*
        Depth Test 활성화
        Camera에 더 가까운 Fragment가 화면에 남아야 함

        OpenGL은 각 Fragment의 깊이 값을 Depth Buffer와 비교해서
        더 가까운 Fragment만 통과시킨다
    */
    glEnable(GL_DEPTH_TEST);
}

void Renderer::BeginFrame() {
    
    // TODO: Clear framebuffer and prepare render state.

    glClearColor(
        0.1f,
        0.1f,
        0.1f,
        1.0f
    ); // 배경색

    /*
        Color Buffer:
            이전 Frame의 색상 제거
        Depth Buffer :
            이전 Frame에서 기록된 깊이 값 제거
        
        Depth Buffer를 매 프레임마다 지우지 않으면
        이전 Frame의 깊이 값 때문에 새 물체가 렌더링되지 않을 수 있다
    */
    glClear(
        GL_COLOR_BUFFER_BIT |
        GL_DEPTH_BUFFER_BIT
    );
}

void Renderer::Render() {

    m_Shader->Bind();

    /*
        Model Matrix -> 이 물체가 World상 어디에 있는가
        지금 Cube는:
            위치 이동, 회전, 크기 변경 x
        -> 단위 행렬 사용
    */
    const glm::mat4 model(1.0f);
    /*
        View Matrix:
            카메라의 위치와 바라보는 방향 기준
            World를 Camera 좌표계로 변환한다.
    */
    const glm::mat4 view = m_Camera->GetViewMatrix();
    /*
        Projection Matrix: 3D 공간에 원근감
    */
    const glm::mat4 projection = m_Camera->GetProjectionMatrix();
    

    /*
        Shader의 uniform으로 Matrix를 전달한다.

        Shader::SetMat4() 내부에서는 보통:

            glUniformMatrix4fv(...)

        를 호출한다.

        현재 Shader가 Bind된 상태여야
        이 Shader Program의 uniform이 수정된다.
    */
    m_Shader->SetMat4("u_Model",model);
    m_Shader->SetMat4("u_View",view);
    m_Shader->SetMat4("u_Projection",projection);

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
