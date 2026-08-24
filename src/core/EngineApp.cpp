#include "core/EngineApp.h"
#include <glad/glad.h>
#include <cstdlib>
/**
 * @brief EngineApp
 * @author 홍윤표
 * 
 */


void EngineApp::Run(){
    Init();
    MainLoop();
    Shutdown();
}

void EngineApp::Init(){
    m_Window = std::make_shared<Window>(1280,720,"MiniBCG");

    // 1. Window + OpenGL Context 생성
    m_Window->Init();

    // 2. Quad 정점 데이터
    // OpenGL의 화면 좌표인 Clip Space의 범위는 -1~1이다.
    // 삼각형 두개로 사각형 그리기
    const float vertices[] = {
        // x     y
        -0.5f, -0.5f,
         0.5f, -0.5f,
         0.5f,  0.5f,


        -0.5f, -0.5f,
         0.5f,  0.5f,
        -0.5f,  0.5f,
    };
    // 3. VAO 생성
    m_VertexArray = std::make_unique<VertexArray>();
    m_VertexArray->Bind();

    // 4. VBO
    m_VertexBuffer = std::make_unique<VertexBuffer>(
        vertices,
        sizeof(vertices)
    );

    m_VertexBuffer->Bind();

    // 5. Vertex Attribute 활성화
    // Shader : layout(location = 0) in vec2 a_Position;
    // 의 location 0을 사용하겠다고 OpenGL에 알려줌
    glEnableVertexAttribArray(0);

    // 6. VBO 데이터를 어떻게 읽을지 설명
    glVertexAttribPointer(
        0,          // Shader location = 0
        2,          // Vertex 하나당, float 2개: x,y
        GL_FLOAT,   
        GL_FALSE,   // 정규화 X : 보낸 숫자 그대로 사용 어떤 범위의 숫자들을 0.0 ~ 1.0 (또는 -1.0 ~ 1.0)로 압축하지 않음
        2 * sizeof(float), // Vertex 하나의 크기 [x y](float 2개)
        nullptr
    );

    // 7. Shader 생성
    m_Shader = Shader::Create("assets/shaders/Quad.glsl");

    m_VertexArray->UnBind();
    m_VertexBuffer->UnBind();
    m_Shader->UnBind();
}

void EngineApp::MainLoop(){
    while(!m_Window->ShouldClose()){
        m_Window->PollEvents(); // 이벤트 관리

        glClearColor(0.1f, 0.3f, 0.8f, 1.0f); //RGBA 다음에 화면을 지울 때 사용할 색

        glClear(GL_COLOR_BUFFER_BIT); // 화면 지우기

        // Shader 선택
        m_Shader->Bind();
        
        // Vertex Layout 선택
        m_VertexArray->Bind();

        glDrawArrays(
            GL_TRIANGLES,
            0,
            6
        );

        m_Window->SwapBuffers();
    }
}

void EngineApp::Shutdown(){
    // 현재는 Window만 정리
    // 나중에는 OpenGL 자원을 정리하고 Window를 마지막에 정리해야 한다.
    
    m_Shader.reset();
    m_VertexBuffer.reset();
    m_VertexArray.reset();

    m_Window->Shutdown();
}