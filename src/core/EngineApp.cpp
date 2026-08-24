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
    m_Window->Init();
}

void EngineApp::MainLoop(){
    while(!m_Window->ShouldClose()){
        m_Window->PollEvents(); // 이벤트 관리

        glClearColor(0.1f, 0.3f, 0.8f, 1.0f); //RGBA 다음에 화면을 지울 때 사용할 색

        glClear(GL_COLOR_BUFFER_BIT); // 화면 지우기
        m_Window->SwapBuffers();
    }
}

void EngineApp::Shutdown(){
    // 현재는 Window만 정리
    // 나중에는 OpenGL 자원을 정리하고 Window를 마지막에 정리해야 한다.
    m_Window->Shutdown();
}