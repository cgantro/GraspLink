#include "ViewerApp.h"

#include "Window.h"
#include "Renderer.h"

#include <glad/glad.h>

ViewerApp::ViewerApp() = default;
ViewerApp::~ViewerApp() = default;

int ViewerApp::Run(){
    if(!Init()) return -1;

    MainLoop();

    Shutdown();

    return 0;
}

bool ViewerApp::Init(){
    // 하위 시스템 생성

    m_Window =
        std::make_unique<PoseLink::Window>(
            PoseLink::Window::Properties{
                1280,720,"PoseLink Viewer",true
            }
        );
    
    m_Renderer = std::make_unique<PoseLink::Renderer>();
    m_Renderer->Init();
    return true;
}

void ViewerApp::MainLoop(){
    /*
        매 Frame
    */

    while(!m_Window->ShouldClose()){
        m_Window->PollEvents();

        m_Renderer->BeginFrame();

        m_Renderer->Render();

        m_Renderer->EndFrame();

        m_Window->SwapBuffers();
    }
}

void ViewerApp::Shutdown(){
    // OpenGL객체는 Context가 살아있을 때 삭제하는 게 안전 -> 렌더러 먼저 삭제
    m_Renderer.reset(); 
    m_Window.reset();
}
