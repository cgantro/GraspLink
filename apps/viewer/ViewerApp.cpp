#include "ViewerApp.h"

#include "Window.h"

#include <glad/glad.h>

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

    return true;
}

void ViewerApp::MainLoop(){
    /*
        매 Frame
    */

    while(!m_Window->ShouldClose()){
        m_Window->PollEvents();

        glClearColor(0.1f,0.1f,0.1f,1.0f); // 추후 렌더러 만들고 BeginFrame

        glClear(GL_COLOR_BUFFER_BIT);

        m_Window->SwapBuffers();
    }
}

void ViewerApp::Shutdown(){
    m_Window.reset();
}