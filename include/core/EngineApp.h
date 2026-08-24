#pragma once

#include<memory>
#include "core/Window.h"


class Window;
class EngineApp{
public:
    void Run();
private:
    void Init();
    void MainLoop();
    void Shutdown();


    float m_LastFrameTime; // 
    std::shared_ptr<Window> m_Window;
    /*
    Window m_window;
    Renderer m_renderer;
    Scene   m_Scene
    */
};