#pragma once

#include <memory>

class Renderer;
class Scene;
class VideoSource;
class Window;

// Owns the top-level application objects and coordinates their lifecycle.
// Rendering details stay behind Renderer so application flow does not depend on OpenGL objects.
class EngineApp {
public:
    EngineApp();
    void Run();
    ~EngineApp();

private:
    void Init();
    void MainLoop();
    void Shutdown();

    float m_LastFrameTime;
    std::unique_ptr<Window> m_Window;
    std::unique_ptr<Renderer> m_Renderer;
    std::unique_ptr<Scene> m_Scene;
    std::unique_ptr<VideoSource> m_VideoSource;
};
