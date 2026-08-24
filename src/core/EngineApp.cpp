
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "core/EngineApp.h"
#include "core/Window.h"
#include "graphics/Renderer.h"
#include "media/VideoSource.h"
#include "scene/Scene.h"

EngineApp::EngineApp() = default;

EngineApp::~EngineApp() = default;

void EngineApp::Run() {
    Init();
    MainLoop();
    Shutdown();
}

void EngineApp::Init() {
    // TODO: Create Window and initialize its platform context.
    m_Window = std::make_unique<Window>(1280,720,"MiniBCG");
    m_Window->Init();

    m_Renderer = std::make_unique<Renderer>();
    m_Renderer->Init();

    m_Scene = std::make_unique<Scene>();
    // TODO: Create Renderer after the Window context is available.
    // TODO: Create the Scene and configure its initial graphic elements.
    // TODO: Create VideoSource when a media input is selected.
}

void EngineApp::MainLoop() {
    
    // TODO: Poll Window events and calculate delta time.
    while(!m_Window->ShouldClose()){
        m_Window->PollEvents(); // 이벤트 처리

        m_Renderer->BeginFrame();
        m_LastFrameTime = (float)glfwGetTime();
        m_Scene->Update(m_LastFrameTime);
        m_Renderer->Render(
            *m_Scene
        );
        m_Renderer->EndFrame();

        m_Window->SwapBuffers();
    }
    // TODO: Update VideoSource and Scene.
    // TODO: Delegate frame rendering to Renderer.
}

void EngineApp::Shutdown() {
    // TODO: Release media and scene state before Renderer and Window teardown.

    m_Renderer.reset();
    m_Scene.reset();
    m_Window.reset();
}
