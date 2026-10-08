#include "gui/GuiModule.h"

#include "graphics/Window.h"

#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

namespace grasplink::gui
{

struct GuiModule::Impl
{
    explicit Impl(grasplink::graphics::Window& window)
    {
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGui::StyleColorsDark();
        ImGui_ImplGlfw_InitForOpenGL(window.GetNativeHandle(), true);
        ImGui_ImplOpenGL3_Init("#version 330");
    }

    ~Impl()
    {
        // UI의 OpenGL backend가 GPU 자원을 해제하므로, 이 호출이 끝날 때까지 Window의 OpenGL context가 유효해야 한다.
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }

    void BeginFrame()
    {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
    }

    void EndFrame()
    {
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }
};

GuiModule::GuiModule(grasplink::graphics::Window& window)
    : m_Impl(std::make_unique<Impl>(window))
{
}

GuiModule::~GuiModule() = default;

void GuiModule::BeginFrame()
{
    m_Impl->BeginFrame();
}

void GuiModule::EndFrame()
{
    m_Impl->EndFrame();
}

bool GuiModule::WantsMouse() const
{
    return ImGui::GetIO().WantCaptureMouse;
}

bool GuiModule::WantsKeyboard() const
{
    return ImGui::GetIO().WantCaptureKeyboard;
}

}
