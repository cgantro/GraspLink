#pragma once

#include <memory>

class Window;

namespace grasplink::gui
{

/**
 * @brief ImGui context와 GLFW/OpenGL backend의 수명 및 프레임을 관리한다.
 * @details Window와 OpenGL context는 ViewerApp 소유다. 이 모듈이 backend를 정리할 때까지
 * Window가 살아 있어야 한다. 패널·화면 선의 구성은 호출자에게 맡기며, BeginFrame과 EndFrame
 * 사이에 그린 ImGui 내용을 EndFrame에서 현재 OpenGL framebuffer로 출력한다.
 */
class GuiModule
{
public:
    /** @brief 살아 있는 Window의 입력과 OpenGL context에 ImGui backend를 연결한다. */
    explicit GuiModule(Window& window);

    /** @brief OpenGL/GLFW backend를 먼저 정리한 뒤 ImGui context를 해제한다. */
    ~GuiModule();

    GuiModule(const GuiModule&) = delete;
    GuiModule& operator=(const GuiModule&) = delete;

    /**
     * @brief backend 입력을 갱신하고 ImGui 프레임을 시작한다.
     * @details 호출 뒤 패널과 화면 선을 구성한다. 각 프레임은 EndFrame으로 끝내야 한다.
     */
    void BeginFrame();

    /**
     * @brief 현재 ImGui 프레임을 끝내고 OpenGL draw 명령을 실행한다.
     * @details BeginFrame 뒤에 호출하며, 화면 교환은 Window의 소유자가 수행한다.
     */
    void EndFrame();

    /** @brief ImGui가 마우스를 잡고 있어 Scene 입력을 막아야 하는지 알려준다. */
    bool WantsMouse() const;

    /** @brief ImGui가 키보드를 잡고 있어 Scene 입력을 막아야 하는지 알려준다. */
    bool WantsKeyboard() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_Impl;
};

}
