#pragma once

#include <memory>

class Window;

namespace grasplink::gui
{

/**
 * @brief ImGui 화면을 한 프레임 동안 준비하고 현재 창에 출력한다.
 * @details Window는 키보드·마우스 입력과 OpenGL context를 제공하며 ViewerApp이 소유한다. ImGui가 이 창에 연결된 입력·출력 장치를 정리할 때까지 Window가 살아 있어야 한다.
 * 호출자는 패널이나 충돌 모양 선을 BeginFrame()과 EndFrame() 사이에 구성한다. EndFrame()은 완성된 UI를 현재 창의 framebuffer, 즉 화면에 보낼 픽셀 저장소에 그린다.
 */
class GuiModule
{
public:
    /** @brief 이미 만들어진 Window의 입력과 OpenGL context에 ImGui 입출력 연결 코드를 설정한다. */
    explicit GuiModule(Window& window);

    /** @brief OpenGL과 GLFW 입출력 연결을 먼저 끊고, 마지막으로 ImGui가 보관한 UI 상태를 해제한다. */
    ~GuiModule();

    GuiModule(const GuiModule&) = delete;
    GuiModule& operator=(const GuiModule&) = delete;

    /**
     * @brief 창에서 들어온 UI 입력을 갱신하고 새 ImGui 프레임을 시작한다.
     * @details 이 함수 다음에 호출자가 패널과 overlay 선을 구성한다. 시작한 프레임은 반드시 EndFrame()으로 마쳐야 한다.
     */
    void BeginFrame();

    /**
     * @brief 현재 ImGui 프레임을 끝내고 작성한 UI를 framebuffer에 그린다.
     * @details BeginFrame() 다음에 호출한다. 화면 buffer 교환은 Window를 소유한 코드가 수행한다.
     */
    void EndFrame();

    /** @brief ImGui 위젯이 마우스 입력을 사용 중이어서 Scene 카메라 조작에 전달하지 않아야 하는지 알려준다. */
    bool WantsMouse() const;

    /** @brief ImGui 위젯이 키보드 입력을 사용 중이어서 Scene 조작에 전달하지 않아야 하는지 알려준다. */
    bool WantsKeyboard() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_Impl;
};

}
