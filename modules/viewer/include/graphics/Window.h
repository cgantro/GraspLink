#pragma once

#include <string>

struct GLFWwindow;

/** @brief Window 입력 API에서 사용하는 mouse button 종류. */
enum class MouseButton
{
    Left,
    Right,
    Middle
};

/**
 * @brief GLFW Window와 OpenGL Context의 lifetime, 이벤트 polling, 최소 입력 조회를 관리한다.
 *
 * @details
 * Window가 GLFWwindow*를 소유한다. GLFW는 OS별 Window 생성과 입력 event를 추상화하고,
 * 생성된 Window의 OpenGL Context를 통해 GLAD가 실제 OpenGL 함수 주소를 얻는다.
 *
 * @todo [FUTURE] keyboard/drag/focus 이벤트가 늘어나면 raw GLFW 조회를 Input 계층으로 분리한다.
 */
class Window
{
public:
    /** @brief Window 생성 옵션. */
    struct Properties
    {
        int width = 1280;
        int height = 720;
        std::string title = "GraspLink";
        bool vsync = true;
    };

public:
    /** @brief Properties에 따라 GLFW Window와 OpenGL Context를 생성한다. */
    explicit Window(const Properties& properties);

    /** @brief Window와 GLFW 자원을 해제한다. */
    ~Window();

    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;

    /** @brief OS가 Window 종료를 요청했는지 반환한다. */
    bool ShouldClose() const;

    /** @brief OS event queue를 처리해 callback과 입력 상태를 갱신한다. */
    void PollEvents() const;

    /** @brief back/front buffer를 교환해 완성된 frame을 화면에 표시한다. */
    void SwapBuffers() const;

    /** @brief HiDPI를 반영한 실제 framebuffer pixel 크기를 반환한다. */
    void GetFramebufferSize(int& width, int& height) const;

    /** @brief 지정 mouse button의 현재 pressed 상태를 반환한다. */
    bool IsMouseButtonPressed(MouseButton button) const;

    /** @brief 현재 cursor 위치를 Window 좌표로 반환한다. */
    void GetCursorPosition(double& x, double& y) const;

    /** @brief 마지막 consume 이후 누적된 vertical scroll을 반환하고 내부 누적값을 0으로 만든다. */
    double ConsumeScrollOffset();

private:
    /** @brief GLFW/GLAD/Context를 초기화한다. */
    void Init(const Properties& properties);

    /** @brief GLFW Window를 파괴하고 라이브러리를 종료한다. */
    void Shutdown();

    /** @brief framebuffer resize 시 OpenGL viewport를 갱신하는 GLFW callback. */
    static void FramebufferSizeCallback(GLFWwindow* window, int width, int height);

    /** @brief scroll event를 현재 Window 객체의 누적값에 저장하는 GLFW callback. */
    static void ScrollCallback(GLFWwindow* window, double xOffset, double yOffset);

private:
    GLFWwindow* m_Handle = nullptr;
    double m_ScrollOffset = 0.0;
};
