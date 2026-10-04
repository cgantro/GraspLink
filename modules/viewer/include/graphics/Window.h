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
/*
 * [추가 그래픽스 용어 설명]
 * - GLFW: Windows/Linux/macOS의 Window 생성, OpenGL Context, 입력 event를 공통 API로 다루게 해주는 library.
 * - OpenGL Context: 어떤 GPU/OpenGL 상태와 리소스를 현재 thread에서 사용할지 연결하는 실행 환경.
 * - GLAD: 실행 환경에서 실제 OpenGL 함수 주소를 로드하는 loader.
 * - Event Polling: OS가 쌓아 둔 mouse/keyboard/window event를 읽어 application 상태를 갱신하는 과정.
 * - Double Buffering: 화면에 보이는 front buffer와 다음 frame을 그리는 back buffer를 분리하는 방식.
 * - Swap Buffers: 완성한 back buffer를 front buffer와 교환해 화면에 표시하는 동작.
 * - VSync: buffer swap을 모니터 주사율 타이밍에 맞춰 tearing을 줄이는 옵션.
 * - Framebuffer Size: 실제 렌더링 pixel 크기. HiDPI에서는 논리 Window 크기와 다를 수 있다.
 *
 * Properties::width/height와 GetFramebufferSize의 width/height는 pixel 계열 값이지만 HiDPI 환경에서는 서로 다를 수 있다.
 */
class Window
{
public:
    /** @brief Window 생성 옵션. */
    struct Properties
    {
        // 요청하는 초기 Window 너비. 일반 desktop 환경에서는 pixel과 비슷하지만 HiDPI에서는 framebuffer와 다를 수 있다.
        int width = 1280;
        // 요청하는 초기 Window 높이.
        int height = 720;
        // OS title bar에 표시할 문자열.
        std::string title = "GraspLink";
        // true이면 swap interval을 사용해 보통 모니터 refresh에 맞춰 buffer swap한다.
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
    // 실제 GLFW window/context를 가리키는 native handle. Window 객체가 소유하며 nullptr이면 생성되지 않은 상태다.
    GLFWwindow* m_Handle = nullptr;

    // callback으로 들어온 vertical scroll을 OnUpdate 쪽에서 한 번 소비할 때까지 누적한 값.
    double m_ScrollOffset = 0.0;
};
