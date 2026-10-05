#pragma once

#include <string>

struct GLFWwindow;

enum class MouseButton
{
    Left,
    Right,
    Middle
};

/**
 * @brief 입력 이벤트를 받고 OpenGL 화면을 표시하는 GLFW 창을 소유한다.
 * @details 생성 과정에서 GLFW를 초기화하고 이 창의 OpenGL Context를 현재 Context로 만든 뒤
 * GLAD 함수 포인터를 적재한다. GLFW 전역 수명과 Context를 함께 맡으므로 GL 자원을 가진
 * 객체보다 나중에 파괴해야 한다. 애플리케이션은 한 번에 하나의 Window만 둔다.
 */
class Window
{
public:

    /** 창 생성과 렌더링 동작에 필요한 설정이다. */
    struct Properties
    {

        /** 요청하는 논리 창 너비 [화면 좌표]. */
        int width = 1280;
        /** 요청하는 논리 창 높이 [화면 좌표]. */
        int height = 720;
        std::string title = "GraspLink";
        /** true이면 버퍼 교환을 화면 주사 주기에 맞춘다. */
        bool vsync = true;
        /** false이면 창을 숨긴 채 렌더링할 수 있다. */
        bool visible = true;
    };

public:

    /**
     * @brief GLFW 창과 OpenGL Context를 만들고 GL 함수를 준비한다.
     * @param properties 논리 창 크기, 제목, 표시 여부, 버퍼 교환 간격
     * @throws std::runtime_error GLFW, 창 생성 또는 GLAD 초기화가 실패한 경우
     */
    explicit Window(const Properties& properties);

    /** 보유한 창을 파괴하고 GLFW 전역 사용을 종료한다. */
    ~Window();

    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;

    /** @return 사용자가 창을 닫았거나 종료가 요청되었으면 true */
    bool ShouldClose() const;

    /**
     * @brief 대기 중인 키보드·마우스·창 크기 이벤트를 처리한다.
     * @details framebuffer 크기 변경 callback은 이 호출 중 viewport를 갱신한다. 호출자는
     * GetFramebufferSize로 실제 픽셀 크기를 다시 읽고, 최소화로 크기가 0이면 그리기를 건너뛴다.
     */
    void PollEvents() const;

    /** 현재 OpenGL Context의 back buffer를 화면에 표시한다. */
    void SwapBuffers() const;

    /**
     * @brief 렌더 대상 framebuffer의 크기를 읽는다.
     * @param width framebuffer 너비 [픽셀]
     * @param height framebuffer 높이 [픽셀]
     * @details HiDPI에서는 GLFW 논리 창 크기보다 픽셀 수가 클 수 있다. 최소화 중에는
     * 한 축 또는 두 축이 0일 수 있으므로 렌더링 전에 확인해야 한다.
     */
    void GetFramebufferSize(int& width, int& height) const;

    /** @param button 확인할 마우스 버튼 @return 버튼을 누르고 있으면 true */
    bool IsMouseButtonPressed(MouseButton button) const;

    /**
     * @brief 현재 커서 위치를 GLFW 논리 창 좌표로 읽는다.
     * @param x 커서의 가로 위치 [화면 좌표]
     * @param y 커서의 세로 위치 [화면 좌표]
     */
    void GetCursorPosition(double& x, double& y) const;

    /** @return GLFW 입력 API와 통합에 필요한 기본 창 핸들 */
    GLFWwindow* GetNativeHandle() const;

    /** @return 누적된 세로 스크롤량을 반환하고 누적값을 0으로 초기화한다. */
    double ConsumeScrollOffset();

private:

    void Init(const Properties& properties);

    void Shutdown();

    static void FramebufferSizeCallback(GLFWwindow* window, int width, int height);

    static void ScrollCallback(GLFWwindow* window, double xOffset, double yOffset);

private:
    GLFWwindow* m_Handle = nullptr;

    // 세로 scroll callback 누계; Consume 시 초기화한다.
    double m_ScrollOffset = 0.0;
};
