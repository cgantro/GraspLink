#pragma once

#include <string>

struct GLFWwindow;

/** @brief Window 입력 API에서 사용하는 mouse button 종류. */
enum class MouseButton
{
    /** GLFW left mouse button. */
    Left,
    /** GLFW right mouse button. */
    Right,
    /** GLFW middle mouse button. */
    Middle
};

/**
 * @brief GLFW Window와 OpenGL Context의 lifetime, 이벤트 polling, 최소 raw input 조회를 관리한다.
 *
 * @details
 * Window가 `GLFWwindow*`를 소유한다. GLFW는 OS별 Window/Input을 추상화하고,
 * 생성된 OpenGL Context를 통해 GLAD가 실제 OpenGL 함수 주소를 로드한다.
 *
 * 좌표/단위:
 * - Properties::width/height: 생성 요청 window 크기 [pixel]
 * - GetFramebufferSize(): HiDPI scale까지 반영된 실제 OpenGL framebuffer 크기 [pixel]
 * - GetCursorPosition(): GLFW window content 좌표 [pixel]
 * - Scroll offset: GLFW callback이 제공하는 무차원 wheel step 누적값
 *
 * @todo [FUTURE] keyboard/drag/focus 이벤트가 늘어나면 raw GLFW 조회를 Input 계층으로 분리한다.
 */
class Window
{
public:
    /** @brief Window 생성 옵션. */
    struct Properties
    {
        /** @brief 요청 client-area width [pixel]. */
        int width = 1280;

        /** @brief 요청 client-area height [pixel]. */
        int height = 720;

        /** @brief OS title bar에 표시할 문자열. */
        std::string title = "GraspLink";

        /** @brief true이면 swap interval을 사용해 vertical sync를 요청한다. */
        bool vsync = true;
    };

public:
    /**
     * @brief Properties에 따라 GLFW Window와 OpenGL Context를 생성한다.
     * @param properties 초기 크기[pixel], title, vsync 설정.
     */
    explicit Window(const Properties& properties);

    /** @brief GLFW Window/Context와 관련 library 상태를 정리한다. */
    ~Window();

    /** @brief Native window/context ownership 중복을 막기 위해 copy construction을 금지한다. */
    Window(const Window&) = delete;

    /** @brief Native window/context ownership 중복을 막기 위해 copy assignment를 금지한다. */
    Window& operator=(const Window&) = delete;

    /** @return OS/GLFW가 Window 종료를 요청했으면 true. */
    bool ShouldClose() const;

    /** @brief OS event queue를 처리해 callback과 key/mouse 상태를 최신화한다. */
    void PollEvents() const;

    /** @brief OpenGL back/front buffer를 교환해 완성된 frame을 화면에 표시한다. */
    void SwapBuffers() const;

    /**
     * @brief HiDPI scale을 반영한 실제 framebuffer 크기를 반환한다.
     * @param width 출력 framebuffer width [pixel].
     * @param height 출력 framebuffer height [pixel].
     */
    void GetFramebufferSize(int& width, int& height) const;

    /**
     * @brief 지정 mouse button의 현재 pressed 상태를 조회한다.
     * @param button Left/Right/Middle 중 확인할 button.
     * @return pressed이면 true.
     */
    bool IsMouseButtonPressed(MouseButton button) const;

    /**
     * @brief 현재 cursor 위치를 Window content 좌표로 반환한다.
     * @param x 출력 X 좌표 [pixel].
     * @param y 출력 Y 좌표 [pixel].
     */
    void GetCursorPosition(double& x, double& y) const;

    /**
     * @brief 마지막 consume 이후 callback이 누적한 vertical scroll 값을 반환하고 내부 값을 0으로 만든다.
     * @return 누적 wheel/trackpad scroll step. 물리 길이 단위가 아니다.
     */
    double ConsumeScrollOffset();

private:
    /**
     * @brief GLFW 초기화, native window/context 생성, GLAD 로딩, callback 설치를 수행한다.
     * @param properties 생성 설정.
     */
    void Init(const Properties& properties);

    /** @brief GLFW Window를 파괴하고 이 wrapper가 획득한 GLFW 관련 자원을 정리한다. */
    void Shutdown();

    /**
     * @brief framebuffer resize 시 OpenGL viewport를 새 실제 framebuffer 크기[pixel]로 갱신하는 GLFW callback.
     */
    static void FramebufferSizeCallback(GLFWwindow* window, int width, int height);

    /**
     * @brief scroll event를 해당 Window wrapper의 누적값에 저장하는 GLFW callback.
     * @param xOffset horizontal scroll step. 현재 controller에서는 사용하지 않는다.
     * @param yOffset vertical scroll step. Zoom 입력으로 소비한다.
     */
    static void ScrollCallback(GLFWwindow* window, double xOffset, double yOffset);

private:
    /** @brief 소유 중인 native GLFW window/context handle. */
    GLFWwindow* m_Handle = nullptr;

    /** @brief 마지막 ConsumeScrollOffset 이후 누적된 vertical scroll step. */
    double m_ScrollOffset = 0.0;
};
