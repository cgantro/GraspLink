#pragma once

#include <string>

struct GLFWwindow;

enum class MouseButton
{
    Left,
    Right,
    Middle
};

// 역할: 현재 프로세스의 단일 GLFW Window와 OpenGL Context 수명을 관리한다.
// 수명: GPU 자원은 이 Context가 파괴되기 전에 해제한다.
class Window
{
public:

    struct Properties
    {

        // 요청 논리 Window 크기 [px].
        int width = 1280;
        int height = 720;
        std::string title = "GraspLink";
        bool vsync = true;
        bool visible = true;
    };

public:

    explicit Window(const Properties& properties);

    ~Window();

    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;

    bool ShouldClose() const;

    void PollEvents() const;

    void SwapBuffers() const;

    // 실제 렌더 크기 [framebuffer px]; HiDPI에서는 논리 크기와 다를 수 있다.
    void GetFramebufferSize(int& width, int& height) const;

    bool IsMouseButtonPressed(MouseButton button) const;

    // Cursor 위치는 논리 Window 좌표 [px].
    void GetCursorPosition(double& x, double& y) const;

    GLFWwindow* GetNativeHandle() const;

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
