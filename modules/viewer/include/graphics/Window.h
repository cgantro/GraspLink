#pragma once

#include <string>

struct GLFWwindow;

enum class MouseButton
{
    Left,
    Right,
    Middle
};

class Window{
public:
    struct Properties
    {
        int width = 1280;
        int height = 720;
        std::string title = "GraspLink";
        bool vsync = true;
    };

public:
    explicit Window(const Properties& properties);
    ~Window();

    // GLFWwindow*의 소유권을 가지므로 복사를 금지한다.
    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;

    bool ShouldClose() const;
    void PollEvents() const;
    void SwapBuffers() const;
    void GetFramebufferSize(int& width, int& height) const;

    // Camera controller 등에서 사용하는 최소 입력 조회 API.
    bool IsMouseButtonPressed(MouseButton button) const;
    void GetCursorPosition(double& x, double& y) const;

    /*
        마지막 Consume 이후 누적된 vertical scroll 값을 반환한다.
        반환 후 내부 값은 0으로 초기화한다.
    */
    double ConsumeScrollOffset();

private:
    void Init(const Properties& properties);
    void Shutdown();

    static void FramebufferSizeCallback(
        GLFWwindow* window,
        int width,
        int height
    );

    static void ScrollCallback(
        GLFWwindow* window,
        double xOffset,
        double yOffset
    );

private:
    GLFWwindow* m_Handle = nullptr;
    double m_ScrollOffset = 0.0;
};
