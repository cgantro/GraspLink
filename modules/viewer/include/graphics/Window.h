
#pragma once

#include <string>

// 전방 선언
struct GLFWwindow;



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

    // GLFWwindow*라는 외부 자원의 소유권을 가진다
    // 따라서 같은 Window를 두 객체가 해제할 수 있다 -> 복사를 막는다.
    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;

    bool ShouldClose() const;
    void PollEvents() const;
    void SwapBuffers() const;
    void GetFramebufferSize(int& width, int& height) const;
private:
    void Init(const Properties& properties);
    void Shutdown();

    static void FramebufferSizeCallback(
        GLFWwindow* window,
        int width,
        int height
    );
private:
    // 실제 GLFW window와 OpenGL Context를 나타내는 핸들
    GLFWwindow* m_Handle = nullptr;
};
