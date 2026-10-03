#include "Window.h"

#include <stdexcept>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

Window::Window(const Properties& properties)
{
    Init(properties);
}

Window::~Window()
{
    Shutdown();
}

void Window::Init(const Properties& properties)
{
    // GLFW는 Window 생성, 입력 이벤트, OpenGL Context 생성을 OS별로 추상화한다.
    if (glfwInit() == GLFW_FALSE)
        throw std::runtime_error("Failed to Init GLFW");

    // OpenGL 3.3 Core Profile을 요청한다.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    m_Handle = glfwCreateWindow(
        properties.width,
        properties.height,
        properties.title.c_str(),
        nullptr,
        nullptr);

    if (m_Handle == nullptr)
    {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW Window");
    }

    // 이후 OpenGL 호출이 이 Window의 Context를 사용하도록 현재 Thread에 연결한다.
    glfwMakeContextCurrent(m_Handle);

    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
    {
        Shutdown();
        throw std::runtime_error("Failed to Init GLAD");
    }

    /*
        GLFW callback은 static/free function 형태라 this를 직접 받을 수 없다.
        Window*를 user pointer에 저장해 callback에서 다시 복원한다.
    */
    glfwSetWindowUserPointer(m_Handle, this);
    glfwSetFramebufferSizeCallback(m_Handle, FramebufferSizeCallback);
    glfwSetScrollCallback(m_Handle, ScrollCallback);

    glfwSwapInterval(properties.vsync ? 1 : 0);

    int framebufferWidth = 0;
    int framebufferHeight = 0;
    glfwGetFramebufferSize(m_Handle, &framebufferWidth, &framebufferHeight);

    // HiDPI 환경을 포함해 실제 framebuffer pixel 크기에 맞춰 viewport를 설정한다.
    glViewport(0, 0, framebufferWidth, framebufferHeight);
}

void Window::Shutdown()
{
    if (m_Handle != nullptr)
    {
        glfwDestroyWindow(m_Handle);
        m_Handle = nullptr;
    }

    glfwTerminate();
}

bool Window::ShouldClose() const
{
    return glfwWindowShouldClose(m_Handle) == GLFW_TRUE;
}

void Window::PollEvents() const
{
    glfwPollEvents();
}

void Window::SwapBuffers() const
{
    glfwSwapBuffers(m_Handle);
}

void Window::GetFramebufferSize(int& width, int& height) const
{
    glfwGetFramebufferSize(m_Handle, &width, &height);
}

bool Window::IsMouseButtonPressed(MouseButton button) const
{
    int glfwButton = GLFW_MOUSE_BUTTON_LEFT;

    switch (button)
    {
    case MouseButton::Left:
        glfwButton = GLFW_MOUSE_BUTTON_LEFT;
        break;
    case MouseButton::Right:
        glfwButton = GLFW_MOUSE_BUTTON_RIGHT;
        break;
    case MouseButton::Middle:
        glfwButton = GLFW_MOUSE_BUTTON_MIDDLE;
        break;
    }

    return glfwGetMouseButton(m_Handle, glfwButton) == GLFW_PRESS;
}

void Window::GetCursorPosition(double& x, double& y) const
{
    glfwGetCursorPos(m_Handle, &x, &y);
}

double Window::ConsumeScrollOffset()
{
    const double result = m_ScrollOffset;
    m_ScrollOffset = 0.0;
    return result;
}

void Window::FramebufferSizeCallback(GLFWwindow* window, int width, int height)
{
    (void)window;
    glViewport(0, 0, width, height);
}

void Window::ScrollCallback(
    GLFWwindow* window,
    double xOffset,
    double yOffset)
{
    (void)xOffset;

    Window* self = static_cast<Window*>(glfwGetWindowUserPointer(window));
    if (self == nullptr) return;

    // 한 frame 사이 여러 scroll event가 발생할 수 있으므로 누적한다.
    self->m_ScrollOffset += yOffset;
}
