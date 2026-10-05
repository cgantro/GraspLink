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

    // GLFW는 프로세스 전역 상태를 사용한다. 초기화 실패 시 정리할 Window는 아직 없다.
    if (glfwInit() == GLFW_FALSE)
        throw std::runtime_error("Failed to Init GLFW");

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_VISIBLE, properties.visible ? GLFW_TRUE : GLFW_FALSE);

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

    // GPU 자원은 이 Context에서만 쓸 수 있으며 Window보다 먼저 해제한다.
    // GLAD는 현재 Context에서 함수 주소를 조회하므로 적재 전에 Context를 활성화한다.
    glfwMakeContextCurrent(m_Handle);

    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
    {
        // 부분 초기화 실패에서도 생성된 창과 GLFW 전역 상태를 함께 정리한다.
        Shutdown();
        throw std::runtime_error("Failed to Init GLAD");
    }

    glfwSetWindowUserPointer(m_Handle, this);
    glfwSetFramebufferSizeCallback(m_Handle, FramebufferSizeCallback);
    glfwSetScrollCallback(m_Handle, ScrollCallback);

    // 1은 화면 주사와 동기화하고, 0은 프레임 제한 없이 버퍼를 교환한다.
    glfwSwapInterval(properties.vsync ? 1 : 0);

    int framebufferWidth = 0;
    int framebufferHeight = 0;
    glfwGetFramebufferSize(m_Handle, &framebufferWidth, &framebufferHeight);

    // OpenGL viewport 단위는 논리 창 좌표가 아니라 실제 framebuffer 픽셀이다.
    glViewport(0, 0, framebufferWidth, framebufferHeight);
}

void Window::Shutdown()
{
    // Context를 소유한 창을 먼저 없애고 GLFW 프로세스 상태를 종료한다.
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

GLFWwindow* Window::GetNativeHandle() const
{
    return m_Handle;
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
    // HiDPI 배율 변경도 framebuffer 크기 이벤트로 전달되므로 viewport를 픽셀 크기에 맞춘다.
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

    // 여러 callback의 세로 scroll을 합쳐 Controller가 한 번에 소비한다.
    self->m_ScrollOffset += yOffset;
}
