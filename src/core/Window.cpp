#include "core/Window.h"

// OpenGL 함수 포인터를 GLAD가 관리하기에 먼저 include
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>


Window::Window(int width, int height, const std::string &title):
m_LogicalWidth(width), m_LogicalHeight(height), m_PhysicalWidth(width),m_PhysicalHeight(height),m_Title(title)
{
}

bool Window::Init(){
    // 1. GLFW 초기화
    // GFLW Window / Context 관련 기능을 사용할 수 있도록 초기화 시킨다.
    GLFWwindow* window = nullptr;
    if(!glfwInit()){
        std::cerr << "Failed to Initialize GFLW\n";
        return false;
    }

    // 2. OpenGL Context 설정

    // OpenGL 3.3 사용
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);

    glfwWindowHint(GLFW_CLIENT_API,GLFW_OPENGL_API);

    // 3. GLFW 창 생성
    window = glfwCreateWindow(
        m_LogicalWidth,
        m_LogicalHeight,
        m_Title.c_str(),
        nullptr,
        nullptr)
    ;

    if(window == nullptr){
        std::cerr << "Failed to Create GLFW Window\n";
        glfwTerminate();
        return false;
    };

    // GLFWwindow를 노출하지 않고, void*로 추상화하여 저장한다.
    m_WindowHandle = static_cast<void*>(window);

    // 4. OpenGL Context 활성화
    // glClear, glDrawArrays 등 OpenGL 함수를 호출하기 위해서는
    // OpenGL Context가 활성화되어 있어야 한다.

    // OpenGL 명령은 현재 Thread에 활성화된 Context에만 영향을 준다.
    glfwMakeContextCurrent(window);


    // 5. GLAD 초기화

    // OpenGL 함수들은 실행 환경의 GPU 드라이버에 따라 지원 여부가 달라진다.
    // GLAD는 OpenGL 함수 포인터를 관리하는 라이브러리로
    // 실행 환경의 GPU 드라이버가 지원하는 OpenGL 함수 포인터를 GLAD가 관리하도록 초기화한다.

    // glfwGetProcAddress: OpenGL 함수 포인터를 가져오는 함수

    // gladLoadGLLoader: GLAD 초기화 함수 : 조회한 주소를 GLAD에 등록한다.

    if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)){
        std::cerr << "Failed to Initialize GLAD\n";
        glfwDestroyWindow(window);
        m_WindowHandle = nullptr;
        glfwTerminate();
        return false;
    }

    // 6. 논리 크기 확인
    // 실제 Window System 상태를 기준으로 다시 받는다
    glfwGetWindowSize(
        window,
        &m_LogicalWidth,
        &m_LogicalHeight
    );


    // 7. 실제 FrameBuffer 크기 확인
    
    // OpenGL이 실제로 그리는 픽셀 영역의 크기.

    glfwGetFramebufferSize(
        window,
        &m_PhysicalWidth,
        &m_PhysicalHeight
    );

    // 8. ViewPort 설정
    // FrameBuffer의 어느 영역에 렌더링할지 지정한다.
    // 실제 FrameBuffer 사이즈 사용

    glViewport(
        0,0,
        m_PhysicalWidth, m_PhysicalHeight
    );

    // 9. Callback에서 Window 객체를 찾을 수 있게 연결
    // GLFW Callback의 함수 형태는 내가 정할 수 없다
    
    // Callback 내부에서 C++ Window 객체를 찾기 위해, GLFW Window에 this를 등록한다
    glfwSetWindowUserPointer(window,this);
    // void Callback(GLFWwindow*, int, int); 이런 형식인데
    // Window* this를 전달할 수가 없으므로, GLFW Widnow안에 this 저장하고, 꺼낼때 쓴다.

    // 렌더링 크기 변경 시 호출 됨.
    glfwSetFramebufferSizeCallback(
        window,
        [](GLFWwindow* glfwWindow, int w, int h){
            Window* self = static_cast<Window*>(
                glfwGetWindowUserPointer(glfwWindow) // 이렇게 꺼내 쓴다.
            );

            if(self = nullptr) return;

            self->m_PhysicalWidth = w;
            self->m_PhysicalHeight = h;
            
            // FrameBuffer 크기가 변경됨 -> ViewPort도 변경
            glViewport(
                0,
                0,
                w,
                h
            );
        }
    );

    // VSync 활성화

    // SwapBuffer가 모니터와 동기화
    glfwSwapInterval(1);

    return true;
}

bool Window::ShouldClose(){
    GLFWwindow* window = static_cast<GLFWwindow*>(m_WindowHandle);

    if(window == nullptr) return true;

    return glfwWindowShouldClose(window) == GLFW_TRUE;
}

void Window::PollEvents(){
    // Event 처리
    glfwPollEvents();
}

void Window::SwapBuffers(){
    GLFWwindow* window = static_cast<GLFWwindow*>(m_WindowHandle);
    if(window == nullptr) return;

    // OpenGL의 Back Buffer와 Front Buffer 교환
    glfwSwapBuffers(window);
}

void Window::Shutdown(){
    GLFWwindow* window = static_cast<GLFWwindow*>(m_WindowHandle);
    if(window != nullptr){
        glfwDestroyWindow(window);
        m_WindowHandle = nullptr;
    }

    glfwTerminate();
}

void* Window::GetNativeWindow(){
    // 외부에서 Imgui 같은 라이브러리에, GLFWwindow*를 넘겨야 하는 경우
    return m_WindowHandle;
}

bool Window::IsMinimized(){
    GLFWwindow* window = static_cast<GLFWwindow*>(m_WindowHandle);
    if(window == nullptr) return true;

    //GLFW가 관리하는 Widnow 상태 기준으로 최소화 여부 판단
    return glfwGetWindowAttrib(
        window,
        GLFW_ICONIFIED
    ) == GLFW_TRUE;
}