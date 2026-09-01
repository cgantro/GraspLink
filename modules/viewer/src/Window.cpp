
#include "Window.h"

#include <stdexcept>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

namespace PoseLink {

Window::Window(const Properties& properties){
    Init(properties);
};

// 소멸자
Window::~Window(){
    Shutdown();
}

void Window::Init(const Properties properties){
    // 1. GLFW 초기화
    // Window 생성, 키보드/마우스 입력, GL Context 생성
    // 위의 작업들을 OS별 코드로 대신 처리해주는 라이브러리
    // Window -> Win32API, Linux -> X11/WayLand와 같은 시스템을 사용한다

    // GLFW가 OS기능을 사용할 준비
    if(glfwInit() == GLFW_FALSE) throw std::runtime_error(
        "Failed to Init GLFW"
    );

    // 2. GL Context의 버전을 요청한다.
    // 아래 조건의 Context를 만들어달라는 요청
    // 3.3버전 사용
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
    glfwWindowHint(GLFW_VERSION_MINOR,3);

    // 오래된 방식인 glBegin(), end()와 같은 레거시 API를 제외한다
    // 왜 제거? -> 정점을 정의할 때 마다 CPU가 GPU에게 데이터를 전달한다.
    // 1. 이 과정에서 10만개의 정점을 그릴 때 -> CPU - GPU 10만번 통신(오버헤드)
    // 2. GPU 하드웨어 구조 변화
    //      셰이더 코드를 직접 작성하여, 연산, 효과등을 제어 가능하다.
    // 3. 모바일 및 최신 API와의 호환성
    // VAO, VBO, Shader등을 사용하는 현대적인 방식 사용
    // 정점 데이터를 한번에 담아 GPU메모리에 전달 -> 통신 횟수가 줄어든다.
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // 3. 실제 Window 생성

    // glfwCreateWindow()
    // 1. OS의 실제 Window
    // 2. 해당 Window와 연결된 GL Context
    //      어떤 Shader인지, VAO 바인드인지 등등의 GL 상태를 보관하는 환경

    m_Handle = glfwCreateWindow(
        properties.width,
        properties.height,
        properties.title.c_str(),
        nullptr, // fullscrean 모니터 지정 x
        nullptr // 다른 window와 context 공유 x
    );

    if(m_Handle == nullptr){
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW Window");
    }

    /*
        4. GL Context를 현재 Thread에 연결한다.
        GL 함수들은 어느 Window에 그릴지를 항상 파라미터로 전달하지 않음
        대신 현재 Thread에 연결된 context 사용

        따라서 context를 현재 Thread에 연결해야함
    */
    glfwMakeContextCurrent(m_Handle);

    /*
        5. GLAD -> OpenGL 함수 주소 로딩
        OpenGL 함수들은 고정된 주소를 가지지 않는다.
        
        그래픽 드라이버 내부에 구현이 있는데, 각 제조사마다 주소도 다르고..
        glfwGetProcAddress가 OpenGL Context 기준으로 함수 주소를 찾아준다.
        
        GLAD는 이 함수를 이용해 필요한 GL 함수 포인터들을 채운다.
    */

    if(!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))){
        Shutdown();
        throw std::runtime_error("Failed to Init GLAD");
    }

    /*
        6. Window 크기 변경 Callback
        창 크기를 바꾸면 렌더링 FrameBuffer의 크기도 바뀌기 때문에 호출 함수 CallBack 등록
    */
    glfwSetFramebufferSizeCallback(m_Handle,FramebufferSizeCallback);

    /*
        7. Vsync 설정
        Swap Interval = 1 -> 모니터 Vertical Refresh에 맞춰 Swap
        Swap Interval = 0 -> 가능한 즉시 Swap
     */
    glfwSwapInterval(properties.vsync ?  1 : 0);

    // 8. 실제 FrameBuffer 크기
    int framebufferWidth = 0;
    int framebufferHeight = 0;

    /*
        Window의 논리적 크기와 GPU가 렌더링하는 FrameBuffer 크기는 다를 수 있다(HiDPi 환경)
        물리적인 화면 크기에 비해 매우 높은 해상도(많은 픽셀)를 가진 디스플레이 환경이다.

        만약 800 x 600 크기의 창을 만들고 그대로 glViewport(0, 0, 800, 600);을 설정하면, 
        실제 물리 픽셀은 1600 x 1200이기 때문에 화면의 왼쪽 아래 4분의 1 영역에만 그림이 그려지는 현상이 나타남

    */

    // 따라서 실제 그려지는 FrameBuffer 크기를 가져온다.
    glfwGetFramebufferSize(m_Handle,&framebufferWidth,&framebufferHeight);

    // OpenGL의 최종 좌표를 실제 화면의 어느 픽셀 영역에 매핑할 것인지 지정한다.
    // 전체 사용
    /*
        동작원리 : NDC -> 픽셀 좌표 변환
        정점들은 모두 정규화된 기기좌표공간에 존재한다(-1.0 ~ 1.0)
        이를 픽셀 좌표로 변환한다./
    */
    glViewport(0,0,framebufferWidth,framebufferHeight);

}

void Window::Shutdown(){
    if(m_Handle != nullptr){
        // 실제 Window 제거
        glfwDestroyWindow(m_Handle);
        m_Handle = nullptr; // Dangling 방지
    }

    // GLFW가 내부적으로 사용하던 OS 자원 해제
    glfwTerminate();
}

bool Window::ShouldClose() const{
    // 종료해야하는가
    return glfwWindowShouldClose(m_Handle) == GLFW_TRUE;
}

void Window::PollEvents() const{
    // OS 이벤트 큐에 쌓여 있는 이벤트 처리

    // GLFW는 이벤트들을 읽어서 등록된 callback 호출
    // MainLoop에서 이 함수를 계속 호출하지 않는다면, Window가 입력에 반응하지 않는다.

    glfwPollEvents();
}

void Window::SwapBuffers() const{
    // OpenGL Window는 일반적으로 Double Buffering 사용
    // 1. Front Buffer -> 현재 모니터에 표시되고 있는 이미지
    // 2. Back Buffer -> GPU가 다음 Frame을 그리고 있는 것

    // 렌더링 과정
    // Back Buffer에 프레임 작성 -> swap -> Back Buffer 화면 표시
    glfwSwapBuffers(m_Handle);
}

void Window::FramebufferSizeCallback(GLFWwindow* window, int width, int height){
    // Window 크기 변경 -> FrameBuffer 크기 변경
    // 따라서 렌더링 영역도 새로 맞춤

    glViewport(0,0,width,height);
    
}
};

