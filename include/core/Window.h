#pragma once

#include <string>

class Window {
public:
    // 창의 논리적 크기와 제목을 받아 Window 객체를 생성한다.
    // 실제 GLFW 창 생성은 Init()에서 수행한다.
    Window(int width, int height, const std::string& title);

    // GLFW 초기화, 창 생성, OpenGL Context 설정 등을 수행한다.
    // 성공하면 true, 실패하면 false를 반환한다.
    bool Init();

    // 사용자가 창 닫기 버튼을 눌렀는지 확인한다.
    // true면 메인 루프를 종료하면 된다.
    bool ShouldClose();

    // 키보드, 마우스, 창 크기 변경 등의 이벤트를 처리한다.
    void PollEvents();

    // Back Buffer에 그린 결과를 화면에 표시한다.
    // Double Buffering 환경에서 Front/Back Buffer를 교체한다.
    void SwapBuffers();

    // 생성한 창과 GLFW 관련 자원을 정리한다.
    void Shutdown();

    // 내부 Window Handle을 외부에 넘길 때 사용한다.
    // GLFWwindow*를 직접 노출하지 않기 위해 void*로 추상화한 형태다.
    void* GetNativeWindow();

    // 창이 최소화되어 실제 렌더링 가능한 영역이 없는지 확인한다.
    bool IsMinimized();

    // 논리적인 창 너비를 반환한다.
    int GetWidth() const { return m_LogicalWidth; }

    // 논리적인 창 높이를 반환한다.
    int GetHeight() const { return m_LogicalHeight; }

private:
    // 실제 플랫폼/GLFW 창 객체의 포인터.
    // 구현부에서는 일반적으로 GLFWwindow*를 저장한다.
    void* m_WindowHandle = nullptr;

    // 사용자가 보는 논리적인 창 크기.
    //
    // 예:
    // Window 크기 1280 x 720
    //
    // HiDPI 환경에서는 실제 framebuffer 크기와 다를 수 있다.
    int m_LogicalWidth;
    int m_LogicalHeight;

    // 실제 OpenGL이 렌더링하는 framebuffer 크기.
    //
    // 예:
    // 논리 크기:   1280 x 720
    // 물리 크기:   2560 x 1440
    //
    // Retina / HiDPI 환경에서 이런 차이가 생길 수 있다.
    int m_PhysicalWidth;
    int m_PhysicalHeight;

    // 창 제목.
    std::string m_Title;
};