
#pragma once

#include <memory>

namespace PoseLink
{
class Window;
class Renderer;
} // namespace PoseLink

class ViewerApp{
public:
    ViewerApp();
    ~ViewerApp();
    int Run();
private:
    bool Init();
    void MainLoop();
    void Shutdown();
private:
    // ViewerApp이 Window의 소유권을 가짐
    // 생명주기 연결, Window 하나를 단독 소유
    std::unique_ptr<PoseLink::Window> m_Window;
    std::unique_ptr<PoseLink::Renderer> m_Renderer;
};
