#pragma once

#include<memory>
#include "core/Window.h"
#include "graphics/Shader.h"
#include "graphics/VertexBuffer.h"
#include "graphics/VertexArray.h"


class Window;
class EngineApp{
public:
    void Run();
private:
    void Init();
    void MainLoop();
    void Shutdown();


    float m_LastFrameTime; // 
    std::shared_ptr<Window> m_Window;

    // Shader / VAO / VBO는 GL Context 생성 후 만들어야 함
    // 생성자에서 바로 만들지 않는다.
    // Window init후 EngineApp Init에서 생성
    // 따라서 포인터를 사용해야 한다.
    std::shared_ptr<Shader> m_Shader;

    std::unique_ptr<VertexBuffer> m_VertexBuffer;
    std::unique_ptr<VertexArray> m_VertexArray;
};