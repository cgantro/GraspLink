#pragma once

class Renderer;
class Camera;

/**
 * @brief RenderSystem이 현재 프레임에 사용할 Renderer와 Camera를 빌려 받는 World singleton이다.
 * @details
 * 이 포인터들은 소유권을 넘기지 않는 borrowed 참조다. ViewerApp이 Renderer와 Camera를 소유하고,
 * 종료 시 World와 이 Context를 먼저 정리한 뒤 소유 객체를 파괴해야 한다. 두 포인터 중 하나라도 null이면
 * RenderSystem은 해당 프레임의 draw를 건너뛴다. World 밖에 복사해 저장하거나 소유 객체보다 오래 보관하지 않는다.
 */
struct RenderContext
{
    /// OpenGL draw 명령을 처리하는 ViewerApp 소유 객체의 비소유 포인터.
    Renderer* renderer = nullptr;

    /// 현재 프레임의 View/Projection 행렬을 제공하는 ViewerApp 소유 객체의 비소유 포인터.
    Camera* camera = nullptr;
};
