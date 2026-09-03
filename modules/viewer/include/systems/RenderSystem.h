#pragma once
#include "Renderable.h"
#include "Transform.h"

#include <flecs.h>

namespace PoseLink
{
class Renderer;
class Camera;

/*
    ECS와 Renderer 연결

    flecs::world -> Transform + Renderable Entity 조회 -> Draw
*/

class RenderSystem{
public:
    /*
        World가 살아있는 동안 사용할 Query 생성
        -> RenderSystem은 world보다 먼저 해제되어야함
    */

    explicit RenderSystem(flecs::world& world);
    // 현재 Cam 기준으로 Renderable Entity 그리기
    void Render(Renderer& renderer, const Camera& camera);
private:
    /*
        매 Frame마다 Query를 다시 정의하지 않고 한 번 생성해서 계속 사용
    */
   flecs::query<Transform,Renderable> m_Query;
};
} // namespace PoseLink
