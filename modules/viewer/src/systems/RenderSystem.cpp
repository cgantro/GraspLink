#include "RenderSystem.h"

#include "Camera.h"
#include "Renderer.h"
#include <iostream>

namespace PoseLink
{
RenderSystem::RenderSystem(flecs::world& world)
    :m_Query(world.query<Transform,Renderable>()){

}

void RenderSystem::Render(Renderer& renderer, const Camera& camera){
    const glm::mat4 view = camera.GetViewMatrix();
    const glm::mat4 projection = camera.GetProjectionMatrix();

    /*
        Transfrom + Renderable을 가진 entity만 조회
    */

    // int cnt = 0;
    m_Query.each(
        [&](flecs::entity entity,Transform& t, Renderable& r){

            // ++cnt;

            // std::cout << "render: " << entity.name() << '\n';
            renderer.Draw(t,r,view,projection);

            
        }
    );
}
} // namespace PoseLink
