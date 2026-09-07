#include "RenderSystem.h"

#include "Camera.h"
#include "RenderContext.h"
#include "Renderable.h"
#include "Renderer.h"
#include "Transform.h"

namespace PoseLink {

RenderSystem::RenderSystem(flecs::world& world)
{
    // RenderSystem을 flecs module로 등록
    world.module<RenderSystem>();

    flecs::world* worldPtr = &world;

    world.system<const Transform, const Renderable>("RenderSystem")
        .kind(flecs::PreStore)
        .each(
            [worldPtr](
                const Transform& transform,
                const Renderable& renderable)
            {
                const RenderContext* context =
                    worldPtr->try_get<RenderContext>();

                if (!context ||
                    !context->renderer ||
                    !context->camera)
                {
                    return;
                }

                const glm::mat4 view =
                    context->camera->GetViewMatrix();

                const glm::mat4 projection =
                    context->camera->GetProjectionMatrix();

                context->renderer->Draw(
                    transform,
                    renderable,
                    view,
                    projection
                );
            }
        );
}

} // namespace PoseLink