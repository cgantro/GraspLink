#include "systems/RenderSystemModule.h"

#include "components/RenderComponents.h"
#include "components/TransformComponents.h"
#include "graphics/Camera.h"
#include "graphics/Renderer.h"
#include "RenderContext.h"

RenderSystemModule::RenderSystemModule(flecs::world& world)
{
    world.module<RenderSystemModule>();
    RegisterSystem(world);
}

void RenderSystemModule::RegisterSystem(flecs::world& world)
{
    flecs::world* worldPointer = &world;

    world.system<const MeshFilter, const MeshRenderer, const TransformMatrix>(
        "RenderSystem")
        .kind(flecs::PreStore)
        .term_at(2).second<World>()
        .each([worldPointer](const MeshFilter& meshFilter,
                             const MeshRenderer& meshRenderer,
                             const TransformMatrix& worldMatrix)
        {
            if (!meshRenderer.visible)
            {
                return;
            }

            const RenderContext* context =
                worldPointer->try_get<RenderContext>();
            if (!context || !context->renderer || !context->camera)
            {
                return;
            }

            context->renderer->Draw(
                static_cast<const glm::mat4&>(worldMatrix),
                meshFilter,
                meshRenderer,
                context->camera->GetViewMatrix(),
                context->camera->GetProjectionMatrix());
        });
}
