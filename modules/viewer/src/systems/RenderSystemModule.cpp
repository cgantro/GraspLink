#include "systems/RenderSystemModule.h"

#include "graphics/Camera.h"
#include "RenderContext.h"
#include "graphics/Renderer.h"
#include "components/TransformComponents.h"

RenderSystemModule::RenderSystemModule(flecs::world& world)
{
    // 렌더 시스템 등록은 이 모듈이 단독으로 책임진다.
    world.module<RenderSystemModule>();
    RegisterSystem(world);
}

void RenderSystemModule::RegisterSystem(flecs::world& world)
{
    flecs::world* worldPtr = &world;

    world.system<const Transform, const Renderable>("RenderSystem")
        .kind(flecs::PreStore)
        .each([worldPtr](const Transform& transform,
                         const Renderable& renderable)
        {
            const RenderContext* context =
                worldPtr->try_get<RenderContext>();
            if (!context || !context->renderer || !context->camera)
            {
                return;
            }

            context->renderer->Draw(
                transform,
                renderable,
                context->camera->GetViewMatrix(),
                context->camera->GetProjectionMatrix());
        });
}
