#include "systems/RenderSystemModule.h"

#include "components/RenderComponents.h"
#include "components/TransformComponents.h"

#include "graphics/Camera.h"
#include "graphics/Renderer.h"

#include "RenderContext.h"

#include <vector>

namespace
{

struct RenderItem
{
    MeshFilter meshFilter;
    MeshRenderer meshRenderer;
    glm::mat4 model;
};

}

RenderSystemModule::RenderSystemModule(
    flecs::world& world)
{
    world.module<RenderSystemModule>();

    RegisterSystem(world);
}

void RenderSystemModule::RegisterSystem(
    flecs::world& world)
{
    flecs::world* worldPointer =
        &world;

    world
        .system<
            const MeshFilter,
            const MeshRenderer,
            const TransformMatrix>(
            "RenderSystem")

        .kind(flecs::PreStore)

        .term_at(2)
        .second<World>()

        .run(
            [worldPointer](
                flecs::iter& it)
            {
                const RenderContext* context =
                    worldPointer
                        ->try_get<RenderContext>();

                if (!context ||
                    !context->renderer ||
                    !context->camera)
                {
                    return;
                }


                std::vector<RenderItem> items;


                while (it.next())
                {
                    auto meshFilters =
                        it.field<const MeshFilter>(
                            0);

                    auto meshRenderers =
                        it.field<const MeshRenderer>(
                            1);

                    auto worldMatrices =
                        it.field<const TransformMatrix>(
                            2);


                    for (auto i : it)
                    {
                        if (!meshRenderers[i].visible)
                        {
                            continue;
                        }

                        items.push_back({
                            meshFilters[i],
                            meshRenderers[i],
                            static_cast<
                                const glm::mat4&>(
                                worldMatrices[i])
                        });
                    }
                }


                // --------------------------------
                // 1. Shadow pass
                // --------------------------------

                context
                    ->renderer
                    ->BeginShadowPass();

                for (const RenderItem& item :
                     items)
                {
                    context
                        ->renderer
                        ->DrawShadow(
                            item.model,
                            item.meshFilter);
                }

                context
                    ->renderer
                    ->EndShadowPass();


                // --------------------------------
                // 2. Main pass
                // --------------------------------

                const glm::mat4 view =
                    context
                        ->camera
                        ->GetViewMatrix();

                const glm::mat4 projection =
                    context
                        ->camera
                        ->GetProjectionMatrix();

                const glm::vec3 cameraPosition =
                    context
                        ->camera
                        ->GetPosition();


                for (const RenderItem& item :
                     items)
                {
                    context
                        ->renderer
                        ->Draw(
                            item.model,
                            item.meshFilter,
                            item.meshRenderer,
                            view,
                            projection,
                            cameraPosition);
                }
            });
}