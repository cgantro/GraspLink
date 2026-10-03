#include "systems/RenderSystemModule.h"

#include "components/RenderComponents.h"
#include "components/TransformComponents.h"
#include "graphics/Camera.h"
#include "graphics/Renderer.h"
#include "RenderContext.h"

#include <vector>

namespace
{
/**
 * @brief 한 frame 동안 Renderer에 전달할 최소 draw data 묶음.
 *
 * @details
 * Flecs iterator가 가리키는 Component 메모리를 pass 사이에 직접 보관하지 않고 값으로 복사한다.
 * 이렇게 모아 두면 같은 visible item 목록을 Shadow Pass와 Main Pass에서 재사용할 수 있다.
 */
struct RenderItem
{
    MeshFilter meshFilter;
    MeshRenderer meshRenderer;
    glm::mat4 model;
};
} // namespace

RenderSystemModule::RenderSystemModule(flecs::world& world)
{
    // Flecs module로 등록하면 import 시 한 번만 system 구성을 설치할 수 있다.
    world.module<RenderSystemModule>();
    RegisterSystem(world);
}

void RenderSystemModule::RegisterSystem(flecs::world& world)
{
    /*
        query 조건:
        - MeshFilter: 어떤 GPU Mesh/index range를 그릴지
        - MeshRenderer: 어떤 Shader/Material로 그릴지
        - (TransformMatrix, World): 최종 World Model Matrix

        flecs::PreStore는 일반 OnUpdate logic과 Transform 계산 이후에 렌더링을 수행하기 위한 phase다.
    */
    flecs::world* worldPointer = &world;

    world
        .system<const MeshFilter, const MeshRenderer, const TransformMatrix>(
            "RenderSystem")
        .kind(flecs::PreStore)
        .term_at(2)
        .second<World>()
        .run(
            [worldPointer](flecs::iter& it)
            {
                const RenderContext* context =
                    worldPointer->try_get<RenderContext>();

                if (!context || !context->renderer || !context->camera)
                    return;

                std::vector<RenderItem> items;

                /*
                    먼저 visible Entity의 draw data를 모은다.
                    Shadow/Main pass가 각각 ECS query를 다시 돌지 않게 하고 두 pass가 같은 frame snapshot을 사용한다.
                */
                while (it.next())
                {
                    auto meshFilters = it.field<const MeshFilter>(0);
                    auto meshRenderers = it.field<const MeshRenderer>(1);
                    auto worldMatrices = it.field<const TransformMatrix>(2);

                    for (auto i : it)
                    {
                        if (!meshRenderers[i].visible) continue;

                        items.push_back({
                            meshFilters[i],
                            meshRenderers[i],
                            static_cast<const glm::mat4&>(worldMatrices[i])});
                    }
                }

                // 1) Light 관점에서 depth만 그려 ShadowMap을 만든다.
                context->renderer->BeginShadowPass();
                for (const RenderItem& item : items)
                {
                    context->renderer->DrawShadow(
                        item.model,
                        item.meshFilter);
                }
                context->renderer->EndShadowPass();

                // 2) 현재 Camera 관점에서 Material/Texture/Lighting을 포함한 Main Pass를 그린다.
                const glm::mat4 view = context->camera->GetViewMatrix();
                const glm::mat4 projection = context->camera->GetProjectionMatrix();
                const glm::vec3 cameraPosition = context->camera->GetPosition();

                for (const RenderItem& item : items)
                {
                    context->renderer->Draw(
                        item.model,
                        item.meshFilter,
                        item.meshRenderer,
                        view,
                        projection,
                        cameraPosition);
                }

                // TODO(FUTURE): Entity 수가 커지면 frustum culling과 draw sorting을 items 생성 단계에 추가한다.
            });
}
