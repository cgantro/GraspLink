#include "systems/RenderSystemModule.h"

#include "components/RenderComponents.h"
#include "components/TransformComponents.h"
#include "graphics/Camera.h"
#include "graphics/Renderer.h"
#include "RenderContext.h"

#include <vector>

namespace
{
// Component 값과 GPU 공유 참조를 복사해 두 pass가 같은 frame의 목록/행렬을 사용하게 한다.
// Flecs Component 주소를 보관하지 않으므로 iterator 이동에 영향을 받지 않는다.
struct RenderItem
{
    MeshFilter meshFilter;
    MeshRenderer meshRenderer;
    glm::mat4 model;
};
} // namespace

RenderSystemModule::RenderSystemModule(flecs::world& world)
{
    world.module<RenderSystemModule>();
    RegisterSystem(world);
}

void RenderSystemModule::RegisterSystem(flecs::world& world)
{
    // PreStore에서 OnUpdate 뒤에 그린다. Transform 계산은 자동 phase가 아니므로 ViewerApp이 progress 전에 끝낸다.
    world
        .system<const MeshFilter, const MeshRenderer, const TransformMatrix>(
            "RenderSystem")
        .kind(flecs::PreStore)
        .term_at(2)
        .second<World>()
        .run(
            [](flecs::iter& it)
            {
                // 등록 함수의 임시 World wrapper를 보관하지 않고 현재 실행 World에서 빌린 RenderContext를 읽는다.
                const RenderContext* context =
                    it.world().try_get<RenderContext>();

                if (!context || !context->renderer || !context->camera)
                    return;

                std::vector<RenderItem> items;

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

                // Light 기준 깊이를 먼저 그려 Main pass에서 사용할 그림자 정보를 만든다.
                context->renderer->BeginShadowPass();
                for (const RenderItem& item : items)
                {
                    context->renderer->DrawShadow(
                        item.model,
                        item.meshFilter);
                }
                context->renderer->EndShadowPass();

                // 같은 Model 행렬을 Camera 기준으로 그린다. Material/Texture/조명은 Main pass에서 적용한다.
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

            });
}
