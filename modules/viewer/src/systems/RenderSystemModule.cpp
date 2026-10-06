#include "systems/RenderSystemModule.h"

#include "components/RenderComponents.h"
#include "components/TransformComponents.h"
#include "graphics/Camera.h"
#include "graphics/Renderer.h"
#include "RenderContext.h"

#include <vector>

namespace
{
// Component 값과 GPU 자원에 대한 공유 참조를 복사해 그림자 생성과 화면 출력이 같은 프레임 목록 및 행렬을 사용하게 한다.
// Flecs의 임시 Component 메모리 주소를 저장하지 않으므로, 순회 iterator가 다음 항목으로 이동해도 자료가 유효하다.
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
    // Flecs의 PreStore 단계에서 Scene OnUpdate 이후에 그린다. 변환 계산은 자동 단계가 아니므로 ViewerApp이 World 진행 전에 명시적으로 끝낸다.
    world
        .system<const MeshFilter, const MeshRenderer, const TransformMatrix>(
            "RenderSystem")
        .kind(flecs::PreStore)
        .term_at(2)
        .second<World>()
        .run(
            [](flecs::iter& it)
            {
                // system 등록 때 받은 임시 World wrapper를 저장하지 않는다. 실제 실행 중인 Flecs World에서 RenderContext를 빌려 읽는다.
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

                        // 부모 변환까지 누적된 TransformMatrix의 World 값을 복사해 Renderer에 보낸다.
                        // 모델 자체의 Local 행렬을 넘기면 Scene 배치 변환이 빠지기 때문이다.
                        items.push_back({
                            meshFilters[i],
                            meshRenderers[i],
                            static_cast<const glm::mat4&>(worldMatrices[i])});
                    }
                }

                // 먼저 광원 위치에서 본 깊이값을 저장한다. 화면을 그리는 단계에서 이 값으로 물체가 그림자를 받는지 판단한다.
                context->renderer->BeginShadowPass();
                for (const RenderItem& item : items)
                {
                    context->renderer->DrawShadow(
                        item.model,
                        item.meshFilter);
                }
                context->renderer->EndShadowPass();

                // 같은 모델 행렬을 카메라 시점으로 그린다. 이 단계에서 Material 색, Texture와 조명을 적용해 화면 색을 만든다.
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
