#include "systems/TransformSystemModule.h"

#include "components/TransformComponents.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

namespace
{
// Physics가 Render 행렬 갱신보다 먼저 실행될 수 있음 → 현재 Local 값으로 계산
glm::mat4 BuildWorldMatrix(flecs::entity entity)
{
    if (entity.id() == 0 || !entity.is_alive())
        return glm::mat4(1.0F);

    const glm::mat4 local = TransformSystemModule::CalculateLocalMatrix(entity);
    const flecs::entity parent = entity.parent();
    if (parent.id() == 0 || !parent.is_alive() || parent == entity)
        return local;

    return BuildWorldMatrix(parent) * local;
}
}

glm::mat4 TransformSystemModule::ComposeLocalMatrix(
    const glm::vec3& position,
    const glm::vec3& rotationRadians,
    const glm::vec3& scale)
{
    // 적용 순서: Entity 크기 → 회전 → 부모 기준 위치
    const glm::mat4 translation = glm::translate(glm::mat4(1.0F), position);
    const glm::mat4 rotationMatrix = glm::mat4_cast(glm::quat(rotationRadians));
    const glm::mat4 scaleMatrix = glm::scale(glm::mat4(1.0F), scale);
    return translation * rotationMatrix * scaleMatrix;
}

glm::mat4 TransformSystemModule::CalculateLocalMatrix(flecs::entity entity)
{
    if (entity.id() == 0 || !entity.is_alive() || !entity.has<Position, Local>() ||
        !entity.has<Rotation, Local>() || !entity.has<Scale, Local>())
    {
        return glm::mat4(1.0F);
    }

    return ComposeLocalMatrix(
        entity.get<Position, Local>(),
        entity.get<Rotation, Local>(),
        entity.get<Scale, Local>());
}

glm::mat4 TransformSystemModule::CalculateWorldMatrix(flecs::entity entity)
{
    // Fixed Update용: Render 캐시 대신 최신 Entity 자세 계산
    return BuildWorldMatrix(entity);
}

TransformSystemModule::TransformSystemModule(flecs::world& world)
{
    world.module<TransformSystemModule>();
    RegisterObserver(world);
    RegisterSystem(world);
}

void TransformSystemModule::RegisterObserver(flecs::world& world)
{
    // 매 프레임 전체 계산. 변경 감시자 미사용
    (void)world;
}

void TransformSystemModule::RegisterSystem(flecs::world& world)
{
    world.system<const Position, const Rotation, const Scale, TransformMatrix>(
        "UpdateLocalTransform")
        .kind(flecs::OnUpdate)
        .term_at(0).second<Local>()
        .term_at(1).second<Local>()
        .term_at(2).second<Local>()
        .term_at(3).second<Local>()
        .each([](const Position& position,
                 const Rotation& rotation,
                 const Scale& scale,
                 TransformMatrix& local)
        {
            static_cast<glm::mat4&>(local) =
                TransformSystemModule::ComposeLocalMatrix(position, rotation, scale);
        });

    world.system<const TransformMatrix, const TransformMatrix*, TransformMatrix>(
        "UpdateWorldTransform")
        .kind(flecs::OnUpdate)
        .term_at(0).second<Local>()
        .term_at(1).second<World>()
        .term_at(1).parent().cascade()
        .term_at(2).second<World>()
        .each([](const TransformMatrix& local,
                 const TransformMatrix* parentWorld,
                 TransformMatrix& worldMatrix)
        {
            // 부모 먼저 계산 → 자식이 최신 부모 행렬 사용
            static_cast<glm::mat4&>(worldMatrix) = parentWorld
                ? static_cast<const glm::mat4&>(*parentWorld) *
                    static_cast<const glm::mat4&>(local)
                : static_cast<const glm::mat4&>(local);
        });
}
