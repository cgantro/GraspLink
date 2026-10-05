#include "systems/TransformSystemModule.h"

#include "components/TransformComponents.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <unordered_map>
#include <vector>

namespace
{
using WorldTransformCache = std::unordered_map<flecs::entity_t, glm::mat4>;

glm::mat4 UpdateEntityWorldTransform(flecs::entity entity, WorldTransformCache& cache)
{
    if (!entity.is_alive())
        return glm::mat4(1.0F);

    const flecs::entity_t id = entity.id();
    if (const auto cached = cache.find(id); cached != cache.end())
        return cached->second;

    const bool hasTransform = entity.has<Position, Local>() &&
        entity.has<Rotation, Local>() && entity.has<Scale, Local>();
    // 완전한 TRS가 없으면 해당 노드의 Local 기여는 항등 행렬이다. 부분 TRS도 골라 쓰지 않는다.
    const glm::mat4 local = hasTransform ? TransformSystemModule::ComposeLocalMatrix(
        entity.get<Position, Local>(), entity.get<Rotation, Local>(), entity.get<Scale, Local>())
        : glm::mat4(1.0F);
    if (hasTransform)
        entity.set<TransformMatrix, Local>(TransformMatrix{local});

    // GLM 열 벡터에서 부모 World를 왼쪽에 곱해 자식 Local 점을 Scene 좌표로 보낸다.
    const flecs::entity parent = entity.parent();
    const glm::mat4 parentWorld = parent.id() != 0 && parent.is_alive() && parent != entity
        ? UpdateEntityWorldTransform(parent, cache)
        : glm::mat4(1.0F);
    const glm::mat4 world = parentWorld * local;
    // TRS 없는 grouping 부모도 누적 World를 보관해야 물리가 자식 pose를 부모 Local로 되돌릴 수 있다.
    entity.set<TransformMatrix, World>(TransformMatrix{world});
    cache.emplace(id, world);
    return world;
}
}

glm::mat4 TransformSystemModule::ComposeLocalMatrix(
    const glm::vec3& position,
    const glm::vec3& rotationRadians,
    const glm::vec3& scale)
{
    // 열 벡터에 Scale → 회전 → 위치를 적용한다. GLM은 입력 Euler [rad]를 quaternion으로 해석한다.
    return glm::translate(glm::mat4(1.0F), position) *
        glm::mat4_cast(glm::quat(rotationRadians)) *
        glm::scale(glm::mat4(1.0F), scale);
}

TransformSystemModule::TransformSystemModule(flecs::world& world)
{
    world.module<TransformSystemModule>();
}

void TransformSystemModule::UpdateWorldTransforms(flecs::world& world)
{
    // 시작 대상은 완전한 TRS와 두 행렬 pair를 가진 Entity다. 부모로만 방문한 grouping node는 여기서 제외된다.
    // 먼저 handle을 모은 뒤 갱신한다. 재귀 중 grouping 부모에 World 행렬을 추가하면 query 구성이 바뀔 수 있다.
    std::vector<flecs::entity> transformedEntities;
    world.each([&](flecs::entity entity)
    {
        if (entity.has<Position, Local>() && entity.has<Rotation, Local>() &&
            entity.has<Scale, Local>() && entity.has<TransformMatrix, Local>() &&
            entity.has<TransformMatrix, World>())
        {
            transformedEntities.push_back(entity);
        }
    });

    WorldTransformCache cache;
    cache.reserve(transformedEntities.size());
    // 이 호출 안에서 공유 부모를 한 번만 계산한다. 다음 fixed step/render 전 호출은 Local 변경을 다시 읽는다.
    for (flecs::entity entity : transformedEntities)
        UpdateEntityWorldTransform(entity, cache);
}
