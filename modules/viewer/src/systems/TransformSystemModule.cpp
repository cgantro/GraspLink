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
    // 완전한 Local TRS가 없는 부모는 항등 Local로 조상 변환을 전달한다. 일부 TRS만 있어도 같은 처리다.
    const glm::mat4 local = hasTransform ? TransformSystemModule::ComposeLocalMatrix(
        entity.get<Position, Local>(), entity.get<Rotation, Local>(), entity.get<Scale, Local>())
        : glm::mat4(1.0F);
    if (hasTransform)
        entity.set<TransformMatrix, Local>(TransformMatrix{local});

    // 배열/생성 순서와 무관하게 부모부터 계산해 자식의 Local 좌표를 Scene 기준으로 옮긴다.
    const flecs::entity parent = entity.parent();
    const glm::mat4 parentWorld = parent.id() != 0 && parent.is_alive() && parent != entity
        ? UpdateEntityWorldTransform(parent, cache)
        : glm::mat4(1.0F);
    const glm::mat4 world = parentWorld * local;
    // grouping node에도 파생 World 행렬을 저장한다. 물리가 자식의 World pose를 Local로 되돌릴 때 필요하다.
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
    // 먼저 handle을 모은 뒤 갱신한다. grouping 부모에 Component를 추가하며 query 순회 구조가 바뀔 수 있다.
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
    // 한 번의 갱신 안에서만 공유 부모를 재사용한다. 다음 호출은 변경된 Local TRS를 다시 읽는다.
    for (flecs::entity entity : transformedEntities)
        UpdateEntityWorldTransform(entity, cache);
}
