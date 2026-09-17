#include "systems/TransformSystemModule.h"

#include "components/TransformComponents.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>



TransformSystemModule::TransformSystemModule(flecs::world& world)
{
    world.module<TransformSystemModule>();
    RegisterObserver(world);
    RegisterSystem(world);
}

void TransformSystemModule::RegisterObserver(flecs::world& world)
{
    // 현재는 매 프레임 계산하므로 별도 dirty observer가 필요하지 않다.
    (void)world;
}

void TransformSystemModule::RegisterSystem(flecs::world& world)
{
    world.system<const Position, const Rotation, const Scale, TransformMatrix>(
        "UpdateLocalTransform")
        .each([](const Position& position,
                 const Rotation& rotation,
                 const Scale& scale,
                 TransformMatrix& matrix)
        {
            const glm::mat4 translation = glm::translate(
                glm::mat4(1.0F),
                static_cast<const glm::vec3&>(position));
            const glm::mat4 rotationMatrix = glm::mat4_cast(
                glm::quat(static_cast<const glm::vec3&>(rotation)));
            const glm::mat4 scaleMatrix = glm::scale(
                glm::mat4(1.0F),
                static_cast<const glm::vec3&>(scale));

            static_cast<glm::mat4&>(matrix) =
                translation * rotationMatrix * scaleMatrix;
        });
}
