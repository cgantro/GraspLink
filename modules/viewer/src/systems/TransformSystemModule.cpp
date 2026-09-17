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
    // 현재는 매 프레임 계산하므로 변환 변경 감시자가 필요하지 않다.
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
            // GLM column-major 기준으로 Local = Translation * Rotation * Scale이다.
            const glm::mat4 translation = glm::translate(
                glm::mat4(1.0F),
                static_cast<const glm::vec3&>(position));
            const glm::mat4 rotationMatrix = glm::mat4_cast(
                glm::quat(static_cast<const glm::vec3&>(rotation)));
            const glm::mat4 scaleMatrix = glm::scale(
                glm::mat4(1.0F),
                static_cast<const glm::vec3&>(scale));

            static_cast<glm::mat4&>(local) =
                translation * rotationMatrix * scaleMatrix;
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
            // cascade가 부모부터 처리하므로 이 시점의 parentWorld는 최신 값이다.
            static_cast<glm::mat4&>(worldMatrix) = parentWorld
                ? static_cast<const glm::mat4&>(*parentWorld) *
                    static_cast<const glm::mat4&>(local)
                : static_cast<const glm::mat4&>(local);
        });
}
