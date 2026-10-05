#include "Entity.h"
#include "components/TransformComponents.h"
#include "scene/Scene.h"
#include "scene/SceneManager.h"
#include "systems/TransformSystemModule.h"
#include "TestSupport.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace
{
constexpr float kTolerance = 1.0e-5F;
const glm::quat kIdentity{1.0F, 0.0F, 0.0F, 0.0F};

void RequireMatrix(const glm::mat4& actual, const glm::mat4& expected, const std::string& label)
{
    for (int column = 0; column < 4; ++column)
        for (int row = 0; row < 4; ++row)
            RequireNear(actual[column][row], expected[column][row], kTolerance,
                label + " [" + std::to_string(column) + "][" + std::to_string(row) + "]");
}

void RequireQuaternion(const glm::quat& actual, const glm::quat& expected, const std::string& label)
{
    RequireNear(glm::length(actual), 1.0, kTolerance, label + " unit length");
    // 같은 방향의 q와 -q를 모두 허용한다. 저장 부호보다 회전 방향을 검사한다.
    RequireNear(std::abs(glm::dot(actual, glm::normalize(expected))), 1.0, kTolerance,
        label + " orientation");
}

void RequireQuaternionComponents(const glm::quat& actual, const glm::quat& expected, const std::string& label)
{
    RequireNear(actual.w, expected.w, 0.0, label + " w");
    RequireNear(actual.x, expected.x, 0.0, label + " x");
    RequireNear(actual.y, expected.y, 0.0, label + " y");
    RequireNear(actual.z, expected.z, 0.0, label + " z");
}

glm::mat4 ReferenceMatrix(const glm::vec3& position, const glm::quat& rotation, const glm::vec3& scale)
{
    return glm::translate(glm::mat4(1.0F), position) * glm::mat4_cast(glm::normalize(rotation)) *
        glm::scale(glm::mat4(1.0F), scale);
}

std::array<glm::quat, 3> InvalidRotations()
{
    return {
        glm::quat{0.0F, 0.0F, 0.0F, 0.0F},
        glm::quat{1.0F, std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F},
        glm::quat{1.0F, 0.0F, std::numeric_limits<float>::infinity(), 0.0F}};
}

void CheckDefaultsAndNormalization(Scene& scene)
{
    RequireQuaternionComponents(Rotation{}, kIdentity, "default Rotation");
    RequireQuaternionComponents(Entity{}.GetLocalRotation(), kIdentity, "invalid Entity rotation");
    Entity withoutRotation{scene.GetWorld().entity()};
    RequireQuaternionComponents(withoutRotation.GetLocalRotation(), kIdentity, "missing rotation pair");
    Entity entity = scene.CreateEntity("Normalization");
    RequireQuaternionComponents(entity.GetLocalRotation(), kIdentity, "Scene rotation default");

    const glm::quat expected = glm::quat(glm::vec3{0.3F, -0.7F, 0.5F});
    // 제곱합을 float로 바로 계산하면 overflow 또는 underflow가 생기는 입력도 같은 방향이어야 한다.
    for (const float magnitude : {7.0F, 1.0e30F, 1.0e-30F})
    {
        const glm::quat input = expected * magnitude;
        RequireQuaternion(Rotation{input}, expected, "Rotation normalizes finite input");
        entity.SetLocalRotation(input);
        RequireQuaternion(entity.GetLocalRotation(), expected, "setter normalizes finite input");
    }
    const glm::quat tinyIdentity{std::numeric_limits<float>::denorm_min(), 0.0F, 0.0F, 0.0F};
    RequireQuaternion(Rotation{tinyIdentity}, kIdentity, "smallest nonzero input");

    TransformSystemModule::UpdateWorldTransforms(scene.GetWorld());
    const glm::quat before = entity.GetLocalRotation();
    const glm::mat4 worldBefore = entity.GetWorldMatrix();
    for (const glm::quat& invalid : InvalidRotations())
    {
        ExpectThrows<std::invalid_argument>([&] { (void)Rotation{invalid}; }, "invalid Rotation rejected");
        ExpectThrows<std::invalid_argument>([&] { entity.SetLocalRotation(invalid); }, "invalid setter rejected");
        RequireQuaternionComponents(entity.GetLocalRotation(), before, "rejected setter preserves Local state");
        RequireMatrix(entity.GetWorldMatrix(), worldBefore, "rejected setter preserves World cache");
        TransformSystemModule::UpdateWorldTransforms(scene.GetWorld());
        RequireMatrix(entity.GetWorldMatrix(), worldBefore, "rejected setter preserves subsequent World pose");
    }
    entity.Destroy();
    RequireQuaternionComponents(entity.GetLocalRotation(), kIdentity, "destroyed Entity rotation");
}

void CheckComposition()
{
    const glm::vec3 position{0.8F, -1.2F, 0.4F};
    const glm::vec3 scale{1.5F, 0.7F, 2.0F};
    const float halfPi = std::acos(-1.0F) * 0.5F;
    // Euler의 pitch 특이 자세 양쪽에서도 quaternion이 나타내는 전체 방향을 그대로 행렬에 반영한다.
    for (const float pitch : {halfPi - 1.0e-5F, halfPi, halfPi + 1.0e-5F})
    {
        const glm::quat rotation = glm::quat(glm::vec3{0.4F, pitch, -0.8F});
        const glm::mat4 expected = ReferenceMatrix(position, rotation, scale);
        RequireMatrix(TransformSystemModule::ComposeLocalMatrix(position, rotation, scale), expected,
            "near singular pitch preserves matrix");
        RequireMatrix(TransformSystemModule::ComposeLocalMatrix(position, -rotation, scale), expected,
            "opposite quaternion sign preserves matrix");
        RequireMatrix(TransformSystemModule::ComposeLocalMatrix(position, rotation * 4.0F, scale), expected,
            "nonunit composition preserves matrix");
    }
    for (const glm::quat& invalid : InvalidRotations())
        ExpectThrows<std::invalid_argument>(
            [&] { (void)TransformSystemModule::ComposeLocalMatrix(position, invalid, scale); },
            "invalid composition rejected");
}

void CheckHierarchy(Scene& scene)
{
    Entity root = scene.CreateEntity("HierarchyRoot");
    Entity child = scene.CreateEntity("HierarchyChild");
    Entity grandchild = scene.CreateEntity("HierarchyGrandchild");
    Entity grouping{scene.GetWorld().entity("Grouping").child_of(root.GetHandle())};
    child.SetParent(grouping);
    grandchild.SetParent(child);

    const glm::vec3 rootPosition{0.2F, -0.3F, 0.7F};
    const glm::vec3 childPosition{0.8F, 0.1F, -0.4F};
    const glm::vec3 grandchildPosition{-0.2F, 0.4F, 0.1F};
    const glm::quat childRotation = glm::quat(glm::vec3{-0.6F, 0.2F, 0.3F});
    const glm::quat grandchildRotation = glm::quat(glm::vec3{0.1F, -0.5F, 0.8F});
    root.SetLocalPosition(rootPosition);
    child.SetLocalPosition(childPosition);
    child.SetLocalRotation(childRotation);
    grandchild.SetLocalPosition(grandchildPosition);
    grandchild.SetLocalRotation(grandchildRotation);
    const float halfPi = std::acos(-1.0F) * 0.5F;
    glm::mat4 previousWorld = grandchild.GetWorldMatrix();
    for (const float pitch : {halfPi - 1.0e-5F, halfPi, halfPi + 1.0e-5F})
    {
        const glm::quat rootRotation = glm::quat(glm::vec3{0.3F, pitch, -0.4F});
        root.SetLocalRotation(rootRotation);
        RequireMatrix(grandchild.GetWorldMatrix(), previousWorld, "setter waits for World update");
        TransformSystemModule::UpdateWorldTransforms(scene.GetWorld());
        const glm::mat4 rootExpected = ReferenceMatrix(rootPosition, rootRotation, glm::vec3{1.0F});
        const glm::mat4 childExpected = rootExpected * ReferenceMatrix(childPosition, childRotation, glm::vec3{1.0F});
        const glm::mat4 grandchildExpected = childExpected *
            ReferenceMatrix(grandchildPosition, grandchildRotation, glm::vec3{1.0F});
        RequireMatrix(root.GetWorldMatrix(), rootExpected, "root World rotation");
        RequireMatrix(grouping.GetWorldMatrix(), rootExpected, "grouping propagates parent World");
        RequireMatrix(child.GetWorldMatrix(), childExpected, "child World pose");
        RequireMatrix(grandchild.GetWorldMatrix(), grandchildExpected, "grandchild World pose");
        previousWorld = grandchildExpected;
    }
}

void CheckCorruptedComponent()
{
    flecs::world world;
    Entity entity{world.entity()
        .set<Position, Local>(Position{})
        .set<Rotation, Local>(Rotation{})
        .set<Scale, Local>(Scale{})
        .set<TransformMatrix, Local>(TransformMatrix{})
        .set<TransformMatrix, World>(TransformMatrix{})};
    const glm::quat valid = glm::quat(glm::vec3{0.2F, 0.4F, -0.6F});
    entity.SetLocalRotation(valid);
    TransformSystemModule::UpdateWorldTransforms(world);
    const glm::mat4 worldBefore = entity.GetWorldMatrix();
    const glm::mat4 localBefore = entity.GetHandle().get<TransformMatrix, Local>();
    for (const glm::quat& invalid : InvalidRotations())
    {
        // mutable ECS 접근은 생성자 검증을 우회한다. 합성 단계가 거부해 행렬 캐시에 NaN을 남기지 않아야 한다.
        Rotation& stored = entity.GetHandle().get_mut<Rotation, Local>();
        stored.w = invalid.w;
        stored.x = invalid.x;
        stored.y = invalid.y;
        stored.z = invalid.z;
        ExpectThrows<std::invalid_argument>([&] { TransformSystemModule::UpdateWorldTransforms(world); },
            "corrupted ECS rotation rejected before matrix generation");
        RequireMatrix(entity.GetWorldMatrix(), worldBefore, "corrupted rotation preserves World cache");
        RequireMatrix(entity.GetHandle().get<TransformMatrix, Local>(), localBefore,
            "corrupted rotation preserves Local matrix cache");
        entity.SetLocalRotation(valid);
    }
}
}

/**
 * @brief quaternion Local 회전의 저장 계약과 계층 World 행렬 전파를 확인한다.
 * @details 정규화, 잘못된 입력 거부, q와 -q의 방향 동등성 및 pitch 90도 부근의 자세 보존을 검사한다.
 * SceneManager는 World보다 먼저 파괴돼 계층을 정리하며, 이 테스트는 OpenGL context를 만들지 않는다.
 */
int main()
{
    try
    {
        flecs::world world;
        world.import<TransformSystemModule>();
        SceneManager scenes(world);
        scenes.LoadScene<Scene>();
        scenes.OnUpdate(0.0F);
        Scene* scene = scenes.GetActiveScene();
        Require(scene != nullptr, "active Scene created");
        CheckDefaultsAndNormalization(*scene);
        CheckComposition();
        CheckHierarchy(*scene);
        CheckCorruptedComponent();
        std::cout << "Quaternion transform normalization, rejection and hierarchy checks passed\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
