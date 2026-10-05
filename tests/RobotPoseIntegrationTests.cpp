#include "Entity.h"
#include "Shader.h"
#include "Window.h"
#include "assets/AssetManager.h"
#include "assets/GltfLoader.h"
#include "assets/GraphicsTypes.h"
#include "assets/PrefabFactory.h"
#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"
#include "scene/Scene.h"
#include "scene/SceneManager.h"
#include "systems/TransformSystemModule.h"
#include "viewer/robotics/RobotTransformAdapter.h"
#include "TestSupport.h"

#include <glm/gtc/quaternion.hpp>

#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace grasplink::robotics;

namespace
{
void RequirePositionNear(const glm::vec3& actual, const glm::vec3& expected, const std::string& label)
{
    // World 위치 [m]를 축별 0.2 mm 허용오차로 비교한다. GLB float 행렬과 FK double 결과 정밀도 차이를 허용한다.
    const auto coordinateLabel = [&](const char* axis, float actualValue, float expectedValue)
    {
        return label + " " + axis + ": actual=" + std::to_string(actualValue) +
            ", expected=" + std::to_string(expectedValue);
    };
    RequireNear(actual.x, expected.x, 2e-4, coordinateLabel("x", actual.x, expected.x));
    RequireNear(actual.y, expected.y, 2e-4, coordinateLabel("y", actual.y, expected.y));
    RequireNear(actual.z, expected.z, 2e-4, coordinateLabel("z", actual.z, expected.z));
}

void ApplyAndCompare(
    flecs::world& world,
    Entity& robotRoot,
    grasplink::robotics::kinematics::RobotKinematics& kinematics,
    grasplink::viewer::robotics::RobotTransformAdapter& adapter,
    const std::array<double, 6>& jointRadians,
    const glm::vec3& rootPosition,
    const glm::quat& rootRotation,
    const std::string& label)
{
    // FK 관절 입력 [rad]에서 나온 ToolFrame Base pose에 Scene root Local TRS를 적용하고 GLB Entity World pose와 비교한다.
    RobotState state;
    state.valid = true;
    state.jointPositionRadians.assign(jointRadians.begin(), jointRadians.end());
    const auto& fk = kinematics.Update(state);
    adapter.Apply(fk);

    robotRoot.SetLocalPosition(rootPosition);
    robotRoot.SetLocalRotation(rootRotation);
    TransformSystemModule::UpdateWorldTransforms(world);

    const Entity toolFrame = robotRoot.FindChildByNameRecursive("ToolFrame");
    Require(static_cast<bool>(toolFrame), label + ": GLB ToolFrame exists");
    const glm::mat4 rootWorld = robotRoot.GetWorldMatrix();
    const glm::mat4 toolWorld = toolFrame.GetWorldMatrix();
    // 비교: GLB에서 생성된 ToolFrame Entity의 실제 world 위치와 CPU FK 결과.
    const glm::vec3 actualWorld = glm::vec3(toolWorld[3]);
    const glm::vec3 expectedWorld = glm::vec3(rootWorld * glm::vec4(
        static_cast<float>(fk.toolFrameInBaseFrame.positionMeters.x),
        static_cast<float>(fk.toolFrameInBaseFrame.positionMeters.y),
        static_cast<float>(fk.toolFrameInBaseFrame.positionMeters.z),
        1.0F));
    RequirePositionNear(actualWorld, expectedWorld, label + ": ToolFrame world position");

    const glm::quat actualOrientation = glm::quat_cast(glm::mat3(toolWorld));
    const auto& expectedRotation = fk.toolFrameInBaseFrame.rotation;
    const glm::quat expectedOrientation = glm::quat_cast(glm::mat3(rootWorld)) * glm::quat{
        static_cast<float>(expectedRotation.w), static_cast<float>(expectedRotation.x),
        static_cast<float>(expectedRotation.y), static_cast<float>(expectedRotation.z)};
    // Quaternion 부호가 반대여도 같은 회전을 뜻하므로 정규화 quaternion 내적의 절댓값으로 방향을 비교한다.
    Require(std::abs(glm::dot(glm::normalize(actualOrientation), glm::normalize(expectedOrientation))) > 0.9999F,
        label + ": ToolFrame world orientation");
}
}

/**
 * @brief CPU FK와 GLB에서 만든 Flecs 관절 계층이 같은 ToolFrame World 자세를 내는지 확인한다.
 * @details 영 자세, 각 관절 단독 회전, 혼합 자세와 Scene root 이동/회전을 비교하고 bind pivot 및 부모 계층
 * 손상 시 RobotTransformAdapter가 거부하는지 확인한다. 숨긴 OpenGL Context는 GLB Mesh 업로드 자원보다 먼저
 * 만들어지고, 이 범위의 AssetManager/Shader GPU 자원이 정리된 뒤 Window 소멸로 마지막에 닫힌다.
 */
int main()
{
    try
    {
        // 수명: 숨긴 Window의 GL context는 AssetManager/Shader가 만든 GPU 리소스보다 오래 살아 있어야 한다.
        Window window(Window::Properties{320, 240, "RobotPoseIntegrationTests", false, false});
        AssetManager assets;
        ModelResource model = GltfLoader::LoadGLB(
            std::filesystem::path("assets") / "HCR12A_2F-85.glb");
        assets.UploadModel(model);
        const auto shader = Shader::CreateFromSource(
            "RobotPoseTest",
            "#version 330 core\nlayout(location=0) in vec3 position; void main(){ gl_Position=vec4(position,1.0); }\n",
            "#version 330 core\nout vec4 color; void main(){ color=vec4(1.0); }\n");

        flecs::world world;
        world.import<TransformSystemModule>();
        SceneManager scenes(world);
        scenes.LoadScene<Scene>();
        scenes.OnUpdate(0.0F);
        Scene* scene = scenes.GetActiveScene();
        Require(scene != nullptr, "active Scene created");

        Entity robotRoot = PrefabFactory::CreateModel(*scene, model, assets, shader);
        robotRoot.SetLocalPosition({1.2F, -0.4F, 0.7F});
        robotRoot.SetLocalRotation(glm::quat{glm::vec3{0.3F, 0.7F, -0.4F}});
        const auto& specification = models::hanwha::kHcr12a;
        grasplink::viewer::robotics::RobotTransformAdapter adapter(robotRoot, specification);
        grasplink::robotics::kinematics::RobotKinematics kinematics(specification);

        ApplyAndCompare(world, robotRoot, kinematics, adapter,
            {0, 0, 0, 0, 0, 0}, {}, {1, 0, 0, 0}, "zero pose");

        const double quarterTurn = std::acos(-1.0) * 0.5;
        for (std::size_t joint = 0; joint < specification.jointCount; ++joint)
        {
            std::array<double, 6> angles{};
            angles[joint] = quarterTurn;
            ApplyAndCompare(world, robotRoot, kinematics, adapter,
                angles, {}, {1, 0, 0, 0}, "single joint " + std::to_string(joint + 1));
        }

        ApplyAndCompare(world, robotRoot, kinematics, adapter,
            {0.45, -0.4, 0.3, 0.5, -0.2, 0.7}, {}, {1, 0, 0, 0}, "mixed pose");
        ApplyAndCompare(world, robotRoot, kinematics, adapter,
            {0.45, -0.4, 0.3, 0.5, -0.2, 0.7},
            {1.2F, -0.4F, 0.7F}, glm::quat{glm::vec3{0.3F, 0.7F, -0.4F}}, "root translation and rotation");

        // 검증: 정상 bind pose에서 시작해 각 실패 원인을 따로 확인.
        ApplyAndCompare(world, robotRoot, kinematics, adapter,
            {0, 0, 0, 0, 0, 0}, {}, {1, 0, 0, 0}, "restore bind pose");
        grasplink::viewer::robotics::RobotTransformAdapter validBind(robotRoot, specification);

        Entity j1 = robotRoot.FindChildByNameRecursive("J1");
        // -identity도 같은 bind 방향이다. quaternion 부호로 정상 자산을 거부하지 않는다.
        j1.SetLocalRotation(glm::quat{-1.0F, 0.0F, 0.0F, 0.0F});
        grasplink::viewer::robotics::RobotTransformAdapter negativeIdentityBind(robotRoot, specification);
        const glm::vec3 originalJ1Position = j1.GetLocalPosition();
        j1.SetLocalPosition(originalJ1Position + glm::vec3{0.01F, 0.0F, 0.0F});
        ExpectThrows<std::invalid_argument>(
            [&] { grasplink::viewer::robotics::RobotTransformAdapter bad(robotRoot, specification); },
            "changed GLB bind pivot rejected");
        j1.SetLocalPosition(originalJ1Position);

        Entity j2 = robotRoot.FindChildByNameRecursive("J2");
        Entity originalJ2Parent = j2.GetParent();
        j2.SetParent(robotRoot);
        ExpectThrows<std::invalid_argument>(
            [&] { grasplink::viewer::robotics::RobotTransformAdapter bad(robotRoot, specification); },
            "broken joint ancestor hierarchy rejected");
        j2.SetParent(originalJ2Parent);

        std::cout << "Robot pose GLB integration checks passed\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
