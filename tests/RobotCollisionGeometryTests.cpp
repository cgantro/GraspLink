#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"
#include "assets/GltfLoader.h"
#include "scene/SceneManager.h"
#include "simulation/components/PhysicsComponents.h"
#include "simulation/robotics/RobotPhysicsAdapter.h"
#include "systems/TransformSystemModule.h"
#include "TestSupport.h"

#include <iostream>
#include <filesystem>

namespace
{
// Physics proxy 추출의 CPU fixture. 정점은 Link Local 기준이며 한 변 0.1 m인 닫힌 입방체다.
MeshData Cube()
{
    MeshData mesh;
    for (unsigned i = 0; i < 8; ++i)
    {
        Vertex vertex{};
        vertex.position = {(i & 1) ? 0.05F : -0.05F, (i & 2) ? 0.05F : -0.05F, (i & 4) ? 0.05F : -0.05F};
        mesh.vertices.push_back(vertex);
    }
    mesh.indices = {0,1,3,0,3,2,4,6,7,4,7,5,0,4,5,0,5,1,2,3,7,2,7,6,0,2,6,0,6,4,1,5,7,1,7,3};
    SubMeshInfo part;
    part.indexCount = static_cast<std::uint32_t>(mesh.indices.size());
    mesh.subMeshes.push_back(part);
    return mesh;
}
}

/**
 * @brief 링크별 충돌 형상의 소유 범위와 FK 자세 추종을 확인한다.
 * @details 작은 합성 계층으로 하위 관절/Gripper geometry 제외, proxy의 joint pivot 추종, 잘못된 계층과
 * 제거된 Scene handle을 검사한 뒤 실제 HCR12A GLB에서도 arm 6개 링크의 hull 수를 확인한다.
 * 이 검사는 GLB CPU 데이터에서 collider 설정을 만드는 경계를 다루며 GPU 업로드나 Jolt 접촉 계산은 수행하지 않는다.
 */
int main()
{
    try
    {
        using namespace grasplink::robotics;
        const models::JointSpecification joints[] = {
            {"J1", {0,0,0}, {0,0,1}, -3.1416, 3.1416, 1},
            {"J2", {0,0.5,0}, {0,0,1}, -3.1416, 3.1416, 1}};
        const models::LinkSpecification links[] = {{"Link1",0},{"Link2",1}};
        const models::RobotSpecification spec{"Test","TwoLink",joints,2,links,2};
        ModelResource model;
        model.rootNodeIndex = 0;
        model.meshes = {Cube(), Cube(), Cube()};
        model.nodes.resize(6);
        const char* names[] = {"RobotRoot","J1","Link1","J2","Link2","Gripper"};
        for (int i = 0; i < 6; ++i)
        {
            model.nodes[i].name = names[i];
            model.nodes[i].parentIndex = i - 1;
        }
        model.nodes[2].meshIndex = 0;
        model.nodes[3].translation.y = 0.5F;
        model.nodes[4].meshIndex = 1;
        model.nodes[5].meshIndex = 2;
        model.nodes[5].translation.y = 0.5F;
        flecs::world world;
        SceneManager scenes(world);
        scenes.LoadScene<Scene>();
        scenes.OnUpdate(0.0F);
        auto& scene = *scenes.GetActiveScene();
        Entity root = scene.CreateEntity("RobotRoot");
        grasplink::simulation::RobotPhysicsAdapter adapter(scene, root, spec, model);
        for (const char* name : {"Link1_CollisionProxy", "Link2_CollisionProxy"})
        {
            Entity proxy = root.GetChild(name);
            Require(proxy.IsValid(), "collision proxy created");
            const auto& colliders = proxy.Get<Colliders>();
            Require(!colliders.shapes.empty(), std::string(name) + ": own geometry produces colliders");
            for (const auto& shape : colliders.shapes)
                for (const auto& point : shape.pointsMeters)
                    Require(std::abs(point.x) < 0.051F && std::abs(point.y) < 0.051F && std::abs(point.z) < 0.051F,
                        "downstream joint and Gripper geometry excluded");
        }
        kinematics::RobotKinematics fk(spec);
        RobotState state;
        state.valid = true;
        state.jointPositionRadians = {1.5707963267948966, 0};
        adapter.Apply(fk.Update(state));
        TransformSystemModule::UpdateWorldTransforms(world);
        const auto link2 = root.GetChild("Link2_CollisionProxy");
        RequireNear(link2.GetLocalPosition().x, -0.5, 1e-5, "upstream joint moves proxy pivot");
        RequireNear(link2.GetLocalPosition().y, 0.0, 1e-5, "proxy follows FK rather than bind geometry");
        auto cycle = model;
        cycle.nodes[0].parentIndex = 5;
        ExpectThrows<std::invalid_argument>([&] { grasplink::simulation::RobotPhysicsAdapter bad(scene, root, spec, cycle); }, "GLB cycle rejected before recursion");
        ExpectThrows<std::invalid_argument>([&] { adapter.Apply({}); }, "missing FK poses rejected");
        scenes.LoadScene<Scene>();
        scenes.OnUpdate(0.0F);
        ExpectThrows<std::runtime_error>([&] { adapter.Apply(fk.Update(state)); }, "removed Scene invalidates physics proxy bindings");
        // 실제 GLB에는 121개 hull이 만들어진다. 상한 150은 링크별 근사를 유지하면서 runtime 형상 수가 불어나지 않게 한다.
        const auto actualModel = GltfLoader::LoadGLB(std::filesystem::path("assets") / "HCR12A_2F-85.glb");
        Entity actualRoot = scenes.GetActiveScene()->CreateEntity("ActualRobot");
        grasplink::simulation::RobotPhysicsAdapter actualAdapter(
            *scenes.GetActiveScene(), actualRoot, models::hanwha::kHcr12a, actualModel);
        std::size_t actualHullCount = 0;
        for (const auto& proxy : actualRoot.GetChildren())
        {
            const auto& shapes = proxy.GetHandle().get<Colliders>().shapes;
            Require(!shapes.empty(), "actual robot link retains collision coverage");
            actualHullCount += shapes.size();
            std::cout << proxy.GetHandle().name() << ": " << shapes.size() << " hulls\n";
        }
        std::cout << "Actual robot hulls: " << actualHullCount << '\n';
        Require(actualRoot.GetChildren().size() == 6, "actual arm keeps six independent rigid links");
        Require(actualHullCount <= 150, "actual arm collider complexity stays within runtime budget");
        std::cout << "Rigid-link geometry ownership and proxy pose checks passed\n";
        return 0;
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
