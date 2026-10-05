#include "robotics/kinematics/RobotKinematics.h"
#include "scene/SceneManager.h"
#include "simulation/components/PhysicsComponents.h"
#include "simulation/robotics/RobotPhysicsAdapter.h"
#include "systems/TransformSystemModule.h"
#include "TestSupport.h"

#include <iostream>

namespace
{
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
        std::cout << "Rigid-link geometry ownership and proxy pose checks passed\n";
        return 0;
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
