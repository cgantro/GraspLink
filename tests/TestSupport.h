#pragma once

#ifdef GRASPLINK_TEST_SCENE_SUPPORT
#include "model/GltfLoader.h"
#include "physics/PhysicsWorld.h"
#include "simulation/systems/PhysicsSystemModule.h"
#include <filesystem>
#include <memory>

inline grasplink::model::ModelResource LoadTestGlb(const std::filesystem::path& filename)
{
    return grasplink::model::GltfLoader::LoadGLB(std::filesystem::path("assets") / filename);
}

inline std::unique_ptr<grasplink::simulation::PhysicsSystemModule> CreateTestPhysicsSystem(
    flecs::world& world, grasplink::physics::PhysicsWorld& physics)
{
    return std::make_unique<grasplink::simulation::PhysicsSystemModule>(world, physics);
}
#endif
