#pragma once

#ifdef GRASPLINK_TEST_SCENE_SUPPORT
#include "assets/GltfLoader.h"
#include "PhysicsWorld.h"
#include "simulation/systems/PhysicsSystemModule.h"
#include <filesystem>
#include <memory>

inline ModelResource LoadTestGlb(const std::filesystem::path& filename)
{
    return GltfLoader::LoadGLB(std::filesystem::path("assets") / filename);
}

inline std::unique_ptr<grasplink::simulation::PhysicsSystemModule> CreateTestPhysicsSystem(
    flecs::world& world, grasplink::physics::PhysicsWorld& physics)
{
    return std::make_unique<grasplink::simulation::PhysicsSystemModule>(world, physics);
}
#endif
