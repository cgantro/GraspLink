#pragma once

class Entity;
class Scene;
struct ModelResource;

namespace grasplink::simulation
{
/**
 * @brief Build the seven TwoF85 rigid-part collision proxies from authored GLB mesh data.
 * @details Vertices are converted to meters in each owning joint/body local frame. Each owned mesh
 * receives its own convex hull; the hulls approximate collision geometry and do not implement
 * grasping or contact control. Proxies are children of the authored ECS joints, which remain the
 * source of truth until a gripper backend updates them. Fixed Update refreshes World transforms
 * before PhysicsSystemModule synchronizes the identity-local Kinematic proxies. The supplied Scene
 * owns their lifetime.
 * @param scene Scene that contains robotRoot and owns the created proxies.
 * @param robotRoot Root of the instantiated robot hierarchy.
 * @param model GLB model resource containing the TwoF85 nodes and mesh vertices in meters.
 * @throws std::invalid_argument when the root, required nodes, hierarchy, transforms, or hulls are invalid.
 */
void ConfigureTwoF85Colliders(Scene& scene, const Entity& robotRoot, const ModelResource& model);
}
