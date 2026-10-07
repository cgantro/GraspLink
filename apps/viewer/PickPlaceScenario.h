#pragma once

#include <memory>

class Scene;
class Shader;
class Entity;

namespace grasplink::viewer::pick_place
{
Entity CreateGraspBox(Scene& scene, const std::shared_ptr<Shader>& shader);
Entity CreatePlacementArea(Scene& scene, const std::shared_ptr<Shader>& shader);
} // namespace grasplink::viewer::pick_place
