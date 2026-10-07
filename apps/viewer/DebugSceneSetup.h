#pragma once

#include <array>
#include <memory>

class Scene;
class Shader;
class Entity;

namespace viewer_debug
{
Entity CreateGraspBox(Scene& scene, const std::shared_ptr<Shader>& shader);
std::array<float, 3> RandomGraspBoxPosition();
std::array<float, 3> RandomPlacementAreaPosition();
float RandomPlanarRotationRadians();
Entity CreatePlacementArea(Scene& scene, const std::shared_ptr<Shader>& shader);
}
