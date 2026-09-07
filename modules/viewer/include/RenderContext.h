#pragma once

namespace PoseLink {

class Renderer;
class Camera;

struct RenderContext
{
    Renderer* renderer = nullptr;
    Camera* camera = nullptr;
};

} // namespace PoseLink