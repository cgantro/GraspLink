#pragma once

#include "Entity.h"

#include <memory>

class AssetManager;
class Scene;
class Shader;
struct ModelResource;


/*
    ModelResource를 실제 Flecs Entity hierarchy로 변환한다.

    ModelResource
        ↓
    PrefabFactory
        ↓
    Flecs Entity

    예:

    RobotRoot
      └─ Base
          ├─ BaseMesh
          │   ├─ Primitive_0
          │   └─ Primitive_1
          │
          └─ J1
              └─ Link1
*/
class PrefabFactory final
{
public:
    static Entity CreateModel(
        Scene& scene,
        const ModelResource& model,
        const AssetManager& assets,
        const std::shared_ptr<Shader>& shader);

private:
    PrefabFactory() = delete;
};