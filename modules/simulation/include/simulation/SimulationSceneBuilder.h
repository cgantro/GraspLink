#pragma once

#include "Entity.h"

namespace grasplink::simulation
{

// Floor의 ECS 물리 설정만 구성한다. Jolt Body 생성과 갱신은 PhysicsSystemModule이 담당한다.
class SimulationSceneBuilder final
{
public:
    static void ConfigureFloor(Entity& floor);

private:
    SimulationSceneBuilder() = delete;
};

}
