#pragma once

#include "Entity.h"

namespace grasplink::simulation
{

/**
 * @brief Scene의 렌더 바닥에 대응하는 Static Environment 설정을 만든다.
 * @details 이 Builder는 ECS 설정만 기록한다. 실제 Jolt Body 생성·갱신·해제는
 * PhysicsSystemModule이 담당한다. 시각 GLB와 충돌 Box는 별도 형상이며, Box는 Entity 기준
 * 치수 [m]로 고정된다. Static Environment 자신의 시각 scale은 허용되어도 Box 치수는 변하지 않는다.
 */
class SimulationSceneBuilder final
{
public:
    /**
     * @brief 바닥 Entity에 고정 환경 Body와 Box 형상을 설정한다.
     * @param floor GLB 바닥 Mesh를 가진 Scene Entity. 부모 scale은 단위여야 한다.
     * @details plane.glb는 X/Z ±3 m, 표면 Y=0이며 렌더 형상의 두께는 1 mm다.
     * 충돌 Box는 반크기 (3, 0.02, 3) m, 중심은 Entity 원점에서 Y=-0.015 m라서
     * 상면이 표면보다 5 mm 높다. 얇은 시각 Mesh 대신 40 mm Box를 써 접촉 두께를 확보한다.
     */
    static void ConfigureFloor(Entity& floor);

private:
    SimulationSceneBuilder() = delete;
};

}
