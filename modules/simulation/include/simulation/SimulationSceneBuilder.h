#pragma once

#include "Entity.h"

namespace grasplink::simulation
{

/**
 * @brief 화면 바닥 아래에 고정된 Environment 충돌 Body 설정을 만든다.
 * @details 이 함수는 Entity에 RigidBody와 Colliders 설정만 넣는다.
 * PhysicsSystemModule이 이 설정을 읽어 Jolt Body를 생성하고 갱신하며 해제한다.
 * GLB 바닥은 화면에 그리는 Mesh이고 Box는 접촉 판정에 쓰는 별도 Collider다.
 * Box 크기는 Entity scale과 무관하게 미터 단위로 고정된다.
 * 따라서 화면 Mesh 크기를 바꿔도 충돌 Box 크기는 바뀌지 않는다.
 */
class SimulationSceneBuilder final
{
public:
    /**
     * @brief 바닥 Entity에 움직이지 않는 Environment Body와 Box Collider 설정을 붙인다.
     * @param floor GLB 바닥 Mesh를 가진 Entity다. 부모 scale은 단위여야 한다.
     * @details plane.glb는 X와 Z축으로 원점에서 3 m까지 뻗고 표면은 Y=0이며 두께는 1 mm다.
     * 충돌 Box 중심은 Entity 원점에서 Y=-0.015 m이고 반쪽 크기는 (3, 0.02, 3) m다.
     * 따라서 Box 윗면은 Y=0.005 m로 렌더 바닥보다 5 mm 높다.
     * 40 mm 두께의 Box를 아래로 내리면 화면 Mesh보다 두꺼운 충돌면이 물체가 바닥을 통과하지 않게 받친다.
     */
    static void ConfigureFloor(Entity& floor);

private:
    SimulationSceneBuilder() = delete;
};

}
