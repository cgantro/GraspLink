#pragma once

#include "Entity.h"
#include "components/PhysicsComponents.h"

#include <string>

class Scene;

namespace grasplink::robotics::models
{
struct RobotSpecification;
}

/**
 * @brief Scene Entity 생성·설정 helper
 *
 * 물리 설정: Entity에 RigidBody와 BoxCollider 부착
 * Jolt Body 수명: PhysicsSystemModule 담당
 * CreateDebugBox: 낙하·바닥 충돌 확인용 Debug 객체
 */
class EntityFactory final
{
public:
    /**
     * @brief 바닥 Entity에 고정 Box Collider 연결
     * @param floor plane.glb Mesh가 붙은 Entity. Mesh와 Collider 공유
     *
     * 위치: Box 윗면이 World Y=0이 되도록 중심을 0.1m 아래로 배치
     * 모양: GLB 변환 없이 평평한 바닥용 단순 Box
     */
    static void ConfigureFloor(Entity& floor);

    /**
     * @brief Physics 낙하 확인용 Cube 생성
     * @param scene Cube를 담을 Scene
     * @return World Y=2m에서 시작하는 Render·Physics Entity
     */
    static Entity CreateDebugBox(Scene& scene);

    /**
     * @brief 로봇 관절 Entity에 충돌용 Box 연결
     * @param robotRoot J1~J6을 찾을 로봇 root
     * @param specification 관절 이름과 순서가 담긴 로봇 사양
     *
     * Collider: 인접 관절 사이를 덮는 단순 Box. GLB Mesh 변환 없음
     * 로봇 자세: Controller가 결정. Physics는 충돌만 계산
     * 미지원: 관절 힘·토크 계산
     * @throws std::runtime_error 로봇 root 또는 사양의 관절 Entity를 찾지 못한 경우
     */
    static void ConfigureRobotPhysics(
        const Entity& robotRoot,
        const grasplink::robotics::models::RobotSpecification& specification);

private:
    EntityFactory() = delete;
};
