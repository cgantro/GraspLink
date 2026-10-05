#pragma once

#include "Entity.h"

#include <memory>

class Scene;
class Shader;

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
    static Entity CreateDebugBox(Scene& scene, const std::shared_ptr<Shader>& shader);

    // Robot link collision boxes
    static void ConfigureRobotPhysics(
        const Entity& robotRoot,
        const grasplink::robotics::models::RobotSpecification& specification);

private:
    EntityFactory() = delete;
};
