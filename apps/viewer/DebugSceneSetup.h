#pragma once

#include "robotics/core/ControlTypes.h"

#include <memory>

class Scene;
class Shader;

namespace grasplink::robotics
{
class IRobotController;
namespace models
{
struct RobotSpecification;
}
}

/** @brief physicsDemo 옵션이 켜졌을 때만 사용하는 시각/물리 데모 구성 함수. */
namespace viewer_debug
{
/**
 * @brief 활성 Scene에 낙하 확인용 Dynamic Box 50개를 만든다.
 * @param scene Entity와 ECS Collider 설정을 소유하는 활성 Scene.
 * @param shader Box가 공유할 렌더 Shader.
 * @details Mesh와 Material은 모든 Box가 공유한다. ECS에 RigidBody/Colliders 설정을 붙이며 실제 Jolt
 * Body 생성과 계산 결과의 Entity 반영은 PhysicsSystemModule이 담당한다.
 * @throws std::runtime_error shader가 비어 있는 경우.
 */
void CreatePhysicsBoxes(Scene& scene, const std::shared_ptr<Shader>& shader);

/**
 * @brief 첫 관절만 30°로 움직이는 데모 명령을 제출한다.
 * @param controller 명령을 받을 Controller.
 * @param specification 관절 수를 확인하는 로봇 사양.
 * @return Controller의 MoveJoint 명령 수락 결과.
 * @details 나머지 목표각은 0 rad, 속도/가속도 배율은 1이다. 이 함수는 명령만 제출하며 실제 움직임은
 * 이후 Fixed Update에서 진행된다.
 */
grasplink::robotics::Result StartRobotMotion(grasplink::robotics::IRobotController& controller,
    const grasplink::robotics::models::RobotSpecification& specification);
}
