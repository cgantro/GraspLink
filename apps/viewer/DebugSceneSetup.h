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

/** @brief Viewer의 physicsDemo 실행에서만 사용하는 화면 및 물리 시험 장면 구성 함수다. */
namespace viewer_debug
{
/**
 * @brief 활성 Scene에 중력으로 떨어지는 동작을 보기 위한 움직이는 Box 50개를 만든다.
 * @param scene Entity와 ECS 충돌 설정을 저장하는 활성 Scene.
 * @param shader 모든 Box가 화면에 그려질 때 공유하는 Shader.
 * @details 모든 상자는 같은 GPU 모양과 표면 설정을 공유한다. 여기서는 Flecs에 움직임 종류와 상자 충돌 모양을 저장한다.
 * PhysicsSystemModule이 그 설정에서 Jolt 물체를 만들고 중력·충돌 계산 뒤 나온 위치를 장면 물체에 반영한다.
 * @throws std::runtime_error 화면에 Box를 그릴 Shader가 비어 있는 경우.
 */
void CreatePhysicsBoxes(Scene& scene, const std::shared_ptr<Shader>& shader);

/**
 * @brief 로봇 첫 관절을 30°로 이동시키는 Controller 명령을 제출한다.
 * @param controller 목표 자세를 받을 로봇 Controller.
 * @param specification 목표 배열 크기에 사용할 관절 수를 제공하는 모델 사양.
 * @return Controller가 이동 명령을 받아들였는지 나타내는 결과.
 * @details 첫 관절 외의 목표각은 모두 0 rad이고 속도·가속도 배율은 1이다. 이 함수는 목표만 제출한다.
 * Controller가 이후 매 고정 시간 간격에서 실제 관절 회전을 갱신한다.
 */
grasplink::robotics::Result StartRobotMotion(grasplink::robotics::IRobotController& controller,
    const grasplink::robotics::models::RobotSpecification& specification);
}
