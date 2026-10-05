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

// 명시적인 physicsDemo 옵션에서만 쓰는 viewer 데모 구성.
namespace viewer_debug
{
// 활성 Scene에 50개 Dynamic box 생성. Mesh/Material은 box들이 공유한다.
void CreatePhysicsBoxes(Scene& scene, const std::shared_ptr<Shader>& shader);

// 관절이 1개 이상인 모델의 절대 목표각: J1=+30도, 나머지는 0.
// 수락 여부를 반환하며 실제 이동은 고정 step에서 진행된다.
grasplink::robotics::Result StartRobotMotion(grasplink::robotics::IRobotController& controller,
    const grasplink::robotics::models::RobotSpecification& specification);
}
