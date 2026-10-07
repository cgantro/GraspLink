#pragma once

#include "robotics/core/ControlTypes.h"

#include <array>
#include <memory>

class Scene;
class Shader;
class Entity;

namespace grasplink::robotics
{
class IRobotController;
namespace models
{
struct RobotSpecification;
}
}

/** @brief Viewer에서 물리 동작과 파지를 확인할 물체를 만드는 장면 구성 함수다. */
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
 * @brief 열린 85 mm 손끝 사이에 들어갈 수 있는 한 변 40 mm의 상자를 바닥 위에 놓는다.
 * @details 화면에 그리는 정육면체와 물리 충돌 상자의 크기는 같으며, 위치와 반쪽 길이는 [m]다. 물체는 Dynamic이므로 들어 올린 뒤 놓으면 중력으로 떨어진다.
 */
Entity CreateGraspBox(Scene& scene, const std::shared_ptr<Shader>& shader);

/**
 * @brief 로봇의 검증된 도달 범위 안에서 바닥 위 상자 위치를 새로 무작위 선택한다.
 * @return X와 Z가 각각 −1.05–1.05 m 범위에서 선택된 Scene 위치다. Y는 상자 중심 높이 0.026 m다.
 * @details 로봇 중심에서 X/Z 평면 거리가 0.50 m보다 작거나 1.05 m보다 큰 위치는 제외해 받침대 주변과 기본 도달 거리 밖에서 뽑지 않는다. Y는 바닥 윗면 0.005 m와 상자 반높이 0.020 m에 1 mm 간격을 더한 높이다.
 */
std::array<float, 3> RandomGraspBoxPosition();

/**
 * @brief 작업대의 접근 가능한 범위 안에서 새 무작위 목표판 중심을 선택한다.
 * @return X와 Z가 각각 −1.05–1.05 m 범위에서 선택된 Scene 좌표계 목표판 중심 [m]이다. Y는 시각화 높이 0.006 m다.
 * @details 로봇 중심에서 X/Z 평면 거리가 0.50 m보다 작거나 1.05 m보다 큰 위치는 제외한다. 목표판이 상자보다 커서 바깥 모서리가 도달 반경을 넘지 않도록 실제 중심 반경 상한은 1.00 m로 둔다.
 */
std::array<float, 3> RandomPlacementAreaPosition();

/** @brief 바닥에 놓인 상자와 목표판에 사용할 임의의 Y축 회전각 [rad]을 반환한다. */
float RandomPlanarRotationRadians();

/** @brief 바닥 위에 상자 한 변의 약 1.73배인 정사각 목표 영역을 그린다. 상자 면적의 세 배이며 충돌 형상은 추가하지 않는다. */
Entity CreatePlacementArea(Scene& scene, const std::shared_ptr<Shader>& shader);

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
