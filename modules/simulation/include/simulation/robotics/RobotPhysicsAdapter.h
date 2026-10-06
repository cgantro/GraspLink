#pragma once

#include "Entity.h"
#include "assets/GraphicsTypes.h"
#include "robotics/models/RobotSpecification.h"

#include <vector>

class Scene;

namespace grasplink::robotics::kinematics
{
struct RobotKinematicState;
}

namespace grasplink::simulation
{

/**
 * @brief 로봇 팔 Mesh에서 충돌 모양을 만들고 FK 자세를 Kinematic Body에 전달한다.
 * @details FK는 관절각에서 Link 위치와 방향을 계산한다.
 * Link는 회전 관절 사이를 잇는 로봇 팔 부분이며 각 Link 메시에서 충돌 모양을 만든다.
 * 다음 관절 아래 메시와 Gripper 메시를 제외해 각 Link가 자기 부품만 가지게 한다.
 * 연결된 삼각형 덩어리를 나눈 뒤 삼각형 중심이 속한 16 cm 셀에 정점을 모아 각 셀의 ConvexHull을 만든다.
 * ConvexHull은 점을 둘러싼 볼록한 모양이므로 실제 메시의 오목한 부분을 채울 수 있다.
 * 삼각형은 셀 경계에서 자르지 않는다.
 * 현재 HCR-12A에서는 여섯 팔 Link에 121개 껍질을 만들며 Base 충돌 모양은 만들지 않는다.
 * Scene이 충돌 proxy Entity를 소유하고 이 어댑터는 Entity 핸들만 빌려 보관한다.
 * Scene이 먼저 제거되면 Apply가 저장한 Entity를 찾지 못해 실패한다.
 */
class RobotPhysicsAdapter final
{
public:
    /**
     * @brief 로봇 모델의 Link와 메시 연결을 확인한 뒤 Link별 충돌용 Entity를 Scene에 만든다.
     * @param scene 새 프록시 Entity를 소유하는 Scene.
     * @param robotRoot GLB 계층의 모델 기준 Entity. FK 자세의 상위 변환으로 사용한다.
     * @param specification 관절·Link 이름과 각 Link를 움직이는 관절 번호를 제공하는 모델 참조.
     * @param model GLB 계층과 정점 데이터가 든 자원. 이 함수가 검증과 Entity 생성 중에만 읽는다.
     * @throws std::invalid_argument 모델 배열이나 번호가 비었거나 중복·누락되었거나 충돌 모양을 만들 수 없을 때.
     * @throws std::runtime_error GLB의 부모 관계, 삼각형 연결 번호 또는 정점 번호가 잘못되었을 때.
     */
    RobotPhysicsAdapter(
        Scene& scene,
        const Entity& robotRoot,
        const grasplink::robotics::models::RobotSpecification& specification,
        const ModelResource& model);

        /**
     * @brief FK 결과를 대응하는 충돌 proxy의 부모 기준 위치와 회전으로 저장한다.
     * @param state Robot base 기준 Link 위치 [m]와 quaternion [w,x,y,z]을 담은 FK 결과다.
     * robotRoot에 설정한 Scene 위치와 회전은 부모 계층을 통해 더해진다.
     * @throws std::invalid_argument 결과에 연결된 관절 자세가 없을 때 발생한다.
     * @throws std::runtime_error Scene이 없어 저장한 Entity 핸들을 찾지 못할 때 발생한다.
     */
    void Apply(const grasplink::robotics::kinematics::RobotKinematicState& state);

private:
    struct LinkBinding
    {
        /// Scene이 소유하며 FK pose를 받는 Kinematic proxy Entity.
        Entity entity;
        /// RobotKinematicState에서 이 proxy가 사용할 관절 pose의 index.
        std::size_t jointIndex = 0;
    };

    std::vector<LinkBinding> links_;
};

}
