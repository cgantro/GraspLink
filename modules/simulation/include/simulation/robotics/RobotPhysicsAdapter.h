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
 * @brief GLB의 팔 링크 메시에서 Kinematic 충돌 프록시를 만들고 FK 자세를 전달한다.
 * @details 각 링크 메시의 위치는 그 링크를 움직이는 관절 원점 기준 [m]으로 저장한다. 다음 가동 관절과
 * Gripper 아래 형상은 소유 링크에서 제외한다. 삼각형 연결 부품별로 16 cm 셀의 볼록 외피를 만들며,
 * 셀은 삼각형 중심으로 정하고 삼각형 자체는 자르지 않는다. 현재 HCR-12A 모델은 팔 링크 6개에서
 * 외피 121개를 만들며 Base 충돌 형상은 만들지 않는다. 볼록 근사는 오목한 빈 공간을 채울 수 있다.
 * Scene이 프록시 Entity를 소유하고 이 어댑터는 Entity 핸들만 보관한다. Scene 제거 뒤 Apply는 실패한다.
 */
class RobotPhysicsAdapter final
{
public:
    /**
     * @brief 모델 메시를 검증하고 링크별 충돌 프록시를 Scene에 추가한다.
     * @param scene 새 프록시 Entity를 소유하는 Scene.
     * @param robotRoot GLB 계층의 모델 기준 Entity. FK 자세의 상위 변환으로 사용한다.
     * @param specification joints/links 이름과 링크를 움직이는 관절 인덱스를 제공하는 모델 참조.
     * @param model GLB 노드와 메시 정점을 담은 로드된 자원. 데이터는 생성 중만 읽는다.
     * @throws std::invalid_argument 모델 배열이나 index가 비었거나 중복·누락되었거나 충돌 형상을 만들 수 없을 때.
     * @throws std::runtime_error GLB parent, primitive index 또는 vertex index가 잘못되었을 때.
     */
    RobotPhysicsAdapter(
        Scene& scene,
        const Entity& robotRoot,
        const grasplink::robotics::models::RobotSpecification& specification,
        const ModelResource& model);

    /**
     * @brief 계산된 FK 링크 자세를 각 프록시 Entity의 Local 변환에 반영한다.
     * @param state Robot base 기준 관절 pose 배열. 위치는 [m], 회전은 [w,x,y,z] 모델 quaternion이다.
     * robotRoot의 Scene 변환은 계층 World 변환에서 적용된다.
     * @throws std::invalid_argument FK link pose 수가 연결된 관절 index에 미치지 못할 때.
     * @throws std::runtime_error Scene 제거로 보관한 Entity handle이 유효하지 않을 때.
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
