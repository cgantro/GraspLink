#include "viewer/robotics/RobotTransformAdapter.h"

#include <glm/gtx/quaternion.hpp>

#include <cmath>
#include <stdexcept>
#include <string>

namespace grasplink::viewer::robotics
{
namespace
{
/**
 * @brief Joint axis가 사실상 0 vector인지 판정하는 squared-length 기준.
 *
 * @details
 * 회전축은 방향을 나타내야 하므로 길이가 0이면 회전 방향을 정의할 수 없다.
 * 제곱길이를 사용하면 sqrt 없이 빠르게 검사할 수 있다.
 */
constexpr float kAxisEpsilon = 1e-8F;

/**
 * @brief controller-ready GLB Joint의 bind rotation이 identity인지 검사하는 허용 오차 [rad].
 *
 * @details
 * Identity rotation은 "처음 로드했을 때 별도 회전이 없는 상태"다.
 * Loader가 quaternion/matrix를 Euler로 바꾸는 과정에서 아주 작은 부동소수점 잔차가 생길 수 있어 0과 완전 일치만 요구하지 않는다.
 */
constexpr float kBindRotationEpsilon = 1e-4F;
}

RobotTransformAdapter::RobotTransformAdapter(
    const Entity& robotRoot,
    const ::grasplink::robotics::models::RobotSpecification& specification)
{
    if (!robotRoot)
        throw std::runtime_error("RobotTransformAdapter: invalid robot root");

    if (specification.joints == nullptr || specification.jointCount == 0)
        throw std::invalid_argument("RobotTransformAdapter: empty robot specification");

    joints_.reserve(specification.jointCount);

    for (std::size_t i = 0; i < specification.jointCount; ++i)
    {
        const auto& jointSpec = specification.joints[i];

        /*
         * Binding 단계:
         * 모델 사양의 이름("J1", "J2"...)과 GLB/Flecs Entity 이름을 key로 사용해 서로 연결한다.
         * 한 번 찾아 joints_에 저장한 뒤 Apply()에서는 반복 검색하지 않는다.
         */
        Entity joint = robotRoot.FindChildByNameRecursive(std::string(jointSpec.name));
        if (!joint)
            throw std::runtime_error(
                "RobotTransformAdapter: joint not found: " + std::string(jointSpec.name));

        /*
         * 현재 controller-ready GLB 계약:
         * moving Joint Node(J1~J6)의 bind rotation은 identity여야 한다.
         *
         * 예전 모델처럼 Joint 자체에 90도 보정 quaternion이 남아 있으면
         * "모델 사양 axis"와 "화면에서 실제 회전하는 axis"가 달라질 수 있다.
         * 그래서 잘못된 asset을 조용히 받아들이지 않고 초기화 시점에 즉시 실패시킨다.
         */
        const glm::vec3 bindEuler = joint.GetLocalRotation();
        if (glm::dot(bindEuler, bindEuler) >
            kBindRotationEpsilon * kBindRotationEpsilon)
        {
            throw std::runtime_error(
                "RobotTransformAdapter: controller-ready GLB requires identity bind rotation: " +
                std::string(jointSpec.name));
        }

        /*
         * Robotics model 계층은 GLM에 의존하지 않기 때문에 axis를 Axis3(double)로 저장한다.
         * Viewer 경계에 들어왔을 때만 glm::vec3(float)로 변환한다.
         *
         * axis는 위치가 아니라 방향이므로 meter 같은 단위가 없다.
         */
        const glm::vec3 axis{
            static_cast<float>(jointSpec.axis.x),
            static_cast<float>(jointSpec.axis.y),
            static_cast<float>(jointSpec.axis.z)};

        if (glm::dot(axis, axis) <= kAxisEpsilon)
            throw std::runtime_error(
                "RobotTransformAdapter: invalid joint axis: " + std::string(jointSpec.name));

        joints_.push_back({joint, jointSpec.axis});
    }
}

void RobotTransformAdapter::Apply(const ::grasplink::robotics::RobotState& state)
{
    // valid=false는 Controller가 "이 snapshot을 화면에 쓰지 말라"고 표시한 상태다.
    if (!state.valid)
        return;

    if (state.jointPositionRadians.size() != joints_.size())
        throw std::invalid_argument("RobotTransformAdapter: RobotState joint count mismatch");

    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
        const double position = state.jointPositionRadians[i];
        if (!std::isfinite(position))
            throw std::invalid_argument("RobotTransformAdapter: non-finite joint position");

        auto& binding = joints_[i];

        glm::vec3 axis{
            static_cast<float>(binding.axis.x),
            static_cast<float>(binding.axis.y),
            static_cast<float>(binding.axis.z)};

        /*
         * normalize는 축 벡터의 길이를 1로 만드는 작업이다.
         * 회전 계산에는 "축의 방향"만 필요하므로 길이는 제거한다.
         */
        axis = glm::normalize(axis);

        /*
         * Controller 값 -> 화면 회전 변환 순서:
         *
         * 1) position: Controller가 계산한 현재 관절각 q [rad]
         * 2) axis: Joint 자신의 local 회전축
         * 3) glm::angleAxis(q, axis): "이 축으로 q만큼 회전"을 quaternion으로 표현
         * 4) glm::eulerAngles(): 현재 ECS가 Rotation을 vec3 Euler[rad]로 저장하므로 형식 변환
         * 5) SetLocalRotation(): 부모 Link 기준 local 회전을 Joint Entity에 저장
         * 6) TransformSystem: ParentWorld * Local을 계산해 자식 Link의 최종 world 위치/방향 결정
         *
         * Pivot은 GLB Node의 translation/hierarchy에 이미 들어 있다.
         * 따라서 여기서는 bindPivotMeters를 더하지 않고 rotation만 바꾼다.
         */
        const glm::quat rotation = glm::angleAxis(static_cast<float>(position), axis);
        binding.entity.SetLocalRotation(glm::eulerAngles(rotation));
    }
}

} // namespace grasplink::viewer::robotics
