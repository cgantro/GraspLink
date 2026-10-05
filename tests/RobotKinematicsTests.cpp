#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"
#include "TestSupport.h"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>

using namespace grasplink::robotics;
using grasplink::robotics::kinematics::RobotKinematics;

namespace
{
void RequireToolPosition(
    RobotKinematics& kinematics,
    RobotState& state,
    const std::array<double, 6>& jointRadians,
    const std::array<double, 3>& expectedMeters,
    const std::string& label)
{
    // jointRadians는 rad, expectedMeters는 로봇 Base 기준 m. 단일 관절 회전으로 축 부호와 링크 누적 순서를 확인한다.
    state.jointPositionRadians.assign(jointRadians.begin(), jointRadians.end());
    const auto& result = kinematics.Update(state);
    Require(result.toolFrameValid, label + ": ToolFrame should be valid");
    Require(result.jointLocalRotations.size() == models::hanwha::kHcr12a.jointCount,
        label + ": local rotation count");
    Require(result.linkPosesInBaseFrame.size() == models::hanwha::kHcr12a.jointCount,
        label + ": base-frame link pose count");
    RequireNear(result.toolFrameInBaseFrame.positionMeters.x, expectedMeters[0], 1e-9, label + ": ToolFrame x");
    RequireNear(result.toolFrameInBaseFrame.positionMeters.y, expectedMeters[1], 1e-9, label + ": ToolFrame y");
    RequireNear(result.toolFrameInBaseFrame.positionMeters.z, expectedMeters[2], 1e-9, label + ": ToolFrame z");
}

void RequireInvalidModel(const models::RobotSpecification& specification, const std::string& label)
{
    ExpectThrows<std::invalid_argument>([&] { RobotKinematics invalid(specification); }, label);
}
}

/**
 * @brief HCR-12A 순기구학의 기준 자세와 입력 검증을 확인한다.
 * @details 영 자세와 각 관절의 90° 단독 회전, 혼합 회전을 별도 축 회전 계산으로 얻은 Base 기준 위치와 비교한다.
 * 자세값은 모델 링크 피벗/ToolFrame을 따라 누적한 결과이며, 잘못된 사양·상태는 계산 전에 거부되어야 한다.
 */
int main()
{
    try
    {
        RobotKinematics kinematics(models::hanwha::kHcr12a);
        RobotState state;
        state.valid = true;
        const double quarterTurn = std::acos(-1.0) * 0.5;

        RequireToolPosition(kinematics, state, {0, 0, 0, 0, 0, 0},
            {0.0, 1.015, 0.9145}, "zero pose");
        RequireToolPosition(kinematics, state, {quarterTurn, 0, 0, 0, 0, 0},
            {0.9145, 1.015, 0.0}, "J1 positive quarter turn");
        RequireToolPosition(kinematics, state, {0, quarterTurn, 0, 0, 0, 0},
            {0.0, -0.5045, 0.805}, "J2 positive quarter turn");
        RequireToolPosition(kinematics, state, {0, 0, quarterTurn, 0, 0, 0},
            {0.0, 0.0955, 0.205}, "J3 positive quarter turn");
        RequireToolPosition(kinematics, state, {0, 0, 0, quarterTurn, 0, 0},
            {0.0, 1.015, 0.9145}, "J4 positive quarter turn");
        RequireToolPosition(kinematics, state, {0, 0, 0, 0, quarterTurn, 0},
            {0.0, 0.7915, 0.691}, "J5 positive quarter turn");
        RequireToolPosition(kinematics, state, {0, 0, 0, 0, 0, quarterTurn},
            {0.0, 1.015, 0.9145}, "J6 positive quarter turn");
        // 기준값: 별도 3×3 축 회전 행렬로 구한 Base 기준 위치 [m]. 1e-9 m 비교는 이 계산 경로의 double 결과를 점검한다.
        RequireToolPosition(kinematics, state, {0.45, -0.4, 0.3, 0.5, -0.2, 0.7},
            {0.26702766609490924, 1.0867538528236094, 0.60173049560973602}, "mixed pose");

        state.jointPositionRadians.assign(6, 0.0);
        const auto& zeroResult = kinematics.Update(state);
        // 관절 피벗도 Base 기준 [m]. 1e-12는 같은 사양 상수를 쓰는 계산에서 피벗 누적 변경을 감지한다.
        RequireNear(zeroResult.linkPosesInBaseFrame.front().positionMeters.y, 0.1985, 1e-12,
            "first joint base-frame pivot");
        RequireNear(zeroResult.linkPosesInBaseFrame.back().positionMeters.z, 0.85475, 1e-12,
            "last joint base-frame pivot");

        models::RobotSpecification withoutTool = models::hanwha::kHcr12a;
        withoutTool.hasToolFrame = false;
        RobotKinematics noToolKinematics(withoutTool);
        Require(!noToolKinematics.Update(state).toolFrameValid, "missing ToolFrame stays invalid");

        RobotState invalidState = state;
        invalidState.valid = false;
        ExpectThrows<std::invalid_argument>([&] { kinematics.Update(invalidState); }, "invalid state rejected");
        invalidState = state;
        invalidState.jointPositionRadians.pop_back();
        ExpectThrows<std::invalid_argument>([&] { kinematics.Update(invalidState); }, "wrong joint count rejected");
        invalidState = state;
        invalidState.jointPositionRadians[2] = std::numeric_limits<double>::quiet_NaN();
        ExpectThrows<std::invalid_argument>([&] { kinematics.Update(invalidState); }, "non-finite joint value rejected");

        auto invalidJoints = models::hanwha::kHcr12aJoints;
        auto invalidSpecification = models::hanwha::kHcr12a;
        invalidSpecification.joints = invalidJoints.data();
        invalidJoints[0].bindPivotMeters.x = std::numeric_limits<double>::infinity();
        RequireInvalidModel(invalidSpecification, "non-finite bind pivot rejected");

        invalidJoints = models::hanwha::kHcr12aJoints;
        invalidSpecification.joints = invalidJoints.data();
        invalidJoints[0].axis = {};
        RequireInvalidModel(invalidSpecification, "zero joint axis rejected");

        invalidJoints[0].axis.x = std::numeric_limits<double>::infinity();
        RequireInvalidModel(invalidSpecification, "non-finite joint axis rejected");

        invalidJoints = models::hanwha::kHcr12aJoints;
        invalidSpecification = models::hanwha::kHcr12a;
        invalidSpecification.joints = invalidJoints.data();
        invalidSpecification.toolFrameInLastJoint.rotation = {0.0, 0.0, 0.0, 0.0};
        RequireInvalidModel(invalidSpecification, "zero ToolFrame quaternion rejected");

        invalidSpecification = models::hanwha::kHcr12a;
        invalidSpecification.toolFrameInLastJoint.positionMeters.z = std::numeric_limits<double>::infinity();
        RequireInvalidModel(invalidSpecification, "non-finite ToolFrame position rejected");
        invalidSpecification = models::hanwha::kHcr12a;
        invalidSpecification.toolFrameInLastJoint.rotation.w = std::numeric_limits<double>::quiet_NaN();
        RequireInvalidModel(invalidSpecification, "non-finite ToolFrame quaternion rejected");

        invalidSpecification = models::hanwha::kHcr12a;
        invalidSpecification.joints = nullptr;
        RequireInvalidModel(invalidSpecification, "missing joint array rejected");
        invalidSpecification = models::hanwha::kHcr12a;
        invalidSpecification.jointCount = 0;
        RequireInvalidModel(invalidSpecification, "empty joint array rejected");

        std::cout << "Robot kinematics reference and validation checks passed\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
