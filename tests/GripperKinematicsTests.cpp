#include "robotics/kinematics/GripperKinematics.h"
#include "robotics/models/robotiq/TwoF85.h"
#include "TestSupport.h"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace
{
using namespace grasplink::robotics;
using kinematics::GripperKinematicState;
using kinematics::GripperKinematics;

GripperState ClosureState(double fraction)
{
    GripperState state;
    state.valid = true;
    state.closureFractionValid = true;
    state.closureFraction = fraction;
    state.mode = GripperMode::Inactive;
    return state;
}

void RequireQuaternion(const models::QuaternionWxyz& actual,
    const models::QuaternionWxyz& expected, double tolerance, const std::string& label)
{
    RequireNear(actual.w, expected.w, tolerance, label + ": w");
    RequireNear(actual.x, expected.x, tolerance, label + ": x");
    RequireNear(actual.y, expected.y, tolerance, label + ": y");
    RequireNear(actual.z, expected.z, tolerance, label + ": z");
}

void RequireSamePose(const GripperKinematicState& actual,
    const GripperKinematicState& expected, const std::string& label)
{
    RequireNear(actual.masterAngleRadians, expected.masterAngleRadians, 0.0, label + ": master angle");
    Require(actual.jointAnglesRadians.size() == expected.jointAnglesRadians.size(), label + ": angle count");
    Require(actual.jointLocalRotations.size() == expected.jointLocalRotations.size(), label + ": rotation count");
    for (std::size_t i = 0; i < expected.jointAnglesRadians.size(); ++i)
    {
        RequireNear(actual.jointAnglesRadians[i], expected.jointAnglesRadians[i], 0.0, label + ": joint angle");
        RequireQuaternion(actual.jointLocalRotations[i], expected.jointLocalRotations[i], 0.0, label + ": rotation");
    }
}

void CheckReferencePoses()
{
    GripperKinematics calculator(models::robotiq::kTwoF85);
    // 기준: 여섯 관절의 mimic 부호와 -Z Local 축. 각도는 rad, quaternion은 [w,x,y,z].
    constexpr std::array<double, 6> signs{1.0, -1.0, 1.0, -1.0, -1.0, 1.0};
    for (double fraction : {0.0, 0.5, 1.0})
    {
        const auto& pose = calculator.Update(ClosureState(fraction));
        const double master = 0.7929 * fraction;
        RequireNear(pose.masterAngleRadians, master, 1.0e-12, "nominal master angle");
        Require(pose.jointAnglesRadians.size() == signs.size(), "six mimic angles");
        Require(pose.jointLocalRotations.size() == signs.size(), "six local rotations");
        for (std::size_t i = 0; i < signs.size(); ++i)
        {
            const double angle = signs[i] * master;
            const std::string label = std::string(models::robotiq::kTwoF85Joints[i].name) +
                " closure " + std::to_string(fraction);
            RequireNear(pose.jointAnglesRadians[i], angle, 1.0e-12, label + ": mimic angle");
            const auto& rotation = pose.jointLocalRotations[i];
            RequireQuaternion(rotation, {std::cos(angle * 0.5), 0.0, 0.0, -std::sin(angle * 0.5)},
                1.0e-12, label);
            RequireNear(rotation.w * rotation.w + rotation.x * rotation.x +
                rotation.y * rotation.y + rotation.z * rotation.z, 1.0, 1.0e-12, label + ": unit rotation");
        }
    }
}

void CheckContinuousInputAndRejectedState()
{
    GripperKinematics calculator(models::robotiq::kTwoF85);
    GripperState state = ClosureState(0.3725);
    state.actualPosition = 255;
    state.requestedPositionEcho = 0;
    const GripperKinematicState continuous = calculator.Update(state);
    state.actualPosition = 0;
    state.requestedPositionEcho = 255;
    state.currentRaw = 255;
    state.activated = true;
    state.mode = GripperMode::Moving;
    RequireSamePose(calculator.Update(state), continuous, "raw feedback and activation do not change pose");

    // Update가 반환한 참조를 유지해 거부 직후의 저장값을 검사한다. 새 정상 Update로 덮어쓰지 않는다.
    const auto& retained = calculator.Update(state);
    const GripperKinematicState before = retained;
    const auto reject = [&](const GripperState& invalid, const std::string& label)
    {
        ExpectThrows<std::invalid_argument>([&] { calculator.Update(invalid); }, label);
        RequireSamePose(retained, before, label + ": previous pose retained");
    };
    GripperState invalid = state;
    invalid.valid = false;
    reject(invalid, "invalid snapshot");
    invalid = state;
    invalid.closureFractionValid = false;
    reject(invalid, "invalid fraction flag");
    for (double fraction : {-0.01, 1.01, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()})
    {
        invalid = state;
        invalid.closureFraction = fraction;
        reject(invalid, "out-of-range or non-finite fraction");
    }
}

void CheckNonunitAxis()
{
    auto joints = models::robotiq::kTwoF85Joints;
    joints[0].axis = {2.0, -3.0, 6.0};
    auto specification = models::robotiq::kTwoF85;
    specification.joints = joints.data();
    GripperKinematics calculator(specification);
    const auto& pose = calculator.Update(ClosureState(0.5));
    const double halfAngle = 0.7929 * 0.25;
    const double sine = std::sin(halfAngle);
    // (2,-3,6)의 길이는 7. 축의 크기는 회전각에 영향을 주지 않는다.
    RequireQuaternion(pose.jointLocalRotations[0],
        {std::cos(halfAngle), sine * 2.0 / 7.0, sine * -3.0 / 7.0, sine * 6.0 / 7.0},
        1.0e-12, "arbitrary axis is normalized");
    RequireNear(pose.jointAnglesRadians[0], 0.7929 * 0.5, 1.0e-12, "axis length preserves joint angle");
}

void CheckInvalidSpecifications()
{
    const auto reject = [](const models::GripperSpecification& specification, const std::string& label)
    {
        ExpectThrows<std::invalid_argument>([&] { GripperKinematics invalid(specification); }, label);
    };
    auto specification = models::robotiq::kTwoF85;
    specification.joints = nullptr;
    reject(specification, "missing joint array");
    specification = models::robotiq::kTwoF85;
    specification.jointCount = 0;
    reject(specification, "empty joint array");
    for (double angle : {0.0, -0.1, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity()})
    {
        specification = models::robotiq::kTwoF85;
        specification.nominalMasterClosedRadians = angle;
        reject(specification, "invalid nominal closed angle");
    }

    auto joints = models::robotiq::kTwoF85Joints;
    const auto rejectJoint = [&](const models::GripperJointSpecification& joint, const std::string& label)
    {
        joints = models::robotiq::kTwoF85Joints;
        joints[0] = joint;
        specification = models::robotiq::kTwoF85;
        specification.joints = joints.data();
        reject(specification, label);
    };
    auto joint = models::robotiq::kTwoF85Joints[0];
    joint.name = {};
    rejectJoint(joint, "empty joint name");
    joint = models::robotiq::kTwoF85Joints[0];
    joint.name = models::robotiq::kTwoF85Joints[1].name;
    rejectJoint(joint, "duplicate joint name");
    for (const models::Axis3 axis : {models::Axis3{},
        models::Axis3{std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0},
        models::Axis3{0.0, std::numeric_limits<double>::infinity(), 1.0},
        models::Axis3{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max()}})
    {
        joint = models::robotiq::kTwoF85Joints[0];
        joint.axis = axis;
        rejectJoint(joint, "invalid joint axis");
    }
    for (double multiplier : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
    {
        joint = models::robotiq::kTwoF85Joints[0];
        joint.masterMultiplier = multiplier;
        rejectJoint(joint, "non-finite mimic multiplier");
    }
    for (const std::array<double, 2> limits : {std::array<double, 2>{0.5, -0.5},
        std::array<double, 2>{std::numeric_limits<double>::quiet_NaN(), 0.8},
        std::array<double, 2>{0.0, std::numeric_limits<double>::infinity()},
        std::array<double, 2>{0.1, 0.8}, std::array<double, 2>{-0.8, -0.1},
        std::array<double, 2>{0.0, 0.7}})
    {
        joint = models::robotiq::kTwoF85Joints[0];
        joint.minPositionRadians = limits[0];
        joint.maxPositionRadians = limits[1];
        rejectJoint(joint, "invalid limits or excluded open-to-closed range");
    }
    joint = models::robotiq::kTwoF85Joints[0];
    joint.masterMultiplier = -1.0;
    rejectJoint(joint, "negative closed angle outside limits");

    joints = models::robotiq::kTwoF85Joints;
    specification = models::robotiq::kTwoF85;
    specification.joints = joints.data();
    specification.nominalMasterClosedRadians = std::numeric_limits<double>::max();
    joints[0].masterMultiplier = 2.0;
    joints[0].maxPositionRadians = std::numeric_limits<double>::max();
    reject(specification, "nominal mimic angle overflow");
}
}

/**
 * @brief 2F-85 연속 개폐 위치의 mimic 각도와 Local quaternion 입력 계약을 검증한다.
 * @details 열린·중간·닫힌 기준 자세, 축 정규화와 잘못된 입력 거부를 확인한다.
 * GLB bind 합성·부모 계층 전파·물리 추종은 별도의 PoseIntegration 테스트가 담당한다.
 */
int main()
{
    try
    {
        CheckReferencePoses();
        CheckContinuousInputAndRejectedState();
        CheckNonunitAxis();
        CheckInvalidSpecifications();
        std::cout << "Gripper kinematics mimic, quaternion and validation checks passed\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
