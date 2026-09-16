#include "SyntheticPoseSource.h"

#include <cmath>

namespace PoseLink
{

std::optional<Pose>
SyntheticPoseSource::Sample(double timeSeconds)
{
    const float t =
        static_cast<float>(timeSeconds);

    Pose pose;


    // --------------------------------------------------
    // Position
    // --------------------------------------------------
    //
    // 원점 주변에서 계속 움직이는 궤적.
    //
    // 같은 t에 대해 항상 같은 위치가 나오므로
    // 나중에 network/interpolation 결과의
    // ground truth로 사용할 수 있다.
    //

    pose.position.x =
        0.75f * std::sin(t);

    pose.position.y =
        0.25f * std::sin(t * 0.5f);

    pose.position.z =
        0.50f * std::cos(t);


    // --------------------------------------------------
    // Rotation
    // --------------------------------------------------
    //
    // Y축을 기준으로 angle만큼 회전.
    //
    // axis = (0, 1, 0)
    //
    // Quaternion:
    //
    // q = (
    //     cos(angle / 2),
    //     0,
    //     sin(angle / 2),
    //     0
    // )
    //

    const float angle = t;
    const float halfAngle = angle * 0.5f;

    pose.orientation.w = std::cos(halfAngle);

    pose.orientation.x = std::sin(halfAngle);

    pose.orientation.y = std::sin(halfAngle);

    pose.orientation.z = std::sin(halfAngle);

    return pose;
}

} // namespace PoseLink