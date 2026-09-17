#pragma once

#include "KinematicsTypes.h"
#include "RobotKinematics.h"

#include <array>
#include <string_view>

/**
 * @brief static CAD assembly mesh와 renderer-independent FK frame을 잇는 visual binding이다.
 *
 * STEP->GLB exporter는 각 link의 vertex를 CAD 조립 상태 좌표계에 bake한다. 따라서
 * `worldFromJoint(q)`를 mesh entity에 곧바로 주면 zero pose에서도 CAD mesh가 원점으로
 * 한 번 더 이동한다. 이 rig는 다음 conjugated delta를 사용한다.
 *
 * \f$T^{CAD}_{mesh}(q) = T^{CAD}_{SIM} T^{SIM}_{J_i}(q)
 * (T^{SIM}_{J_i}(0))^{-1} (T^{CAD}_{SIM})^{-1}\f$
 *
 * 즉 q=0일 때 항들이 정확히 상쇄되어 original GLB assembly pose가 유지되고, 이후에는
 * joint i의 FK 변화만 CAD root frame에서 적용된다. 모든 길이는 metre, quaternion은
 * right-handed active rotation이다. 이 타입은 Flecs/OpenGL을 소유하거나 수정하지 않아
 * CAD frame calibration의 testable contract를 renderer 밖에 둔다.
 *
 * `cadFromSimulationRoot`는 supplied STEP의 Z-up CAD와 nominal simulator frame 사이의
 * R00 orientation calibration이다. link mesh pivot의 정확한 CAD metrology가 확정되면
 * 이 constant와 mesh-to-joint mapping을 calibration table로 교체해야 한다. 현재 mapping은
 * zero-pose continuity를 보장하지만, J3/J4와 J5/J6가 한 STEP group으로 묶인 geometry의
 * sub-link articulation을 만들어 내지는 않는다.
 */
class CadVisualRig final
{
public:
    static constexpr std::size_t kMeshCount = 7U;

    /** Expected node order/names emitted by the deterministic HCR-12A GLB exporter. */
    static const std::array<std::string_view, kMeshCount>& ExpectedMeshNames() noexcept;

    /** Captures the immutable q=0 FK reference needed by the delta equation. */
    explicit CadVisualRig(const PoseLink::Kinematics::ForwardKinematicsResult& zeroPose) noexcept;

    /** Returns one CAD-root model transform per GLB mesh: base followed by J1...tool groups. */
    std::array<PoseLink::Kinematics::RigidTransform, kMeshCount> Evaluate(
        const PoseLink::Kinematics::ForwardKinematicsResult& currentPose) const noexcept;

private:
    std::array<PoseLink::Kinematics::RigidTransform, 6U> zeroJointFrames_{};
    PoseLink::Kinematics::RigidTransform cadFromSimulationRoot_{};
};
