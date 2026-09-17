#pragma once

#include "RobotKinematics.h"

namespace PoseLink::Kinematics
{
/** Object local frame에서 TCP가 잡아야 하는 frame을 정의한다. */
class GraspPoseCalculator final
{
public:
    explicit GraspPoseCalculator(RigidTransform objectFromTcp) noexcept;

    /** \f$T^W_{TCP,target}=T^W_{Object}T^{Object}_{TCP,grasp}\f$ 를 계산한다. */
    RigidTransform CalculateTcpTarget(const RigidTransform& worldFromObject) const noexcept;
    const RigidTransform& ObjectFromTcp() const noexcept;

private:
    RigidTransform objectFromTcp_;
};

struct GraspThresholds
{
    double positionMetres = 0.015;
    double orientationRadians = 0.12;
};

/**
 * \brief grasp의 논리 상태와 object-to-end-effector relative transform을 소유한다.
 *
 * attach 조건은 위치와 방향 오차를 동시에 만족하는 것이다. attach 순간
 * \f$T^{EE}_{Object}=(T^W_{EE})^{-1}T^W_{Object}\f$ 를 저장한다. 다음 frame에서
 * `T^W_Object = T^W_EE * T^EE_Object`를 사용하므로, object의 world pose가 attach
 * 직전에 보이던 위치에서 갑자기 grasp nominal pose로 튀지 않는다. 물리 제약은 만들지 않으며
 * 이것은 visual/logical parenting state machine이다.
 */
class GraspController final
{
public:
    explicit GraspController(GraspThresholds thresholds = {});
    GraspState State() const noexcept;
    bool IsAttached() const noexcept;

    /** Only succeeds when `endEffector` reaches `desiredTcp`. */
    bool TryAttach(const RigidTransform& endEffector, const RigidTransform& object,
                   const RigidTransform& desiredTcp) noexcept;
    /** 새 operator target은 기존 파지를 의도하지 않는 것으로 보고 release한다. */
    void ReleaseForNewTarget() noexcept;
    void Release() noexcept;

    /** Attached 상태이면 EE relative offset을 적용한 새 object pose를 반환한다. */
    bool UpdateAttachedObject(const RigidTransform& endEffector, RigidTransform& inOutObject) const noexcept;

private:
    GraspThresholds thresholds_;
    GraspState state_ = GraspState::Open;
    RigidTransform endEffectorFromObject_{};
};
} // namespace PoseLink::Kinematics
