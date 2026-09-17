#pragma once

#include "KinematicsTypes.h"

#include <array>
#include <cstddef>

namespace PoseLink::Kinematics
{
struct JointLimit
{
    double minimumRadians = 0.0;
    double maximumRadians = 0.0;
};

/**
 * \brief HCR-12A용 renderer-independent serial-chain 명세다.
 *
 * `jointAxisLocal[i]`와 `linkOffsetLocal[i]`는 joint i의 부모 link frame에서 표현된다.
 * FK는 `T = T * Rot(axis_i, q_i) * Trans(linkOffset_i)`를 순서대로 적용한다.
 * 따라서 CAD mesh의 vertex origin은 이 link frame과 일치해야 하며, CAD 원점을 이 API에
 * 억지로 맞추기보다 import 단계에서 mesh local transform을 보정하는 것이 안전하다.
 *
 * HCR-12A의 공식 6 축/1.3 m reach 및 motion range를 limit에 반영한다. link offset은
 * 실제 CAD joint-frame 확정 전의 개발용 parameter이며, CAD metrology가 끝나면 이 factory의
 * 수치만 교체한다. 관절 limit은 software safety limit이므로 IK는 절대 이를 넘는 결과를
 * 반환하지 않는다.
 */
struct RobotSpecification
{
    std::array<Position3D, 6> jointAxisLocal{};
    std::array<Position3D, 6> linkOffsetLocal{};
    std::array<JointLimit, 6> jointLimits{};
    RigidTransform toolFromJoint6{};

    /** HCR-12A 6R nominal specification. 모든 길이는 metre, limit은 radian이다. */
    static RobotSpecification MakeHcr12aNominal();

    /** NaN/축 길이 0/역전된 limit처럼 FK를 정의할 수 없는 specification을 거부한다. */
    bool IsValid() const noexcept;

    /** 관절값을 안전 limit 안으로 투영한다. 이 함수는 angle wrap을 하지 않는다. */
    JointState ClampToLimits(const JointState& joints) const noexcept;
};

/** FK가 계산한 joint origin, link endpoint, TCP frame의 snapshot이다. */
struct ForwardKinematicsResult
{
    std::array<RigidTransform, 6> jointFrames{};
    std::array<RigidTransform, 6> linkFrames{};
    RigidTransform tcp{};
};

/**
 * \brief 6R serial arm의 forward kinematics와 geometric Jacobian을 제공한다.
 *
 * 객체는 immutable RobotSpecification을 복사해 보관하므로 호출간 hidden state가 없다.
 * Evaluate() 결과는 caller가 소유하며 scene/ECS transform을 직접 수정하지 않는다.
 */
class ForwardKinematics final
{
public:
    explicit ForwardKinematics(RobotSpecification specification);
    const RobotSpecification& Specification() const noexcept;
    ForwardKinematicsResult Evaluate(const JointState& joints) const;

    /**
     * \brief TCP에서의 6x6 geometric Jacobian을 row-major로 반환한다.
     *
     * top 3 rows are m/rad, bottom 3 rows are rad/rad. Column i is
     * `[z_i x (p_tcp - p_i); z_i]`, where z_i and p_i are world joint axis/origin.
     */
    std::array<double, 36> GeometricJacobian(const JointState& joints) const;

private:
    RobotSpecification specification_;
};

enum class IkStatus : std::uint8_t
{
    Converged,
    /** Target origin is outside the chain's conservative maximum link-length sphere. */
    Unreachable,
    IterationLimit,
    InvalidInput
};

struct IkOptions
{
    std::size_t maximumIterations = 160;
    double damping = 0.08;
    double maximumStepRadians = 0.12;
    double positionToleranceMetres = 0.003;
    double orientationToleranceRadians = 0.035;
    /** orientation error에 곱하는 길이 scale[m/rad]; unit-inconsistent 6-vector를 균형화한다. */
    double orientationWeightMetresPerRadian = 0.20;
};

struct IkResult
{
    IkStatus status = IkStatus::InvalidInput;
    JointState joints{}; ///< 실패에도 마지막 joint-limit 내 안전 자세를 반환한다.
    std::size_t iterations = 0;
    PoseError finalError{};
};

/** 다형적 motion planner가 solver 교체 없이 IK를 사용할 수 있게 하는 contract다. */
class IIkSolver
{
public:
    virtual ~IIkSolver() = default;
    virtual IkResult Solve(const RigidTransform& targetTcp, const JointState& seed) const = 0;
};

/**
 * \brief Damped Least Squares (Levenberg-Marquardt) 6DoF inverse kinematics solver.
 *
 * 각 반복에서 \f$\Delta q=J^T(JJ^T+\lambda^2I)^{-1}e\f$ 를 푼다. damping은 singular
 * configuration에서 pseudo-inverse의 큰 joint velocity를 제한한다. orientation error에는
 * axis-angle의 최단 회전을 사용하며, position[m]과 rotation[rad]의 단위 차이는
 * orientationWeightMetresPerRadian으로 조정한다. solver는 scene state를 소유하지 않으며
 * convergence failure 때에도 입력 seed에서 출발한 마지막 limit-safe state만 반환한다.
 */
class DampedLeastSquaresIkSolver final : public IIkSolver
{
public:
    DampedLeastSquaresIkSolver(ForwardKinematics kinematics, IkOptions options = {});
    IkResult Solve(const RigidTransform& targetTcp, const JointState& seed) const override;

private:
    ForwardKinematics kinematics_;
    IkOptions options_;
};

RigidTransform Compose(const RigidTransform& parentFromMid, const RigidTransform& midFromChild) noexcept;
RigidTransform Inverse(const RigidTransform& parentFromChild) noexcept;
PoseError CalculatePoseError(const RigidTransform& current, const RigidTransform& target) noexcept;
bool IsFinite(const RigidTransform& transform) noexcept;
} // namespace PoseLink::Kinematics
