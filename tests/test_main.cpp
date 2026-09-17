#include "Grasping.h"
#include "RobotKinematics.h"

#include <cmath>
#include <iostream>

namespace
{
int failures = 0;

void Expect(bool condition, const char* description)
{
    if (!condition)
    {
        std::cerr << "FAILED: " << description << '\n';
        ++failures;
    }
}

bool Near(float left, float right, float tolerance = 1.0e-5F)
{
    return std::abs(left - right) <= tolerance;
}
} // namespace

int main()
{
    using namespace PoseLink::Kinematics;

    const RobotSpecification specification = RobotSpecification::MakeHcr12aNominal();
    Expect(specification.IsValid(), "HCR-12A nominal specification is valid");

    ForwardKinematics forwardKinematics(specification);
    const JointState zero{};
    const auto zeroFrames = forwardKinematics.Evaluate(zero);
    Expect(IsFinite(zeroFrames.tcp), "zero-pose TCP is finite");

    const auto jacobian = forwardKinematics.GeometricJacobian(zero);
    bool finiteJacobian = true;
    for (const double value : jacobian)
    {
        finiteJacobian = finiteJacobian && std::isfinite(value);
    }
    Expect(finiteJacobian, "zero-pose geometric Jacobian is finite");

    DampedLeastSquaresIkSolver solver(forwardKinematics);
    const IkResult identitySolution = solver.Solve(zeroFrames.tcp, zero);
    Expect(identitySolution.status == IkStatus::Converged, "FK zero pose is a reachable IK target");

    GraspController grasp(GraspThresholds{0.01, 0.01});
    RigidTransform object = zeroFrames.tcp;
    Expect(grasp.TryAttach(zeroFrames.tcp, object, zeroFrames.tcp), "matching EE/object target attaches");

    RigidTransform movedEndEffector = zeroFrames.tcp;
    movedEndEffector.position.x += 0.10F;
    Expect(grasp.UpdateAttachedObject(movedEndEffector, object), "attached object updates");
    Expect(Near(object.position.x, movedEndEffector.position.x), "attach preserves EE-relative origin");

    grasp.ReleaseForNewTarget();
    Expect(!grasp.IsAttached(), "release clears attached state");

    if (failures == 0)
    {
        std::cout << "All GraspLink simulation tests passed.\n";
    }

    return failures == 0 ? 0 : 1;
}
