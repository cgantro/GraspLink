#include "Grasping.h"
#include "PoseReceiver.h"
#include "Protocol.h"
#include "RobotKinematics.h"
#include "SerialTransport.h"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

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
    using namespace PoseLink;
    using namespace PoseLink::Kinematics;

    // Wire test fixes byte order independently from the sender's host ABI.
    TargetCommand command{};
    command.sequence = 0x01020304U;
    command.targetId = 7U;
    command.position = {0.25F, -0.50F, 1.0F};
    const auto bytes = Protocol::Encode(command);
    Expect(bytes.size() == Protocol::kTargetCommandSize, "TargetCommand is exactly 28 bytes");
    Expect(bytes[0] == 'G' && bytes[1] == 'L' && bytes[2] == 'N' && bytes[3] == 'K', "wire magic is GLNK");
    Expect(bytes[8] == 0x01U && bytes[9] == 0x02U && bytes[10] == 0x03U && bytes[11] == 0x04U,
           "sequence is big-endian");
    const auto decoded = Protocol::Decode(bytes);
    Expect(decoded.has_value() && decoded->target.has_value(), "encoded target decodes");
    if (decoded && decoded->target)
    {
        Expect(decoded->target->sequence == command.sequence, "sequence round-trip");
        Expect(Near(decoded->target->position.y, command.position.y), "coordinate round-trip");
    }
    auto malformed = bytes;
    malformed[16] = 0x7fU; malformed[17] = 0xc0U; malformed[18] = 0x00U; malformed[19] = 0x00U;
    Expect(!Protocol::Decode(malformed).has_value(), "NaN coordinate is rejected");

    // UART may split one GLNK frame at any byte or prefix it with the ESP32
    // boot log. The framer must recover a single valid command in both cases.
    SerialPacketFramer serialFramer;
    const std::vector<std::uint8_t> firstChunk(bytes.begin(), bytes.begin() + 6);
    const std::vector<std::uint8_t> secondChunk(bytes.begin() + 6, bytes.end());
    Expect(serialFramer.Push(firstChunk).empty(), "partial serial header does not emit a packet");
    const auto framed = serialFramer.Push(secondChunk);
    Expect(framed.size() == 1U && framed.front().decoded.target.has_value(),
           "split serial frame is reassembled");
    serialFramer.Reset();
    std::vector<std::uint8_t> noisy{'b', 'o', 'o', 't', '\r', '\n'};
    noisy.insert(noisy.end(), bytes.begin(), bytes.end());
    Expect(serialFramer.Push(noisy).size() == 1U, "serial boot noise is discarded before GLNK");

    Expect(LatestTargetReceiver::IsStrictlyNewer(0U, 0xffffffffU), "sequence wrap is newer");
    Expect(!LatestTargetReceiver::IsStrictlyNewer(9U, 9U), "duplicate sequence is stale");
    Expect(!LatestTargetReceiver::IsStrictlyNewer(8U, 9U), "older sequence is stale");

    const RobotSpecification specification = RobotSpecification::MakeHcr12aNominal();
    Expect(specification.IsValid(), "HCR nominal specification is valid");
    ForwardKinematics forwardKinematics(specification);
    const JointState zero{};
    const auto zeroFrames = forwardKinematics.Evaluate(zero);
    Expect(IsFinite(zeroFrames.tcp), "zero-pose TCP is finite");
    const auto jacobian = forwardKinematics.GeometricJacobian(zero);
    bool finiteJacobian = true;
    for (const double value : jacobian) { finiteJacobian = finiteJacobian && std::isfinite(value); }
    Expect(finiteJacobian, "zero-pose geometric Jacobian is finite");

    DampedLeastSquaresIkSolver solver(forwardKinematics);
    const IkResult identitySolution = solver.Solve(zeroFrames.tcp, zero);
    Expect(identitySolution.status == IkStatus::Converged, "FK zero pose is a reachable IK target");

    GraspController grasp(GraspThresholds{0.01, 0.01});
    // Object and TCP initially coincide, so the saved EE->object transform is
    // identity and the expected follow-up pose is especially easy to verify.
    RigidTransform object = zeroFrames.tcp;
    Expect(grasp.TryAttach(zeroFrames.tcp, object, zeroFrames.tcp), "matching EE/object target attaches");
    RigidTransform movedEndEffector = zeroFrames.tcp;
    movedEndEffector.position.x += 0.10F;
    Expect(grasp.UpdateAttachedObject(movedEndEffector, object), "attached object updates");
    Expect(Near(object.position.x, movedEndEffector.position.x), "attach preserves EE-relative origin");
    grasp.ReleaseForNewTarget();
    Expect(!grasp.IsAttached(), "new target releases previous object");

    if (failures == 0)
    {
        std::cout << "All GraspLink deterministic tests passed.\n";
    }
    return failures == 0 ? 0 : 1;
}
