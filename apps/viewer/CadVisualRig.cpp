#include "CadVisualRig.h"

namespace
{
using PoseLink::Kinematics::Compose;
using PoseLink::Kinematics::Inverse;
using PoseLink::Kinematics::RigidTransform;
}

const std::array<std::string_view, CadVisualRig::kMeshCount>& CadVisualRig::ExpectedMeshNames() noexcept
{
    static constexpr std::array<std::string_view, kMeshCount> kNames{
        "HCR12A_Base", "HCR12A_Link1", "HCR12A_Link2", "HCR12A_Link3",
        "HCR12A_Link4", "HCR12A_Link5", "HCR12A_Link6_Tool"};
    return kNames;
}

CadVisualRig::CadVisualRig(const PoseLink::Kinematics::ForwardKinematicsResult& zeroPose) noexcept
    : zeroJointFrames_(zeroPose.jointFrames)
{
    // Maps simulator basis (x,y,z) to GLB/CAD basis (-y,z,-x). The unit
    // quaternion (w,x,y,z) = (1,-1,1,1)/2 is a proper 120-degree rotation:
    // det(R)=+1, so its conjugation preserves handedness and triangle winding.
    cadFromSimulationRoot_.orientation = {0.5F, -0.5F, 0.5F, 0.5F};
}

std::array<RigidTransform, CadVisualRig::kMeshCount> CadVisualRig::Evaluate(
    const PoseLink::Kinematics::ForwardKinematicsResult& currentPose) const noexcept
{
    std::array<RigidTransform, kMeshCount> result{};
    const RigidTransform simulationFromCad = Inverse(cadFromSimulationRoot_);
    // Mesh 0 is the fixed base. Meshes 1..6 move around the matching FK joint
    // reference. Applying the delta rather than the absolute pose is what
    // makes the baked CAD geometry continuous at q=0.
    for (std::size_t link = 0U; link < zeroJointFrames_.size(); ++link)
    {
        const RigidTransform deltaInSimulation = Compose(
            currentPose.jointFrames[link], Inverse(zeroJointFrames_[link]));
        result[link + 1U] = Compose(cadFromSimulationRoot_,
                                    Compose(deltaInSimulation, simulationFromCad));
    }
    return result;
}
