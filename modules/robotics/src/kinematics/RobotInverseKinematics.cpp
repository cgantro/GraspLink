#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/kinematics/detail/AlternativeIkSeeds.h"
#include "robotics/kinematics/detail/PoseMath.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace grasplink::robotics::kinematics
{
namespace
{
using namespace detail;
using SixVector = std::array<double, 6>;
using SixMatrix = std::array<SixVector, 6>;

constexpr std::size_t kMaximumDampingAttempts = 8;
constexpr std::size_t kMaximumLineSearchSteps = 10;
constexpr double kSingularSystemDampingGrowth = 10.0;
constexpr double kRejectedStepDampingGrowth = 4.0;
constexpr double kAcceptedStepDampingDecay = 0.5;
constexpr double kMinimumDampingRatio = 0.01;
// 6×6 DLS normal equation에서 pivot이 이보다 작으면 double 정밀도에서 불안정한 나눗셈으로 본다.
constexpr double kNormalEquationPivotCutoff = 1e-24;

bool Positive(double value) { return std::isfinite(value) && value > 0.0; }

bool SolveSystem(SixMatrix matrix, SixVector right, SixVector& answer)
{
    // 6개의 TCP 오차 성분에 대한 작은 선형 연립방정식을 부분 피벗 소거법으로 푼다. 큰 피벗 행을 고르면 작은 수로 나눌 때의 수치 오차를 줄일 수 있다.
    for (std::size_t column = 0; column < 6; ++column)
    {
        std::size_t pivot = column;
        for (std::size_t row = column + 1; row < 6; ++row)
            if (std::abs(matrix[row][column]) > std::abs(matrix[pivot][column]))
                pivot = row;
        if (!std::isfinite(matrix[pivot][column]) ||
            std::abs(matrix[pivot][column]) <= kNormalEquationPivotCutoff)
            return false;
        std::swap(matrix[pivot], matrix[column]);
        std::swap(right[pivot], right[column]);
        for (std::size_t row = column + 1; row < 6; ++row)
        {
            const double factor = matrix[row][column] / matrix[column][column];
            for (std::size_t entry = column; entry < 6; ++entry)
                matrix[row][entry] -= factor * matrix[column][entry];
            right[row] -= factor * right[column];
        }
    }
    for (int row = 5; row >= 0; --row)
    {
        double residual = right[static_cast<std::size_t>(row)];
        for (std::size_t column = static_cast<std::size_t>(row) + 1; column < 6; ++column)
            residual -= matrix[static_cast<std::size_t>(row)][column] * answer[column];
        answer[static_cast<std::size_t>(row)] = residual / matrix[static_cast<std::size_t>(row)][static_cast<std::size_t>(row)];
        if (!std::isfinite(answer[static_cast<std::size_t>(row)]))
            return false;
    }
    return true;
}

struct Error
{
    SixVector weighted{};
    double position = 0.0;
    double orientation = 0.0;
    double cost = 0.0;
};

Error Measure(const Pose3& target, const Pose3& current, double weight)
{
    const Vec3 position = Subtract(target.positionMeters, current.positionMeters);
    const Vec3 rotation = RotationError(target.rotation, current.rotation);
    Error error;
    error.position = Length(position);
    error.orientation = Length(rotation);
    error.weighted = {position.x, position.y, position.z, rotation.x * weight, rotation.y * weight, rotation.z * weight};
    for (double entry : error.weighted)
        error.cost += entry * entry;
    return error;
}
}

DampedLeastSquaresIk::DampedLeastSquaresIk(const models::RobotSpecification& specification, models::Pose3 tcpInToolFrame)
    : specification_(specification), forward_(specification), tcpInToolFrame_(tcpInToolFrame)
{
    if (!Finite(tcpInToolFrame_.positionMeters))
        throw std::invalid_argument("DampedLeastSquaresIk: invalid TCP offset");
    tcpInToolFrame_.rotation = Normalize(tcpInToolFrame_.rotation);
    for (std::size_t i = 0; i < specification_.jointCount; ++i)
    {
        const auto& joint = specification_.joints[i];
        if (!std::isfinite(joint.minPositionRadians) || !std::isfinite(joint.maxPositionRadians) ||
            joint.minPositionRadians > joint.maxPositionRadians)
            throw std::invalid_argument("DampedLeastSquaresIk: invalid joint limits");
        if (i > 0)
            maximumReachMeters_ += Length(Subtract(joint.bindPivotMeters, specification_.joints[i - 1].bindPivotMeters));
    }
    // 첫 관절 중심을 기준으로 링크 사이 길이와 ToolFrame/TCP offset 길이를 모두 더한다. 회전이나 관절 제한을 무시한 넉넉한 구이므로 이 거리 밖은 확실히 도달할 수 없다.
    if (specification_.hasToolFrame)
        maximumReachMeters_ += Length(specification_.toolFrameInLastJoint.positionMeters) + Length(tcpInToolFrame_.positionMeters);
    if (!std::isfinite(maximumReachMeters_))
        throw std::invalid_argument("DampedLeastSquaresIk: non-finite robot reach");
}

CartesianPose DampedLeastSquaresIk::EvaluateTcp(const JointVector& jointPositionRadians)
{
    if (!specification_.hasToolFrame)
        throw std::invalid_argument("DampedLeastSquaresIk: missing ToolFrame");
    return ToCartesian(Compose(forward_.Update(jointPositionRadians).toolFrameInBaseFrame, tcpInToolFrame_));
}

IkResult DampedLeastSquaresIk::SolveFromSeed(const CartesianPose& targetInBase, const JointVector& currentSeed, const IkOptions& options)
{
    IkResult result;
    result.positionErrorMeters = std::numeric_limits<double>::infinity();
    result.orientationErrorRadians = std::numeric_limits<double>::infinity();
    auto fail = [&](IkStatus status, const char* message) {
        result.status = status;
        result.message = message;
        return result;
    };
    if (currentSeed.size() != specification_.jointCount || options.maxIterations == 0 ||
        !Positive(options.positionToleranceMeters) || !Positive(options.orientationToleranceRadians) ||
        !Positive(options.damping) || !Positive(options.orientationWeightMetersPerRadian) || !Positive(options.maxJointStepRadians))
        return fail(IkStatus::InvalidInput, "IK: invalid seed size or solver options");
    for (std::size_t i = 0; i < currentSeed.size(); ++i)
    {
        if (!std::isfinite(currentSeed[i]) || currentSeed[i] < specification_.joints[i].minPositionRadians ||
            currentSeed[i] > specification_.joints[i].maxPositionRadians)
            return fail(IkStatus::InvalidInput, "IK: seed outside joint limits or non-finite");
    }
    Pose3 target;
    try { target = FromCartesian(targetInBase); }
    catch (const std::invalid_argument&) { return fail(IkStatus::InvalidInput, "IK: invalid target pose"); }
    result.jointPositionRadians = currentSeed;
    if (!specification_.hasToolFrame)
        return fail(IkStatus::MissingToolFrame, "IK: robot specification has no ToolFrame");
    const double targetRadius = Length(Subtract(target.positionMeters, specification_.joints[0].bindPivotMeters));
    if (!std::isfinite(targetRadius) || targetRadius > maximumReachMeters_ + options.positionToleranceMeters)
        return fail(IkStatus::Unreachable, "IK: target beyond conservative robot reach");

    JointVector angles = currentSeed;
    const double weight = options.orientationWeightMetersPerRadian;
    double damping = options.damping;
    bool boundaryBlocked = false;
    std::vector<SixVector> columns(specification_.jointCount);
    for (std::size_t iteration = 0; iteration <= options.maxIterations; ++iteration)
    {
        const auto& state = forward_.Update(angles);
        const Pose3 current = Compose(state.toolFrameInBaseFrame, tcpInToolFrame_);
        const Error error = Measure(target, current, weight);
        result.jointPositionRadians = angles;
        result.iterations = iteration;
        result.positionErrorMeters = error.position;
        result.orientationErrorRadians = error.orientation;
        if (error.position <= options.positionToleranceMeters && error.orientation <= options.orientationToleranceRadians)
        {
            result.status = IkStatus::Success;
            return result;
        }
        if (!std::isfinite(error.cost))
            return fail(IkStatus::InvalidInput, "IK: target or solver scaling exceeds numeric range");
        if (iteration == options.maxIterations)
            break;

        for (auto& column : columns)
            column.fill(0.0);
        // Jacobian은 관절각을 조금 바꿨을 때 TCP 위치와 방향이 얼마나 변하는지 나타내는 변화율 표다.
        // 회전 관절의 위치 열은 axis × (TCP - pivot) [m/rad]이고 방향 열은 base 축이다. ToolFrame 대신 offset을 더한 TCP를 쓰므로 길이가 긴 공구도 반영된다.
        for (std::size_t i = 0; i < columns.size(); ++i)
        {
            const Vec3 axis = state.jointAxesInBaseFrame[i];
            const Vec3 linear = Cross(axis, Subtract(current.positionMeters, state.linkPosesInBaseFrame[i].positionMeters));
            columns[i] = {linear.x, linear.y, linear.z, axis.x * weight, axis.y * weight, axis.z * weight};
        }

        bool improved = false;
        boundaryBlocked = false;
        for (std::size_t attempt = 0; attempt < kMaximumDampingAttempts && !improved; ++attempt)
        {
            // Damped Least Squares는 Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e를 계산한다.
            // e는 위치·회전 오차이고 λ는 damping이다. λ²를 대각에 더하면 특이 자세에서도 역행렬 대신 안정적인 연립방정식을 풀 수 있지만 도달 가능한 해의 수렴을 보장하지는 않는다.
            SixMatrix normal{};
            for (std::size_t row = 0; row < 6; ++row)
            {
                for (std::size_t column = 0; column < 6; ++column)
                    for (const auto& jointColumn : columns)
                        normal[row][column] += jointColumn[row] * jointColumn[column];
                normal[row][row] += damping * damping;
            }
            SixVector taskStep{};
            if (!SolveSystem(normal, error.weighted, taskStep))
            {
                damping *= kSingularSystemDampingGrowth;
                continue;
            }
            JointVector jointStep(angles.size(), 0.0);
            double largest = 0.0;
            for (std::size_t i = 0; i < angles.size(); ++i)
            {
                for (std::size_t row = 0; row < 6; ++row)
                    jointStep[i] += columns[i][row] * taskStep[row];
                largest = std::max(largest, std::abs(jointStep[i]));
            }
            if (!std::isfinite(largest))
                return fail(IkStatus::DidNotConverge, "IK: non-finite numerical step");
            const double stepScale = largest > options.maxJointStepRadians ? options.maxJointStepRadians / largest : 1.0;
            for (std::size_t line = 0; line < kMaximumLineSearchSteps && !improved; ++line)
            {
                const double fraction = stepScale * std::ldexp(1.0, -line);
                JointVector candidate = angles;
                bool clipped = false;
                for (std::size_t i = 0; i < angles.size(); ++i)
                {
                    const double requested = angles[i] + jointStep[i] * fraction;
                    candidate[i] = std::clamp(requested, specification_.joints[i].minPositionRadians, specification_.joints[i].maxPositionRadians);
                    clipped = clipped || candidate[i] != requested;
                }
                boundaryBlocked = boundaryBlocked || clipped;
                const Error next = Measure(target, Compose(forward_.Update(candidate).toolFrameInBaseFrame, tcpInToolFrame_), weight);
                if (next.cost < error.cost)
                {
                    angles = std::move(candidate);
                    // 오차가 실제로 줄었으면 damping을 낮춰 특이 자세 근처의 작은 변화율도 더 정확히 따른다.
                    // 초기값을 항상 하한으로 쓰면 홈 자세의 손목처럼 거의 겹친 축에서 필요한 관절 변화가 지나치게 억제되어 도달 가능한 목표도 반복 한도에 막힐 수 있다.
                    // 초기값의 1%를 하한으로 남기고 maxJointStepRadians와 오차 감소 검사를 유지해 큰 관절 변화는 계속 제한한다.
                    damping = std::max(options.damping * kMinimumDampingRatio,
                        damping * kAcceptedStepDampingDecay);
                    improved = true;
                }
            }
            if (!improved)
                damping *= kRejectedStepDampingGrowth;
        }
        if (!improved)
            return fail(boundaryBlocked ? IkStatus::JointLimitReached : IkStatus::DidNotConverge,
                boundaryBlocked ? "IK: joint limits block local improvement" : "IK: local iteration stalled");
    }
    return fail(boundaryBlocked ? IkStatus::JointLimitReached : IkStatus::DidNotConverge,
        boundaryBlocked ? "IK: iteration limit while constrained by joint limits" : "IK: iteration limit reached");
}

IkResult DampedLeastSquaresIk::Solve(
    const CartesianPose& targetInBase, const JointVector& currentSeed, const IkOptions& options)
{
    IkResult best = SolveFromSeed(targetInBase, currentSeed, options);
    if (best.Ok() || (best.status != IkStatus::DidNotConverge && best.status != IkStatus::JointLimitReached))
        return best;

    double bestJointDistance = std::numeric_limits<double>::infinity();
    const auto distanceFromSeed = [&](const JointVector& candidate)
    {
        double squaredDistance = 0.0;
        for (std::size_t joint = 0; joint < candidate.size(); ++joint)
        {
            const double delta = candidate[joint] - currentSeed[joint];
            squaredDistance += delta * delta;
        }
        return squaredDistance;
    };
    const auto residual = [&](const IkResult& candidate)
    {
        return candidate.positionErrorMeters +
            options.orientationWeightMetersPerRadian * candidate.orientationErrorRadians;
    };
    // 첫 시드가 막힌 경우에만 HCR의 어깨·팔꿈치·손목 자세를 반사해 제한된 대체 시드를 검사한다.
    for (const auto& alternate : BuildAlternativeIkSeeds(currentSeed, specification_))
    {
        IkResult candidate = SolveFromSeed(targetInBase, alternate, options);
        if (candidate.Ok())
        {
            const double jointDistance = distanceFromSeed(candidate.jointPositionRadians);
            if (!best.Ok() || jointDistance < bestJointDistance)
            {
                bestJointDistance = jointDistance;
                best = std::move(candidate);
            }
        }
        else if (!best.Ok() && residual(candidate) < residual(best))
            best = std::move(candidate);
    }
    return best;
}

IkResult DampedLeastSquaresIk::SolveSingleSeed(
    const CartesianPose& targetInBase, const JointVector& seed, const IkOptions& options)
{
    return SolveFromSeed(targetInBase, seed, options);
}
}
