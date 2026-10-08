#include "robotics/planning/LinearPathPlanner.h"
#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/kinematics/detail/PoseMath.h"
#include "robotics/models/hanwha/Hcr12a.h"

#include <benchmark/benchmark.h>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <optional>
#include <random>
#include <sstream>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
using namespace grasplink::robotics;
using namespace grasplink::robotics::kinematics;
using namespace grasplink::robotics::models;
using namespace grasplink::robotics::models::hanwha;
using namespace grasplink::robotics::planning;
using grasplink::robotics::backends::simulation::SimRobotController;

const JointVector kSeed(6, 0.0);
const JointVector kTargetJoints{0.04, -0.05, 0.03, 0.02, -0.03, 0.04};

CartesianPose MakeTargetPose()
{
    DampedLeastSquaresIk solver(kHcr12a);
    return solver.EvaluateTcp(kTargetJoints);
}

void Fk(benchmark::State& state)
{
    RobotKinematics forward(kHcr12a);
    for (auto _ : state)
    {
        const auto& result = forward.Update(kSeed);
        benchmark::DoNotOptimize(result.toolFrameInBaseFrame.positionMeters);
        benchmark::DoNotOptimize(result.toolFrameInBaseFrame.rotation);
    }
}

void BenchmarkDlsIk(benchmark::State& state)
{
    DampedLeastSquaresIk solver(kHcr12a);
    const auto target = MakeTargetPose();
    const auto validation = solver.Solve(target, kSeed);
    if (!validation)
    {
        state.SkipWithError("DLS IK benchmark target did not converge during validation");
        return;
    }

    for (auto _ : state)
    {
        const auto result = solver.Solve(target, kSeed);
        if (!result)
        {
            state.SkipWithError("DLS IK benchmark failed to converge");
            break;
        }
        benchmark::DoNotOptimize(result.jointPositionRadians.data());
        benchmark::DoNotOptimize(result.positionErrorMeters);
    }
}

void LinearPathPlanning(benchmark::State& state)
{
    DampedLeastSquaresIk solver(kHcr12a);
    LinearPathMoveCommand command;
    command.targetPoses.push_back(MakeTargetPose());
    const CartesianPose startTcp = solver.EvaluateTcp(kSeed);

    LinearPathPlan plan;
    const auto validation = BuildLinearPath(
        command, kHcr12a, kSeed, startTcp, solver, {}, plan);
    if (!validation)
    {
        state.SkipWithError("LinearPathPlanner benchmark path failed validation");
        return;
    }

    std::size_t ikSolveCount = 0;
    std::size_t ikIterationCount = 0;
    std::size_t refinementCount = 0;
    std::size_t validityStateChecks = 0;
    std::size_t straightnessChecks = 0;
    for (auto _ : state)
    {
        const auto result = BuildLinearPath(
            command, kHcr12a, kSeed, startTcp, solver, {}, plan);
        if (!result)
        {
            state.SkipWithError("LinearPathPlanner failed to build the benchmark path");
            break;
        }
        benchmark::DoNotOptimize(plan.points.data());
        benchmark::DoNotOptimize(plan.points.size());
        ikSolveCount += plan.ikSolveCount;
        ikIterationCount += plan.ikIterationCount;
        refinementCount += plan.tcpRefinementCount;
        validityStateChecks += plan.validityStateChecks;
        straightnessChecks += plan.tcpStraightnessChecks;
    }
    state.SetItemsProcessed(state.iterations() * static_cast<std::int64_t>(plan.points.size()));
    state.counters["ik_solves_per_path"] = static_cast<double>(ikSolveCount) / state.iterations();
    state.counters["ik_iterations_per_path"] = static_cast<double>(ikIterationCount) / state.iterations();
    state.counters["tcp_refinements_per_path"] = static_cast<double>(refinementCount) / state.iterations();
    state.counters["planner_state_checks_per_path"] = static_cast<double>(validityStateChecks) / state.iterations();
    state.counters["tcp_straightness_checks_per_path"] = static_cast<double>(straightnessChecks) / state.iterations();
}

void IncrementalLinearPathPlanning(benchmark::State& state)
{
    using Clock = std::chrono::steady_clock;
    using namespace grasplink::robotics::kinematics::detail;

    DampedLeastSquaresIk solver(kHcr12a);
    LinearPathMoveCommand command;
    command.targetPoses.push_back(MakeTargetPose());
    const CartesianPose startTcp = solver.EvaluateTcp(kSeed);
    std::vector<double> sliceMicroseconds;
    std::vector<double> workFramesPerPath;
    std::size_t succeeded = 0;
    std::size_t plannerStateChecks = 0;
    double totalPlanningMicroseconds = 0.0;
    double maximumPositionErrorMeters = 0.0;
    double maximumOrientationErrorRadians = 0.0;
    std::size_t totalWorkFrames = 0;

    for (auto _ : state)
    {
        LinearPathPlanningJob job;
        if (!job.Begin(command, kHcr12a, kSeed, startTcp, solver, {}))
        {
            state.SkipWithError("incremental planner could not begin the benchmark path");
            break;
        }

        std::size_t workFrames = 0;
        const auto planningStart = Clock::now();
        while (job.GetState() == LinearPathPlanningState::Running && workFrames < 4096)
        {
            const auto frameStart = Clock::now();
            const auto frameDeadline = frameStart + std::chrono::microseconds(1500);
            do
            {
                job.Advance(1);
            } while (job.GetState() == LinearPathPlanningState::Running && Clock::now() < frameDeadline);
            sliceMicroseconds.push_back(std::chrono::duration<double, std::micro>(
                Clock::now() - frameStart).count());
            ++workFrames;
        }
        const auto planningEnd = Clock::now();
        if (job.GetState() != LinearPathPlanningState::Completed)
        {
            state.SkipWithError("incremental planner did not complete a valid benchmark path");
            break;
        }

        const auto& plan = job.GetPlan();
        for (std::size_t point = 1; point < plan.points.size(); ++point)
        {
            const auto& start = plan.points[point - 1];
            const auto& end = plan.points[point];
            const Pose3 desiredStart = FromCartesian(start.tcpPose);
            const Pose3 desiredEnd = FromCartesian(end.tcpPose);
            JointVector joints(start.joints.size());
            for (const double fraction : {0.25, 0.5, 0.75})
            {
                for (std::size_t joint = 0; joint < joints.size(); ++joint)
                    joints[joint] = start.joints[joint] +
                        (end.joints[joint] - start.joints[joint]) * fraction;
                const Pose3 actual = FromCartesian(solver.EvaluateTcp(joints));
                const Pose3 desired = Interpolate(desiredStart, desiredEnd, fraction);
                maximumPositionErrorMeters = std::max(maximumPositionErrorMeters,
                    Length(Subtract(actual.positionMeters, desired.positionMeters)));
                maximumOrientationErrorRadians = std::max(maximumOrientationErrorRadians,
                    Length(RotationError(desired.rotation, actual.rotation)));
            }
        }

        totalPlanningMicroseconds += std::chrono::duration<double, std::micro>(planningEnd - planningStart).count();
        plannerStateChecks += plan.validityStateChecks;
        totalWorkFrames += workFrames;
        workFramesPerPath.push_back(static_cast<double>(workFrames));
        ++succeeded;
        benchmark::DoNotOptimize(plan.points.data());
    }

    const auto percentile = [](std::vector<double>& values, double fraction)
    {
        if (values.empty())
            return 0.0;
        std::sort(values.begin(), values.end());
        const std::size_t index = static_cast<std::size_t>(std::ceil(
            fraction * static_cast<double>(values.size()))) - 1;
        return values[std::min(index, values.size() - 1)];
    };
    if (!sliceMicroseconds.empty())
    {
        state.counters["slice_p50_us"] = percentile(sliceMicroseconds, 0.50);
        state.counters["slice_p95_us"] = percentile(sliceMicroseconds, 0.95);
        state.counters["slice_p99_us"] = percentile(sliceMicroseconds, 0.99);
        state.counters["slice_max_us"] = *std::max_element(sliceMicroseconds.begin(), sliceMicroseconds.end());
    }
    if (succeeded != 0)
    {
        const double averageWorkFrames = static_cast<double>(totalWorkFrames) / succeeded;
        const double workFramesP50 = percentile(workFramesPerPath, 0.50);
        const double workFramesP95 = percentile(workFramesPerPath, 0.95);
        const double workFramesP99 = percentile(workFramesPerPath, 0.99);
        const double workFramesMax = *std::max_element(workFramesPerPath.begin(), workFramesPerPath.end());
        constexpr double millisecondsPerFrameAt60Hz = 1000.0 / 60.0;
        state.counters["success_rate"] = static_cast<double>(succeeded) / state.iterations();
        state.counters["planning_us_per_path"] = totalPlanningMicroseconds / succeeded;
        state.counters["work_frames"] = static_cast<double>(totalWorkFrames);
        state.counters["work_frames_per_path"] = averageWorkFrames;
        state.counters["work_frames_p50"] = workFramesP50;
        state.counters["work_frames_p95"] = workFramesP95;
        state.counters["work_frames_p99"] = workFramesP99;
        state.counters["work_frames_max"] = workFramesMax;
        state.counters["estimated_latency_ms_at_60hz"] = averageWorkFrames * millisecondsPerFrameAt60Hz;
        state.counters["estimated_latency_ms_at_60hz_p50"] = workFramesP50 * millisecondsPerFrameAt60Hz;
        state.counters["estimated_latency_ms_at_60hz_p95"] = workFramesP95 * millisecondsPerFrameAt60Hz;
        state.counters["estimated_latency_ms_at_60hz_p99"] = workFramesP99 * millisecondsPerFrameAt60Hz;
        state.counters["estimated_latency_ms_at_60hz_max"] = workFramesMax * millisecondsPerFrameAt60Hz;
        state.counters["planner_state_checks_per_path"] = static_cast<double>(plannerStateChecks) / succeeded;
        state.counters["max_fk_tcp_position_error_mm"] = maximumPositionErrorMeters * 1000.0;
        state.counters["max_fk_tcp_orientation_error_mrad"] = maximumOrientationErrorRadians * 1000.0;
    }
}

struct RuntimePathCase
{
    CartesianPose targetPose;
    JointVector reachableTargetJoints;
};

std::vector<RuntimePathCase> RuntimePathCases(std::size_t caseCount)
{
    std::mt19937_64 random(0x47524153504C494EULL);
    DampedLeastSquaresIk inverse(kHcr12a);
    std::vector<RuntimePathCase> generated;
    generated.reserve(caseCount);
    for (std::size_t sample = 0; sample < caseCount; ++sample)
    {
        JointVector targetJoints(kHcr12a.jointCount);
        for (std::size_t joint = 0; joint < kHcr12a.jointCount; ++joint)
        {
            const auto& limits = kHcr12a.joints[joint];
            std::uniform_real_distribution<double> position(
                limits.minPositionRadians * 0.8, limits.maxPositionRadians * 0.8);
            targetJoints[joint] = position(random);
        }
        generated.push_back({inverse.EvaluateTcp(targetJoints), std::move(targetJoints)});
    }
    return generated;
}

void RuntimeLinearPathStress(benchmark::State& state)
{
    const auto cases = RuntimePathCases(static_cast<std::size_t>(state.range(0)));
    std::uint64_t plannerRejections = 0;
    std::uint64_t plannerIkRejections = 0;
    std::uint64_t plannerJointLimitStatuses = 0;
    std::uint64_t plannerIkLimitStalls = 0;
    std::uint64_t plannerVerifiedJointLimitViolations = 0;
    std::uint64_t rejectionsWithKnownReachableEndpoints = 0;
    std::uint64_t runtimeFaults = 0;
    std::uint64_t runtimeIkFaults = 0;
    std::uint64_t timeouts = 0;
    std::uint64_t completed = 0;
    std::uint64_t stateValidityCallbackCalls = 0;
    std::string lastPlannerRejection;

    for (auto _ : state)
    {
        for (std::size_t caseIndex = 0; caseIndex < cases.size(); ++caseIndex)
        {
            const auto& testCase = cases[caseIndex];
            SimRobotController controller(kHcr12a);
            if (!controller.Connect())
            {
                state.SkipWithError("SimRobotController could not connect");
                return;
            }
            controller.SetJointStateValidityChecker([&stateValidityCallbackCalls](const JointVector&)
            {
                ++stateValidityCallbackCalls;
                return JointStateInvalidity::None;
            });

            LinearPathMoveCommand command;
            command.targetPoses.push_back(testCase.targetPose);
            command.maxLinearVelocityMetersPerSecond = 2.0;
            command.maxAngularVelocityRadiansPerSecond = 8.0;
            command.maxLinearAccelerationMetersPerSecondSquared = 30.0;
            command.maxAngularAccelerationRadiansPerSecondSquared = 120.0;
            const JointVector startJoints = controller.GetStateView().jointPositionRadians;
            const auto accepted = controller.MoveLinearPath(command);
            if (!accepted)
            {
                std::ostringstream details;
                details << "case " << caseIndex << " start_deg=[" << std::fixed << std::setprecision(6);
                for (std::size_t joint = 0; joint < startJoints.size(); ++joint)
                    details << (joint == 0 ? "" : ",") << startJoints[joint] * 180.0 / 3.14159265358979323846;
                details << "] endpoint_seed_deg=[";
                for (std::size_t joint = 0; joint < testCase.reachableTargetJoints.size(); ++joint)
                    details << (joint == 0 ? "" : ",") <<
                        testCase.reachableTargetJoints[joint] * 180.0 / 3.14159265358979323846;
                details << "]; " << accepted.message;
                lastPlannerRejection = details.str();
                ++plannerRejections;
                if (accepted.code == ErrorCode::IkDidNotConverge)
                    ++plannerIkRejections;
                else if (accepted.code == ErrorCode::JointLimitReached)
                {
                    ++plannerJointLimitStatuses;
                    if (accepted.message.find("IK: joint limits block local improvement") != std::string::npos ||
                        accepted.message.find("IK: iteration limit while constrained by joint limits") != std::string::npos)
                        ++plannerIkLimitStalls;
                    else if (accepted.message.find("SimRobotController: planned state exceeds a joint limit") != std::string::npos)
                        ++plannerVerifiedJointLimitViolations;
                }
                if (ValidateJointState(kHcr12a, testCase.reachableTargetJoints, {}) == JointStateInvalidity::None)
                    ++rejectionsWithKnownReachableEndpoints;
                continue;
            }

            constexpr double fixedDeltaSeconds = 0.004;
            constexpr std::size_t maxTicks = 5000;
            std::size_t tick = 0;
            for (; tick < maxTicks && controller.GetStateView().mode == RobotMode::Moving; ++tick)
                controller.Update(fixedDeltaSeconds);

            const auto& result = controller.GetStateView();
            if (result.mode == RobotMode::Fault)
            {
                ++runtimeFaults;
                if (result.errorCode == ErrorCode::IkDidNotConverge)
                    ++runtimeIkFaults;
            }
            else if (result.mode == RobotMode::Idle)
            {
                ++completed;
            }
            else
            {
                ++timeouts;
            }
        }
    }

    const double totalCases = static_cast<double>(state.iterations() * cases.size());
    state.counters["cases"] = totalCases;
    state.counters["planner_rejections"] = static_cast<double>(plannerRejections);
    state.counters["planner_ik_rejections"] = static_cast<double>(plannerIkRejections);
    state.counters["planner_joint_limit_statuses"] = static_cast<double>(plannerJointLimitStatuses);
    state.counters["planner_ik_limit_stalls"] = static_cast<double>(plannerIkLimitStalls);
    state.counters["planner_verified_joint_limit_violations"] = static_cast<double>(plannerVerifiedJointLimitViolations);
    state.counters["rejections_with_known_reachable_endpoints"] = static_cast<double>(rejectionsWithKnownReachableEndpoints);
    state.counters["runtime_faults"] = static_cast<double>(runtimeFaults);
    state.counters["runtime_ik_faults"] = static_cast<double>(runtimeIkFaults);
    state.counters["timeouts"] = static_cast<double>(timeouts);
    state.counters["completed"] = static_cast<double>(completed);
    state.counters["dummy_state_validity_callback_calls"] = static_cast<double>(stateValidityCallbackCalls);
    state.counters["dummy_state_validity_callback_calls_per_case"] = totalCases > 0.0 ?
        static_cast<double>(stateValidityCallbackCalls) / totalCases : 0.0;
    if (!lastPlannerRejection.empty())
        state.SetLabel(lastPlannerRejection);
    state.SetItemsProcessed(static_cast<std::int64_t>(totalCases));
}

bool ParseIterations(std::string_view value, std::int64_t& iterations)
{
    const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), iterations);
    return error == std::errc{} && end == value.data() + value.size() && iterations > 0;
}

}

int main(int argc, char** argv)
{
    std::int64_t iterations = 0;
    int output = 1;
    for (int input = 1; input < argc; ++input)
    {
        constexpr std::string_view option = "--iterations=";
        const std::string_view argument(argv[input]);
        if (argument.compare(0, option.size(), option) == 0)
        {
            if (!ParseIterations(argument.substr(option.size()), iterations))
            {
                std::cerr << "--iterations must be a positive integer\n";
                return 2;
            }
            continue;
        }
        argv[output++] = argv[input];
    }
    argc = output;
    argv[argc] = nullptr;

    benchmark::Initialize(&argc, argv);
    if (benchmark::ReportUnrecognizedArguments(argc, argv))
        return 2;

    auto* fk = benchmark::RegisterBenchmark("FK", Fk);
    auto* ik = benchmark::RegisterBenchmark("DLS_IK", BenchmarkDlsIk);
    auto* planner = benchmark::RegisterBenchmark("LinearPathPlanner", LinearPathPlanning);
    auto* incrementalPlanner = benchmark::RegisterBenchmark(
        "IncrementalLinearPathPlanner", IncrementalLinearPathPlanning);
    benchmark::RegisterBenchmark("RuntimeLinearPathStress", RuntimeLinearPathStress)
        ->Arg(64)
        ->Arg(256);
    if (iterations > 0)
    {
        fk->Iterations(iterations);
        ik->Iterations(iterations);
        planner->Iterations(iterations);
        incrementalPlanner->Iterations(iterations);
    }

    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
    return 0;
}
