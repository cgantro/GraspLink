#include "robotics/planning/LinearPathPlanner.h"
#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"

#include <benchmark/benchmark.h>

#include <charconv>
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
    }
    state.SetItemsProcessed(state.iterations() * static_cast<std::int64_t>(plan.points.size()));
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
    std::uint64_t validityCheckerCalls = 0;
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
            controller.SetJointStateValidityChecker([&validityCheckerCalls](const JointVector&)
            {
                ++validityCheckerCalls;
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
    state.counters["validity_checker_calls"] = static_cast<double>(validityCheckerCalls);
    state.counters["validity_checks_per_case"] = totalCases > 0.0 ?
        static_cast<double>(validityCheckerCalls) / totalCases : 0.0;
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
    benchmark::RegisterBenchmark("RuntimeLinearPathStress", RuntimeLinearPathStress)
        ->Arg(64)
        ->Arg(256);
    if (iterations > 0)
    {
        fk->Iterations(iterations);
        ik->Iterations(iterations);
        planner->Iterations(iterations);
    }

    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
    return 0;
}
