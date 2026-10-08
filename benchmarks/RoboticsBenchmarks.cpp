#include "robotics/planning/LinearPathPlanner.h"
#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"

#include <benchmark/benchmark.h>

#include <charconv>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string_view>

namespace
{
using namespace grasplink::robotics;
using namespace grasplink::robotics::kinematics;
using namespace grasplink::robotics::models;
using namespace grasplink::robotics::models::hanwha;
using namespace grasplink::robotics::planning;

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
