#pragma once

#include "robotics/planning/PlanningPolicy.h"
#include "robotics/planning/StateValidity.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace grasplink::robotics::planning
{

/**
 * @brief 관절 공간에서 시작 자세와 목표 자세를 잇는 경로를 점진적으로 계획한다.
 * @details 관절 공간 경로는 각 관절각을 보간해 만들기 때문에 TCP가 직선으로 움직인다는 보장은 없다.
 * 먼저 시작 자세와 목표 자세를 직접 잇는 경로를 검사하고, 충돌하거나 관절 제한을 넘으면 RRT-Connect로 우회 경로를 찾는다.
 * 상태 유효성 검사 함수는 Begin 또는 Advance를 호출한 스레드에서만 실행하므로, 장면과 Jolt를 다른 스레드에서 접근하지 않는다.
 */
struct JointPathPlannerOptions final
{
    PlanningPolicy validationPolicy{};
    std::size_t maximumIterations = 4096;
    std::size_t maximumNodesPerTree = 2048;
    std::size_t maximumShortcutAttempts = 128;
    double extensionStepFraction = 0.08;
    double goalBias = 0.1;
    std::uint32_t randomSeed = 1;
};

struct JointPathPlan final
{
    std::vector<JointVector> points;
    std::size_t iterations = 0;
    std::size_t validityChecks = 0;
};

enum class JointPathPlanningState : std::uint8_t
{
    Idle,
    Running,
    Completed,
    Failed,
    Cancelled
};

/**
 * @brief 관절 제한과 충돌을 검사하면서 MoveJ용 관절 경로를 찾는다.
 * @details 각 RRT 간선은 `validationPolicy`가 지정한 관절 간격으로 검사한다.
 * 직접 경로와 우회 경로 모두 같은 검사 정책을 사용하며, 단축한 경로도 마지막에 다시 검사한다.
 * Advance의 작업 예산은 트리 확장, 경로 단축, 최종 간선 확인을 합친 최대 단계 수다.
 */
class JointPathPlanningJob final
{
public:
    JointPathPlanningJob();
    ~JointPathPlanningJob();
    JointPathPlanningJob(JointPathPlanningJob&&) noexcept;
    JointPathPlanningJob& operator=(JointPathPlanningJob&&) noexcept;
    JointPathPlanningJob(const JointPathPlanningJob&) = delete;
    JointPathPlanningJob& operator=(const JointPathPlanningJob&) = delete;

    Result Begin(const models::RobotSpecification& specification,
        const JointVector& start,
        const JointVector& goal,
        const StateValidityChecker& stateValidityChecker,
        const JointPathPlannerOptions& options = {});
    JointPathPlanningState Advance(std::size_t workBudget);
    void Cancel() noexcept;
    [[nodiscard]] JointPathPlanningState GetState() const noexcept;
    [[nodiscard]] const Result& GetResult() const noexcept;
    [[nodiscard]] const JointPathPlan& GetPlan() const noexcept;
    [[nodiscard]] std::optional<JointPathPlan> TakePlan();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace grasplink::robotics::planning
