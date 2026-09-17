#include "grasplink/controller/FeedbackPresenter.h"

namespace grasplink::controller
{
FeedbackPresenter::FeedbackPresenter(IFeedbackOutput& output) noexcept : output_(output) {}

bool FeedbackPresenter::OnRobotStatus(const RobotFeedbackStatus status) noexcept
{
    const FeedbackView next = MakeView(status);
    if (hasPresented_ && current_.status == next.status) {
        return false;
    }

    current_ = next;
    hasPresented_ = true;
    output_.Present(current_);
    return true;
}

void FeedbackPresenter::RequestRefresh() noexcept
{
    hasPresented_ = false;
}

FeedbackView FeedbackPresenter::CurrentView() const noexcept
{
    return current_;
}

FeedbackView FeedbackPresenter::MakeView(const RobotFeedbackStatus status) noexcept
{
    switch (status) {
    case RobotFeedbackStatus::Idle:
        return FeedbackView{status, LedIndication::Waiting, BuzzerIndication::Silent};
    case RobotFeedbackStatus::TargetReceived:
    case RobotFeedbackStatus::Moving:
        return FeedbackView{status, LedIndication::Active, BuzzerIndication::Silent};
    case RobotFeedbackStatus::GraspSuccess:
        return FeedbackView{status, LedIndication::Success, BuzzerIndication::Success};
    case RobotFeedbackStatus::GraspFailed:
    case RobotFeedbackStatus::TransportError:
        return FeedbackView{status, LedIndication::Failure, BuzzerIndication::Failure};
    case RobotFeedbackStatus::Unknown:
    default:
        return FeedbackView{RobotFeedbackStatus::Unknown, LedIndication::Off,
                            BuzzerIndication::Silent};
    }
}
} // namespace grasplink::controller
