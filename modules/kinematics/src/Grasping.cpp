#include "Grasping.h"

#include <cmath>

namespace PoseLink::Kinematics
{
GraspPoseCalculator::GraspPoseCalculator(RigidTransform objectFromTcp) noexcept : objectFromTcp_(objectFromTcp) {}
RigidTransform GraspPoseCalculator::CalculateTcpTarget(const RigidTransform& worldFromObject) const noexcept { return Compose(worldFromObject,objectFromTcp_); }
const RigidTransform& GraspPoseCalculator::ObjectFromTcp() const noexcept{return objectFromTcp_;}
GraspController::GraspController(GraspThresholds thresholds):thresholds_(thresholds){}
GraspState GraspController::State()const noexcept{return state_;}
bool GraspController::IsAttached()const noexcept{return state_==GraspState::Attached;}
bool GraspController::TryAttach(const RigidTransform& ee,const RigidTransform& object,const RigidTransform& desired)noexcept {if(!IsFinite(ee)||!IsFinite(object)||!IsFinite(desired)||!std::isfinite(thresholds_.positionMetres)||!std::isfinite(thresholds_.orientationRadians)||thresholds_.positionMetres<0||thresholds_.orientationRadians<0)return false;const auto error=CalculatePoseError(ee,desired);const auto length=[](const Position3D& v){return std::sqrt(double(v.x)*v.x+double(v.y)*v.y+double(v.z)*v.z);};if(length(error.positionError)>thresholds_.positionMetres||length(error.rotationError)>thresholds_.orientationRadians)return false;endEffectorFromObject_=Compose(Inverse(ee),object);state_=GraspState::Attached;return true;}
void GraspController::ReleaseForNewTarget()noexcept{Release();}
void GraspController::Release()noexcept{state_=GraspState::Open;endEffectorFromObject_={};}
bool GraspController::UpdateAttachedObject(const RigidTransform& ee,RigidTransform& object)const noexcept{if(!IsAttached()||!IsFinite(ee))return false;object=Compose(ee,endEffectorFromObject_);return true;}
} // namespace PoseLink::Kinematics
