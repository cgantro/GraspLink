#include "RobotKinematics.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace PoseLink::Kinematics
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-10;

Position3D V(double x, double y, double z) noexcept { return {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)}; }
double Dot(const Position3D& a, const Position3D& b) noexcept { return double(a.x)*b.x + double(a.y)*b.y + double(a.z)*b.z; }
Position3D Cross(const Position3D& a, const Position3D& b) noexcept { return V(double(a.y)*b.z-double(a.z)*b.y, double(a.z)*b.x-double(a.x)*b.z, double(a.x)*b.y-double(a.y)*b.x); }
double Length(const Position3D& v) noexcept { return std::sqrt(Dot(v,v)); }
Position3D Scale(const Position3D& v, double s) noexcept { return V(v.x*s,v.y*s,v.z*s); }
Position3D Add(const Position3D& a, const Position3D& b) noexcept { return V(double(a.x)+b.x,double(a.y)+b.y,double(a.z)+b.z); }
Position3D Sub(const Position3D& a, const Position3D& b) noexcept { return V(double(a.x)-b.x,double(a.y)-b.y,double(a.z)-b.z); }
Quaternion Normalized(Quaternion q) noexcept { const double n=std::sqrt(double(q.w)*q.w+double(q.x)*q.x+double(q.y)*q.y+double(q.z)*q.z); return n>kEpsilon ? Quaternion{float(q.w/n),float(q.x/n),float(q.y/n),float(q.z/n)} : Quaternion{}; }
Quaternion Multiply(const Quaternion& a, const Quaternion& b) noexcept { return Normalized({float(a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z), float(a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y), float(a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x), float(a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w)}); }
Quaternion Conjugate(const Quaternion& q) noexcept { return {q.w,-q.x,-q.y,-q.z}; }
Position3D Rotate(const Quaternion& raw, const Position3D& v) noexcept { const auto q=Normalized(raw); const Position3D qv{q.x,q.y,q.z}; return Add(Add(v,Scale(Cross(qv,v),2.0*q.w)),Scale(Cross(qv,Cross(qv,v)),2.0)); }
Quaternion AxisAngle(const Position3D& rawAxis, double angle) noexcept { const double n=Length(rawAxis); if(n <= kEpsilon) return {}; const double s=std::sin(angle*.5)/n; return Normalized({float(std::cos(angle*.5)),float(rawAxis.x*s),float(rawAxis.y*s),float(rawAxis.z*s)}); }
RigidTransform Translation(const Position3D& p) noexcept { return {p,{}}; }
bool Finite(double value) noexcept { return std::isfinite(value); }
bool SolveLinear6(std::array<double,36> a, const std::array<double,6>& b, std::array<double,6>& x) noexcept {
    std::array<double,6> rhs=b;
    for(std::size_t col=0;col<6;++col){ std::size_t pivot=col; for(std::size_t r=col+1;r<6;++r) if(std::abs(a[r*6+col])>std::abs(a[pivot*6+col])) pivot=r; if(std::abs(a[pivot*6+col])<kEpsilon) return false; if(pivot!=col){for(std::size_t c=col;c<6;++c)std::swap(a[col*6+c],a[pivot*6+c]);std::swap(rhs[col],rhs[pivot]);} const double d=a[col*6+col]; for(std::size_t c=col;c<6;++c)a[col*6+c]/=d; rhs[col]/=d; for(std::size_t r=0;r<6;++r){if(r==col)continue; const double f=a[r*6+col]; for(std::size_t c=col;c<6;++c)a[r*6+c]-=f*a[col*6+c];rhs[r]-=f*rhs[col];}}
    x=rhs; return true;
}
} // namespace

RigidTransform Compose(const RigidTransform& a, const RigidTransform& b) noexcept { return {Add(a.position,Rotate(a.orientation,b.position)),Multiply(a.orientation,b.orientation)}; }
RigidTransform Inverse(const RigidTransform& t) noexcept { const auto r=Conjugate(Normalized(t.orientation)); return {Scale(Rotate(r,t.position),-1.0),r}; }
bool IsFinite(const RigidTransform& t) noexcept { return Finite(t.position.x)&&Finite(t.position.y)&&Finite(t.position.z)&&Finite(t.orientation.w)&&Finite(t.orientation.x)&&Finite(t.orientation.y)&&Finite(t.orientation.z); }
PoseError CalculatePoseError(const RigidTransform& current,const RigidTransform& target) noexcept { PoseError error{}; error.positionError=Sub(target.position,current.position); Quaternion q=Multiply(target.orientation,Conjugate(Normalized(current.orientation))); if(q.w<0){q.w=-q.w;q.x=-q.x;q.y=-q.y;q.z=-q.z;} const Position3D v{q.x,q.y,q.z}; const double vLength=Length(v); const double angle=2.0*std::atan2(vLength,std::max(0.0,double(q.w))); error.rotationError=vLength>kEpsilon?Scale(v,angle/vLength):Position3D{}; return error; }

RobotSpecification RobotSpecification::MakeHcr12aNominal() {
    RobotSpecification s{};
    s.jointAxisLocal={V(0,0,1),V(0,1,0),V(0,1,0),V(1,0,0),V(0,1,0),V(1,0,0)};
    // Sum of horizontal nominal arm/tool offsets is 1.30 m: HCR-12A reach reference.
    s.linkOffsetLocal={V(0,0,0.20),V(0.50,0,0),V(0.45,0,0),V(0.20,0,0),V(0,0,0),V(0,0,0)};
    s.toolFromJoint6={V(0.15,0,0),{}};
    const auto rad=[](double degree){return degree*kPi/180.0;};
    // Hanwha HCR-12A public motion range: J1 ±180, J2 ±150, J3 ±165,
    // J4 ±190, J5 ±170, J6 ±360 degrees. Values become a hard IK clamp.
    s.jointLimits={JointLimit{rad(-180),rad(180)},JointLimit{rad(-150),rad(150)},JointLimit{rad(-165),rad(165)},JointLimit{rad(-190),rad(190)},JointLimit{rad(-170),rad(170)},JointLimit{rad(-360),rad(360)}};
    return s;
}
bool RobotSpecification::IsValid() const noexcept { for(std::size_t i=0;i<6;++i){if(Length(jointAxisLocal[i])<=kEpsilon||!Finite(linkOffsetLocal[i].x)||!Finite(linkOffsetLocal[i].y)||!Finite(linkOffsetLocal[i].z)||!Finite(jointLimits[i].minimumRadians)||!Finite(jointLimits[i].maximumRadians)||jointLimits[i].minimumRadians>jointLimits[i].maximumRadians)return false;} return IsFinite(toolFromJoint6); }
JointState RobotSpecification::ClampToLimits(const JointState& joints) const noexcept { JointState r{};for(std::size_t i=0;i<6;++i)r.radians[i]=std::clamp(Finite(joints.radians[i])?joints.radians[i]:0.0,jointLimits[i].minimumRadians,jointLimits[i].maximumRadians);return r; }

ForwardKinematics::ForwardKinematics(RobotSpecification specification):specification_(specification){}
const RobotSpecification& ForwardKinematics::Specification()const noexcept{return specification_;}
ForwardKinematicsResult ForwardKinematics::Evaluate(const JointState& raw) const { ForwardKinematicsResult result{}; const auto joints=specification_.ClampToLimits(raw); RigidTransform world{}; for(std::size_t i=0;i<6;++i){result.jointFrames[i]=Compose(world,{ {},AxisAngle(specification_.jointAxisLocal[i],joints.radians[i])});world=Compose(result.jointFrames[i],Translation(specification_.linkOffsetLocal[i]));result.linkFrames[i]=world;}result.tcp=Compose(world,specification_.toolFromJoint6);return result; }
std::array<double,36> ForwardKinematics::GeometricJacobian(const JointState& joints) const {std::array<double,36> j{};const auto fk=Evaluate(joints);for(std::size_t c=0;c<6;++c){const auto axis=Rotate(fk.jointFrames[c].orientation,specification_.jointAxisLocal[c]);const auto linear=Cross(axis,Sub(fk.tcp.position,fk.jointFrames[c].position));j[c]=linear.x;j[6+c]=linear.y;j[12+c]=linear.z;j[18+c]=axis.x;j[24+c]=axis.y;j[30+c]=axis.z;}return j;}

DampedLeastSquaresIkSolver::DampedLeastSquaresIkSolver(ForwardKinematics kinematics,IkOptions options):kinematics_(std::move(kinematics)),options_(options){}
IkResult DampedLeastSquaresIkSolver::Solve(const RigidTransform& target,const JointState& seed)const {IkResult result{};JointState state=kinematics_.Specification().ClampToLimits(seed);result.joints=state;if(!IsFinite(target)||!kinematics_.Specification().IsValid()||options_.maximumIterations==0||!Finite(options_.damping)||!Finite(options_.maximumStepRadians)||!Finite(options_.positionToleranceMetres)||!Finite(options_.orientationToleranceRadians)||!Finite(options_.orientationWeightMetresPerRadian)||options_.damping<=0||options_.maximumStepRadians<=0||options_.positionToleranceMetres<0||options_.orientationToleranceRadians<0||options_.orientationWeightMetresPerRadian<=0)return result;double conservativeReach=Length(kinematics_.Specification().toolFromJoint6.position);for(const auto& offset:kinematics_.Specification().linkOffsetLocal)conservativeReach+=Length(offset);if(Length(target.position)>conservativeReach+options_.positionToleranceMetres){result.status=IkStatus::Unreachable;result.finalError=CalculatePoseError(kinematics_.Evaluate(state).tcp,target);return result;}for(std::size_t iter=0;iter<options_.maximumIterations;++iter){const auto fk=kinematics_.Evaluate(state);const auto error=CalculatePoseError(fk.tcp,target);result.finalError=error;result.iterations=iter+1;if(Length(error.positionError)<=options_.positionToleranceMetres&&Length(error.rotationError)<=options_.orientationToleranceRadians){result.status=IkStatus::Converged;result.joints=state;return result;}const auto j=kinematics_.GeometricJacobian(state);std::array<double,6> e{error.positionError.x,error.positionError.y,error.positionError.z,error.rotationError.x*options_.orientationWeightMetresPerRadian,error.rotationError.y*options_.orientationWeightMetresPerRadian,error.rotationError.z*options_.orientationWeightMetresPerRadian};std::array<double,36> scaled=j;for(std::size_t row=3;row<6;++row)for(std::size_t col=0;col<6;++col)scaled[row*6+col]*=options_.orientationWeightMetresPerRadian;std::array<double,36> a{};for(std::size_t r=0;r<6;++r)for(std::size_t c=0;c<6;++c)for(std::size_t k=0;k<6;++k)a[r*6+c]+=scaled[r*6+k]*scaled[c*6+k];for(std::size_t d=0;d<6;++d)a[d*6+d]+=options_.damping*options_.damping;std::array<double,6> y{};if(!SolveLinear6(a,e,y))break;JointState next=state;for(std::size_t q=0;q<6;++q){double delta=0;for(std::size_t r=0;r<6;++r)delta+=scaled[r*6+q]*y[r];next.radians[q]+=std::clamp(delta,-options_.maximumStepRadians,options_.maximumStepRadians);}next=kinematics_.Specification().ClampToLimits(next);double moved=0;for(std::size_t q=0;q<6;++q)moved+=std::abs(next.radians[q]-state.radians[q]);state=next;result.joints=state;if(moved<1e-9)break;}result.finalError=CalculatePoseError(kinematics_.Evaluate(state).tcp,target);result.status=IkStatus::IterationLimit;return result;}
} // namespace PoseLink::Kinematics
