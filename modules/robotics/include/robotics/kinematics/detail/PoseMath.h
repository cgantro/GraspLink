#pragma once

#include "robotics/core/ControlTypes.h"
#include "robotics/models/ModelTypes.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace grasplink::robotics::kinematics::detail
{
using models::Pose3;
using models::QuaternionWxyz;
using models::Vec3;

inline bool Finite(const Vec3& v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline Vec3 Add(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 Subtract(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 Scale(const Vec3& a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline double Length(const Vec3& v) { return std::hypot(v.x, v.y, v.z); }
inline Vec3 Cross(const Vec3& a, const Vec3& b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline QuaternionWxyz Normalize(const QuaternionWxyz& q)
{
    const double magnitude = std::max({std::abs(q.w), std::abs(q.x), std::abs(q.y), std::abs(q.z)});
    if (!std::isfinite(q.w) || !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || magnitude <= 0.0)
        throw std::invalid_argument("Invalid pose quaternion");
    const QuaternionWxyz scaled{q.w / magnitude, q.x / magnitude, q.y / magnitude, q.z / magnitude};
    const double n = std::sqrt(scaled.w * scaled.w + scaled.x * scaled.x + scaled.y * scaled.y + scaled.z * scaled.z);
    return {scaled.w / n, scaled.x / n, scaled.y / n, scaled.z / n};
}

inline QuaternionWxyz Multiply(const QuaternionWxyz& a, const QuaternionWxyz& b)
{
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

inline Vec3 Rotate(const QuaternionWxyz& q, const Vec3& v)
{
    const Vec3 qv{q.x, q.y, q.z};
    const Vec3 t = Scale(Cross(qv, v), 2.0);
    return Add(v, Add(Scale(t, q.w), Cross(qv, t)));
}

inline Pose3 FromCartesian(const CartesianPose& p)
{
    const Vec3 position{p.positionMeters[0], p.positionMeters[1], p.positionMeters[2]};
    if (!Finite(position))
        throw std::invalid_argument("Non-finite pose position");
    // 공통 Controller API는 [x,y,z,w] 순서이고 모델 계산은 [w,x,y,z] 순서다. GLM을 추가하더라도 생성자 인자 순서를 이 변환과 혼동하면 안 된다.
    return {position, Normalize({p.orientationXyzw[3], p.orientationXyzw[0], p.orientationXyzw[1], p.orientationXyzw[2]})};
}

inline CartesianPose ToCartesian(const Pose3& p)
{
    return {{p.positionMeters.x, p.positionMeters.y, p.positionMeters.z},
        {p.rotation.x, p.rotation.y, p.rotation.z, p.rotation.w}};
}

inline Pose3 Compose(const Pose3& parent, const Pose3& local)
{
    return {Add(parent.positionMeters, Rotate(parent.rotation, local.positionMeters)),
        Normalize(Multiply(parent.rotation, local.rotation))};
}

inline Vec3 RotationError(const QuaternionWxyz& target, const QuaternionWxyz& current)
{
    if ((target.w == current.w && target.x == current.x && target.y == current.y && target.z == current.z) ||
        (target.w == -current.w && target.x == -current.x && target.y == -current.y && target.z == -current.z))
        return {};
    QuaternionWxyz difference = Normalize(Multiply(target, {current.w, -current.x, -current.y, -current.z}));
    // q와 -q는 같은 회전이다. w를 양수로 맞추면 180° 이하의 최단 회전을 Robot base 방향의 회전벡터 [rad]로 얻는다.
    if (difference.w < 0.0)
        difference = {-difference.w, -difference.x, -difference.y, -difference.z};
    const Vec3 axis{difference.x, difference.y, difference.z};
    const double sine = Length(axis);
    if (sine < 1e-12)
        return Scale(axis, 2.0);
    return Scale(axis, 2.0 * std::atan2(sine, difference.w) / sine);
}

inline QuaternionWxyz Slerp(QuaternionWxyz start, QuaternionWxyz end, double fraction)
{
    double dot = start.w * end.w + start.x * end.x + start.y * end.y + start.z * end.z;
    if (dot < 0.0)
    {
        end = {-end.w, -end.x, -end.y, -end.z};
        dot = -dot;
    }
    dot = std::clamp(dot, 0.0, 1.0);
    double a = 1.0 - fraction;
    double b = fraction;
    if (dot < 0.9995)
    {
        const double angle = std::acos(dot);
        const double sine = std::sin(angle);
        a = std::sin((1.0 - fraction) * angle) / sine;
        b = std::sin(fraction * angle) / sine;
    }
    return Normalize({a * start.w + b * end.w, a * start.x + b * end.x,
        a * start.y + b * end.y, a * start.z + b * end.z});
}

inline Pose3 Interpolate(const Pose3& start, const Pose3& end, double fraction)
{
    return {Add(Scale(start.positionMeters, 1.0 - fraction), Scale(end.positionMeters, fraction)),
        Slerp(start.rotation, end.rotation, fraction)};
}
}
