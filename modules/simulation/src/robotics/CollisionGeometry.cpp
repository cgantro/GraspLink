#include "CollisionGeometry.h"

#include <glm/gtx/norm.hpp>
#include <algorithm>
#include <cmath>

namespace grasplink::simulation::detail
{
std::vector<glm::vec3> BuildConvexSupportPoints(const std::vector<glm::vec3>& vertices)
{
    std::vector<glm::vec3> result;
    if (vertices.size() < 4) return result;

    // 구면의 162개 방향과 6개 축 방향에서 가장 먼 정점을 골라 Jolt Convex Hull 입력을 제한한다.
    // 이 근사는 오목한 메시 내부를 보존하지 않고 바깥 점을 잇는 볼록 외피로 채울 수 있다.
    constexpr int DirectionCount = 162;
    constexpr float GoldenAngle = 2.39996323F;
    auto addExtreme = [&](const glm::vec3& direction)
    {
        const auto iterator = std::max_element(vertices.begin(), vertices.end(),
            [&](const glm::vec3& left, const glm::vec3& right)
            {
                return glm::dot(left, direction) < glm::dot(right, direction);
            });
        if (iterator == vertices.end()) return;
        if (std::none_of(result.begin(), result.end(), [&](const glm::vec3& point)
            { return glm::length2(point - *iterator) < 1.0e-10F; }))
            result.push_back(*iterator);
    };

    for (int i = 0; i < DirectionCount; ++i)
    {
        const float y = 1.0F - 2.0F * (static_cast<float>(i) + 0.5F) / DirectionCount;
        const float radius = std::sqrt(std::max(0.0F, 1.0F - y * y));
        const float angle = GoldenAngle * i;
        addExtreme({radius * std::cos(angle), y, radius * std::sin(angle)});
    }
    addExtreme({1.0F, 0.0F, 0.0F});
    addExtreme({-1.0F, 0.0F, 0.0F});
    addExtreme({0.0F, 1.0F, 0.0F});
    addExtreme({0.0F, -1.0F, 0.0F});
    addExtreme({0.0F, 0.0F, 1.0F});
    addExtreme({0.0F, 0.0F, -1.0F});
    return result;
}

bool HasHullVolume(const std::vector<glm::vec3>& points, float coordinateScaleMeters)
{
    // 점 4개 미만은 부피가 불가능하다. 이후에는 정규화 좌표에서 한 평면에 놓이는지도 검사한다.
    if (points.size() < 4) return false;
    glm::vec3 minimum = points.front();
    glm::vec3 maximum = points.front();
    for (const auto& point : points)
    {
        minimum = glm::min(minimum, point);
        maximum = glm::max(maximum, point);
    }
    // 0은 부품 크기별 기준, 양수는 고정 meter 기준이다. Arm의 1.0과 Gripper의 기본값은 허용 오차가 다르다.
    const float extent = coordinateScaleMeters > 0.0F ? coordinateScaleMeters : glm::length(maximum - minimum);
    if (!std::isfinite(extent) || extent <= 0.0F) return false;
    std::vector<glm::vec3> normalized;
    normalized.reserve(points.size());
    for (const auto& point : points) normalized.push_back(point / extent);
    const glm::vec3 origin = normalized.front();
    std::size_t edgeIndex = 1;
    while (edgeIndex < normalized.size() && glm::length2(normalized[edgeIndex] - origin) < 1.0e-10F)
        ++edgeIndex;
    if (edgeIndex == normalized.size()) return false;

    const glm::vec3 edge = normalized[edgeIndex] - origin;
    glm::vec3 normal{0.0F};
    bool foundPlane = false;
    for (std::size_t i = 1; i < normalized.size(); ++i)
    {
        normal = glm::cross(edge, normalized[i] - origin);
        if (glm::length2(normal) > 1.0e-10F)
        {
            foundPlane = true;
            break;
        }
    }
    if (!foundPlane) return false;
    for (const glm::vec3& point : normalized)
        if (std::abs(glm::dot(normal, point - origin)) > 1.0e-7F)
            return true;
    return false;
}

}
