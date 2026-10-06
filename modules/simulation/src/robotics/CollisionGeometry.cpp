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

    // 전체 정점에서 162개 구면 방향과 6개 축 방향으로 가장 멀리 있는 점을 고른다. 이 대표 정점만 Jolt에 전달해 Convex Hull 계산 입력을 줄인다.
    // 선택한 바깥 점을 연결하면 오목한 메시 안쪽 공간을 그대로 남기지 못한다. 따라서 실제 메시에는 빈 곳이 있어도 볼록 충돌 외피가 그 공간을 채울 수 있다.
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
    // 서로 다른 점이 4개보다 적으면 3차원 부피를 만들 수 없다. 점이 더 있어도 한 평면 위에만 놓인 경우가 있으므로 크기 차이를 정규화한 뒤 평면 밖의 점이 있는지 확인한다.
    if (points.size() < 4) return false;
    glm::vec3 minimum = points.front();
    glm::vec3 maximum = points.front();
    for (const auto& point : points)
    {
        minimum = glm::min(minimum, point);
        maximum = glm::max(maximum, point);
    }
    // coordinateScaleMeters는 좌표 검사에 사용할 기준 길이 [m]다. 값이 0이면 부품 크기에 맞춰 허용 오차를 정한다.
// 양수면 그 길이에 고정한 오차를 쓴다. Arm 호출부는 1 m를 주고 Gripper는 0을 써 작은 부품 크기에 비례시킨다.
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
