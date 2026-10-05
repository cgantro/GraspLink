#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace grasplink::simulation::detail
{
/**
 * @brief 조밀한 정점 집합에서 방향별 support point를 골라 볼록 외피 입력을 줄인다.
 * @param vertices Body 원점 기준 정점 [m].
 * @return 구면 표본 방향의 극점 목록. 원래 오목한 mesh 대신 Convex Hull을 만들므로 빈 공간을 채울 수 있다.
 */
std::vector<glm::vec3> BuildConvexSupportPoints(const std::vector<glm::vec3>& vertices);

/**
 * @brief 점 집합이 부피 있는 3차원 hull인지 판별한다.
 * @param points 검사할 support point [m].
 * @param coordinateScaleMeters 양수이면 그 고정 길이로, 0이면 점 집합의 대각 길이로 좌표를 정규화한다.
 * Gripper는 기본값 0을 사용해 작은 부품도 크기 비례로 검사한다. arm은 1 m를 넘겨 기존 절대 허용오차를 유지한다.
 * @return 점·선·평면만 이루어진 입력은 false, 검사를 통과한 부피 입력은 true다.
 */
bool HasHullVolume(const std::vector<glm::vec3>& points, float coordinateScaleMeters = 0.0F);
}
