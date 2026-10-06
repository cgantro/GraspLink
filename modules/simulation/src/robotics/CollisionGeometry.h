#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace grasplink::simulation::detail
{
/**
 * @brief 여러 방향에서 가장 바깥쪽에 있는 정점만 골라 Jolt 볼록 충돌 외피의 계산량을 줄인다.
 * @param vertices Body 원점 좌표계에서 측정한 표면 정점 [m].
 * @return 검사 방향에서 가장 바깥쪽인 정점 목록이다. 이 점으로 오목한 원본 메시 대신 볼록 외피를 만들기 때문에 메시 내부의 빈 공간이 채워질 수 있다.
 */
std::vector<glm::vec3> BuildConvexSupportPoints(const std::vector<glm::vec3>& vertices);

/**
 * @brief 정점들이 선이나 평면에만 놓이지 않고 실제 3차원 충돌 부피를 만들 수 있는지 확인한다.
 * @param points 대표 방향별로 선택한 충돌 정점 [m].
 * @param coordinateScaleMeters 양수이면 해당 미터 값을 기준으로 좌표를 정규화한다. 0이면 점 집합을 둘러싼 상자의 대각 길이를 사용해 크기 차이를 맞춘다.
 * Gripper는 기본값 0을 써 작은 부품도 부품 크기에 비례해 검사한다. Arm은 1 m를 전달해 기존의 절대 허용 오차를 유지한다.
 * @return 점·선·평면처럼 부피가 없는 입력은 false, 허용 오차 밖으로 평면에서 벗어난 점이 있어 부피가 확인되면 true다.
 */
bool HasHullVolume(const std::vector<glm::vec3>& points, float coordinateScaleMeters = 0.0F);
}
