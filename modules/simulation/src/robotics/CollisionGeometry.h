#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace grasplink::simulation::detail
{
// 입력: Body 기준 정점 [m]. 구면 방향별 극점으로 볼록 외피 입력의 크기를 제한한다.
std::vector<glm::vec3> BuildConvexSupportPoints(const std::vector<glm::vec3>& vertices);

// scale=0: 크기로 정규화해 얇은 손가락도 판정. scale=1: 기존 로봇 링크의 m 기준 허용오차 유지.
// 점·선·평면은 Jolt 입체 hull에서 제외한다.
bool HasHullVolume(const std::vector<glm::vec3>& points, float coordinateScaleMeters = 0.0F);
}
