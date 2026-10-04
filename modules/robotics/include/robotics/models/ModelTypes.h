#pragma once

namespace grasplink::robotics::models
{

/**
 * @file ModelTypes.h
 * @brief Robot/Gripper 모델 사양에서 사용하는 최소 수학 타입.
 *
 * @details
 * 이 계층은 Viewer의 GLM 타입에 의존하지 않도록 단순한 double 3개만 사용한다.
 * 로보틱스에서 자주 나오는 용어는 다음처럼 읽으면 된다.
 *
 * - 좌표(position): 어떤 점이 기준점에서 X/Y/Z 방향으로 얼마나 떨어져 있는지 나타낸 값.
 * - 방향벡터(direction vector): 위치가 아니라 "어느 방향인가"만 나타내는 값.
 * - 회전축(axis): 물체가 회전할 때 중심이 되는 방향선. 보통 길이 1인 방향벡터로 표현한다.
 * - Local frame: 해당 Joint/Gripper 자체를 원점과 축의 기준으로 보는 좌표계.
 * - World/Base frame: 전체 장면 또는 로봇 base를 기준으로 보는 좌표계.
 */

/**
 * @brief 3차원 값 `(x,y,z)`를 저장하는 backend-neutral 실수 벡터.
 *
 * @details
 * 이 타입 자체는 단위를 정하지 않는다.
 * `bindPivotMeters`에 들어가면 각 값은 meter [m], `axis`에 들어가면 단위 없는 방향 성분이다.
 */
struct Vec3
{
    /** @brief X축 성분. 단위는 이 Vec3를 사용하는 필드 설명을 따른다. */
    double x = 0.0;
    /** @brief Y축 성분. 단위는 이 Vec3를 사용하는 필드 설명을 따른다. */
    double y = 0.0;
    /** @brief Z축 성분. 단위는 이 Vec3를 사용하는 필드 설명을 따른다. */
    double z = 0.0;
};

/**
 * @brief Joint가 어느 축을 중심으로 회전하는지 표현하는 Vec3 별칭.
 *
 * @details
 * 예를 들어 `{1,0,0}`은 local +X축, `{0,1,0}`은 local +Y축, `{0,0,-1}`은 local -Z축 회전을 뜻한다.
 * 축은 위치가 아니므로 meter 같은 단위가 없고, 회전 계산 시 길이 1인 방향으로 정규화해 사용한다.
 */
using Axis3 = Vec3;

} // namespace grasplink::robotics::models
