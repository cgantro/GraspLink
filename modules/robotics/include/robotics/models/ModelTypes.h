#pragma once

namespace grasplink::robotics::models
{

/*
 * [추가 용어 설명]
 * - Vector(벡터): x/y/z처럼 여러 성분으로 크기 또는 방향을 표현하는 값.
 * - Axis(축): 회전의 기준이 되는 방향선. 보통 길이 1인 방향벡터로 표현한다.
 * - Local frame: 어떤 Joint/부품 자신을 기준으로 한 좌표계.
 * - World frame: Scene 전체가 공유하는 전역 좌표계.
 *
 * Vec3 자체는 단위를 강제하지 않는다.
 * bindPivotMeters에 쓰이면 [m], axis에 쓰이면 단위 없는 방향값처럼 사용 문맥이 단위를 결정한다.
 */

struct Vec3
{
    // X축 성분. 단위는 이 Vec3를 사용하는 필드의 의미를 따른다.
    double x = 0.0;
    // Y축 성분. 단위는 이 Vec3를 사용하는 필드의 의미를 따른다.
    double y = 0.0;
    // Z축 성분. 단위는 이 Vec3를 사용하는 필드의 의미를 따른다.
    double z = 0.0;
};

// Joint local frame의 회전축을 표현할 때 Vec3와 같은 저장형식을 사용한다.
// 예: {0,1,0}은 local +Y축, {0,0,-1}은 local -Z축 방향이다.
using Axis3 = Vec3;

} // namespace grasplink::robotics::models
