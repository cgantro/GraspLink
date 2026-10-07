#pragma once

#include "assets/GraphicsTypes.h"
#include "PhysicsTypes.h"
#include "robotics/models/RobotSpecification.h"

#include <cstddef>
#include <vector>

namespace grasplink::simulation::robot_collision_geometry
{

/**
 * @brief GLB 꼭짓점을 합치고 충돌 근사 도형을 만들 때 쓰는 오차와 크기 기준이다.
 * @details 큰 충돌 셀은 계산량을 줄이고, 작은 연결 부품은 별도 ConvexHull로 남겨 얇은 구조가 사라지지 않게 한다.
 */
struct Options
{
    /** @brief 이 거리 안의 GLB 꼭짓점을 같은 점으로 취급한다. 단위는 m이다. */
    double vertexMergeToleranceMeters = 0.00001;
    /** @brief 큰 충돌 영역을 나눌 때 쓰는 셀 크기다. 단위는 m이다. */
    float cellSizeMeters = 0.16F;
    /** @brief 이 길이보다 짧은 연결 부품은 별도 도형으로 만들지 않는다. 단위는 m이다. */
    float componentMinimumExtentMeters = 0.04F;
    /** @brief 이 크기보다 작은 셀은 충돌 도형으로 만들지 않는다. 단위는 m이다. */
    float cellMinimumExtentMeters = 0.01F;
};

/** @brief 한 관절에 연결된 링크와 해당 링크의 충돌 도형이다. */
struct LinkGeometry
{
    std::size_t jointIndex = 0;
    std::vector<grasplink::physics::CollisionShapeDescription> shapes;
};

/**
 * @brief 로봇 베이스와 링크별 충돌 도형을 담는다.
 * @details 베이스 위치는 RobotRoot 기준이며 링크 도형은 각 링크 관절 기준으로 반환한다.
 */
struct Result
{
    std::vector<grasplink::physics::CollisionShapeDescription> baseShapes;
    std::vector<LinkGeometry> links;
};

/**
 * @brief GLB의 메시와 로봇 관절 명세에서 Jolt용 충돌 도형을 만든다.
 * @throws std::invalid_argument 옵션, 관절 명세, GLB 노드 또는 링크 충돌 메시가 올바르지 않은 경우
 */
[[nodiscard]] Result Build(
    const grasplink::robotics::models::RobotSpecification& specification,
    const ModelResource& model,
    const Options& options = {});

}
