#pragma once

#include "assets/GraphicsTypes.h"
#include "PhysicsTypes.h"
#include "robotics/models/RobotSpecification.h"

#include <cstddef>
#include <vector>

namespace grasplink::simulation
{

/**
 * @brief GLB 로봇 메시를 물리 충돌에 사용할 Base와 Link별 볼록 형상으로 변환한다.
 * @details 화면 메시 전체를 그대로 충돌 검사에 쓰면 세부 삼각형이 많아 계산이 무거워진다.
 * Builder는 연결된 메시 부품을 16 cm 셀로 나누고 각 셀을 ConvexHull로 근사한다.
 * 반환된 Base 좌표는 RobotRoot 기준이고 Link 좌표는 해당 Link 관절 원점 기준이다.
 * Scene Entity 생성과 FK 자세 적용은 이 타입의 책임이 아니다.
 */
struct RobotCollisionGeometryBuilder final
{
    /**
     * @brief 충돌 외피를 만들 때 형상을 단순화하는 기준을 지정한다.
     * @details Vertex merge tolerance는 GLB seam에서 같은 위치로 저장된 정점을 합치는 거리 [m]다.
     * Cell size는 한 충돌 형상으로 묶을 삼각형 중심의 셀 한 변 [m]이다.
     * Component와 cell minimum extent는 각각 연결 부품과 셀 내부의 최소 폭 [m]이며, 이보다 작으면 형상을 생략한다.
     */
    struct Options
    {
        double vertexMergeToleranceMeters = 0.00001;
        float cellSizeMeters = 0.16F;
        float componentMinimumExtentMeters = 0.04F;
        float cellMinimumExtentMeters = 0.01F;
    };

    struct LinkGeometry
    {
        std::size_t jointIndex = 0;
        std::vector<grasplink::physics::CollisionShapeDescription> shapes;
    };

    struct Result
    {
        std::vector<grasplink::physics::CollisionShapeDescription> baseShapes;
        std::vector<LinkGeometry> links;
    };

    /**
     * @brief GLB 계층과 로봇 명세에서 충돌 형상을 계산한다.
     * @param specification 각 Link와 이를 움직이는 joint 번호를 제공하는 로봇 명세.
     * @param model 형상 정점과 노드 계층을 보관하는 GLB 자원.
     * @param options 정점 병합과 형상 생략 기준이며 길이 단위는 m다.
     * @return Base 형상과 명세 순서에 맞춘 Link 형상 및 관절 번호를 반환한다.
     * @throws std::invalid_argument 명세가 비었거나 GLB 노드가 중복·순환하거나 Link 형상을 만들 수 없을 때.
     * @throws std::runtime_error GLB 노드 관계, 삼각형 범위 또는 정점 번호가 잘못되었을 때.
     */
    [[nodiscard]] static Result Build(
        const grasplink::robotics::models::RobotSpecification& specification,
        const ModelResource& model,
        const Options& options = {});
};

}
