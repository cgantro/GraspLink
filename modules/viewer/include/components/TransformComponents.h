#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

/**
 * @brief 부모 좌표계에서 Entity 원점의 Local 위치를 [m]로 저장한다.
 * @details GLB 배치와 로봇 pose도 같은 meter 기준을 사용한다. `Position, Local`은 입력 상태이며,
 * TransformSystem이 이를 행렬로 옮기고 부모의 World 행렬과 합쳐 Scene 기준 위치를 만든다.
 */
struct Position : public glm::vec3
{
    using glm::vec3::vec3;

    Position() : glm::vec3(0.0F) {}
    Position(const glm::vec3& value) : glm::vec3(value) {}
};

/**
 * @brief 부모 좌표계 기준 Euler X/Y/Z 회전을 [rad]로 저장한다.
 * @details TransformSystem은 GLM의 Euler 입력으로 quaternion을 만들고 회전 행렬에 반영한다.
 * FK 결과는 계산 중 quaternion으로 유지되지만 Entity에 적용할 때 이 성분으로 변환된다.
 * Euler 표기는 같은 방향도 여러 값으로 표현할 수 있고 특정 자세에서 값이 불연속일 수 있으므로,
 * 이 변환의 정밀도·연속성 문제는 World 행렬 캐시를 프레임마다 갱신하는 것과 별개의 표현 경계다.
 */
struct Rotation : public glm::vec3
{
    using glm::vec3::vec3;

    Rotation() : glm::vec3(0.0F) {}
    Rotation(const glm::vec3& value) : glm::vec3(value) {}
};

/**
 * @brief 부모 좌표계 축별 Local 크기 배율을 저장한다.
 * @details 단위가 없으며 `(1, 1, 1)`은 원래 크기다. TransformSystem은 비균일 배율도 행렬에 반영한다.
 * 물리 모듈은 collider 치수에 Entity 계층의 scale을 전파하지 않으므로 별도 계약으로 단위 scale을
 * 요구한다. Static Environment 본인의 시각 scale만 예외이며 collider의 지정 치수는 그대로다.
 */
struct Scale : public glm::vec3
{
    using glm::vec3::vec3;

    Scale() : glm::vec3(1.0F) {}
    explicit Scale(float value) : glm::vec3(value) {}
    Scale(const glm::vec3& value) : glm::vec3(value) {}
};

/**
 * @brief TransformSystem이 계산한 Local 또는 Scene 기준 변환 행렬을 보관한다.
 * @details `TransformMatrix, Local`은 Entity 자체의 TRS 행렬이고 `TransformMatrix, World`는
 * 조상부터 현재 Entity까지 누적한 결과다. 둘 다 파생값이므로 Position/Rotation/Scale을 바꿔도
 * 즉시 바뀌지 않는다. 물리와 렌더링이 최신 pose를 읽기 전에 `UpdateWorldTransforms`가 명시적으로 갱신한다.
 */
struct TransformMatrix : public glm::mat4
{
    using glm::mat4::mat4;

    TransformMatrix() : glm::mat4(1.0F) {}
    TransformMatrix(const glm::mat4& value) : glm::mat4(value) {}
};

/** @brief TransformComponent의 부모 좌표계 기준 값을 구분하는 Flecs pair 표식. */
struct Local
{
};

/** @brief TransformMatrix의 계층 누적 Scene 좌표계 값을 구분하는 Flecs pair 표식. */
struct World
{
};

/**
 * @brief 공간 변환과 무관하게 Entity가 속한 Scene의 소유 경계를 표시한다.
 * @details Entity의 재부모화 검사는 이 표식으로 Scene 간 이동을 막는다. TransformSystem은 이 태그를
 * 소유권 판단에 사용하지 않으며, SceneRoot도 일반 grouping 부모처럼 자신의 TRS 없이 자식 World 변환을 전달한다.
 */
struct SceneRootTag {};
