#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>
#include <stdexcept>

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
 * @brief 부모 좌표계 기준 Local 자세를 단위 quaternion으로 저장한다.
 * @details 성분은 무차원이며 GLM 생성자 순서는 (w,x,y,z), 항등은 (1,0,0,0)이다.
 * quaternion은 회전축과 회전량을 함께 나타내므로 Euler 각의 특이 자세에서 축이 겹치는 문제를 피한다.
 * q와 -q는 같은 방향이다. FK·GLB·Physics의 자세를 각도로 왕복 변환하지 않고 행렬 합성까지 전달한다.
 * 생성 시 정규화하며 영벡터·비유한 값은 거부한다. ECS에서 성분을 직접 수정한 경우에도
 * TransformSystem이 행렬 합성 전에 다시 검증한다.
 */
struct Rotation : public glm::quat
{
    Rotation() : glm::quat(1.0F, 0.0F, 0.0F, 0.0F) {}

    /** @brief 유한한 영벡터 아닌 quaternion을 정규화한다. 잘못된 입력은 invalid_argument로 거부한다. */
    Rotation(const glm::quat& value) : glm::quat(Normalized(value)) {}

private:
    static glm::quat Normalized(const glm::quat& value)
    {
        // float 제곱합의 overflow·underflow를 피한다. 작은 유한 quaternion도 방향을 보존한다.
        const double length = std::hypot(std::hypot(static_cast<double>(value.w), value.x),
            std::hypot(static_cast<double>(value.y), value.z));
        if (!std::isfinite(value.w) || !std::isfinite(value.x) || !std::isfinite(value.y) ||
            !std::isfinite(value.z) || !std::isfinite(length) || length <= 0.0)
            throw std::invalid_argument("Rotation requires a finite, nonzero quaternion");
        return glm::quat{static_cast<float>(value.w / length), static_cast<float>(value.x / length),
            static_cast<float>(value.y / length), static_cast<float>(value.z / length)};
    }
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
