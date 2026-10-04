#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

/**
 * @brief Entity의 위치를 저장하는 vec3 기반 ECS Component.
 *
 * @details
 * `(Position, Local)` pair는 부모 Entity 기준 local translation이고,
 * `(Position, World)`는 현재 구조에서 직접 저장하지 않는다. World 위치는 TransformMatrix에서 읽는다.
 *
 * 단위는 asset/scene의 공간 단위를 따른다. controller-ready HCR-12A GLB와 robotics 계층은 meter [m]로 정규화되어 있다.
 */
struct Position : public glm::vec3
{
    using glm::vec3::vec3;

    /** @brief 원점 (0,0,0)으로 초기화한다. */
    Position() : glm::vec3(0.0F) {}

    /** @brief 기존 glm::vec3 값을 Position component로 복사한다. */
    Position(const glm::vec3& value) : glm::vec3(value) {}
};

/**
 * @brief Entity local 회전을 Euler XYZ angle로 저장하는 ECS Component.
 *
 * @details
 * 각 성분 단위는 radian [rad]이다. TransformSystemModule에서 `glm::quat(vec3)`로 quaternion을 만든 뒤
 * 4x4 rotation matrix로 변환한다. RobotTransformAdapter도 joint quaternion을 최종적으로 이 저장 형식에 맞춰
 * Euler radian으로 바꾼 뒤 SetLocalRotation()을 호출한다.
 *
 * @warning Euler 표현은 회전 합성 순서/특이점 문제가 있다. Joint state의 원본 수학 표현으로 사용하지 말고
 *          현재 Viewer transform 저장 형식으로만 취급한다.
 * @todo [FUTURE] FK/IK/복합 회전 정확도를 위해 quaternion component로 전환하고 중간 Euler 변환을 제거한다.
 */
struct Rotation : public glm::vec3
{
    using glm::vec3::vec3;

    /** @brief identity rotation에 해당하는 (0,0,0) rad로 초기화한다. */
    Rotation() : glm::vec3(0.0F) {}

    /** @brief Euler radian vec3를 Rotation component로 복사한다. */
    Rotation(const glm::vec3& value) : glm::vec3(value) {}
};

/**
 * @brief Entity의 부모 기준 local scale을 저장하는 vec3 기반 ECS Component.
 *
 * @note Scale은 무차원 배율이다. (1,1,1)이 원본 크기이며 position의 meter 단위와 혼동하지 않는다.
 */
struct Scale : public glm::vec3
{
    using glm::vec3::vec3;

    /** @brief 원본 크기 (1,1,1)로 초기화한다. */
    Scale() : glm::vec3(1.0F) {}

    /** @brief 세 축에 같은 배율을 적용한다. */
    explicit Scale(float value) : glm::vec3(value) {}

    /** @brief 축별 scale 값을 복사한다. */
    Scale(const glm::vec3& value) : glm::vec3(value) {}
};

/**
 * @brief Local 또는 World 변환 결과를 저장하는 4x4 homogeneous transform matrix.
 *
 * @details
 * `(TransformMatrix, Local)`은 `Translation * Rotation * Scale`,
 * `(TransformMatrix, World)`은 `ParentWorld * Local` 결과다.
 * column-major storage는 GLM/OpenGL convention을 따른다.
 */
struct TransformMatrix : public glm::mat4
{
    using glm::mat4::mat4;

    /** @brief Identity transform으로 초기화한다. */
    TransformMatrix() : glm::mat4(1.0F) {}

    /** @brief 기존 glm::mat4를 복사한다. */
    TransformMatrix(const glm::mat4& value) : glm::mat4(value) {}
};

/**
 * @brief Flecs pair의 값이 부모 Entity 기준 Local space임을 표시하는 tag.
 *
 * 예: `(Position, Local)`, `(Rotation, Local)`, `(TransformMatrix, Local)`.
 */
struct Local
{
};

/**
 * @brief Flecs pair의 값이 Scene hierarchy 기준 World space임을 표시하는 tag.
 *
 * 현재 주 사용 예는 `(TransformMatrix, World)`다.
 */
struct World
{
};
