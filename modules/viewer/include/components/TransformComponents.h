#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

/** @brief Entity 위치를 저장하는 vec3 기반 ECS Component. */
struct Position : public glm::vec3
{
    using glm::vec3::vec3;

    Position() : glm::vec3(0.0F) {}
    Position(const glm::vec3& value) : glm::vec3(value) {}
};

/**
 * @brief Entity 회전을 Euler radian XYZ로 저장하는 ECS Component.
 * @warning Euler 표현은 회전 합성에서 순서 문제와 특이점이 생길 수 있다.
 * @todo [FUTURE] FK/IK와 Joint 회전의 정확도가 중요해지는 시점에 quaternion 저장 방식으로 전환을 검토한다.
 */
struct Rotation : public glm::vec3
{
    using glm::vec3::vec3;

    Rotation() : glm::vec3(0.0F) {}
    Rotation(const glm::vec3& value) : glm::vec3(value) {}
};

/** @brief Entity의 local scale을 저장하는 vec3 기반 ECS Component. */
struct Scale : public glm::vec3
{
    using glm::vec3::vec3;

    Scale() : glm::vec3(1.0F) {}
    explicit Scale(float value) : glm::vec3(value) {}
    Scale(const glm::vec3& value) : glm::vec3(value) {}
};

/** @brief Local 또는 World 변환 결과를 저장하는 4x4 homogeneous transform matrix. */
struct TransformMatrix : public glm::mat4
{
    using glm::mat4::mat4;

    TransformMatrix() : glm::mat4(1.0F) {}
    TransformMatrix(const glm::mat4& value) : glm::mat4(value) {}
};

/** @brief Flecs pair에서 부모 기준 Local 공간임을 나타내는 tag. */
struct Local
{
};

/** @brief Flecs pair에서 Scene 전체 기준 World 공간임을 나타내는 tag. */
struct World
{
};
