#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>



// 메시의 위치, 회전, 크기를 보관한다. GLM 쿼터니언은 w, x, y, z 순서다.
struct Transform
{
    glm::vec3 position{0.0F};
    glm::quat rotation{1.0F, 0.0F, 0.0F, 0.0F};
    glm::vec3 scale{1.0F};

    glm::mat4 GetMatrix() const
    {
        return glm::translate(glm::mat4(1.0F), position)
            * glm::mat4_cast(glm::normalize(rotation))
            * glm::scale(glm::mat4(1.0F), scale);
    }
};

// ECS에서 위치, 회전, 크기를 별도 컴포넌트로 저장하기 위한 얇은 타입이다.
struct Position : glm::vec3
{
    using glm::vec3::vec3;
    Position() : glm::vec3(0.0F) {}
    Position(const glm::vec3& value) : glm::vec3(value) {}
};

struct Rotation : glm::vec3
{
    using glm::vec3::vec3;
    Rotation() : glm::vec3(0.0F) {}
    Rotation(const glm::vec3& value) : glm::vec3(value) {}
};

struct Scale : glm::vec3
{
    using glm::vec3::vec3;
    Scale() : glm::vec3(1.0F) {}
    explicit Scale(float value) : glm::vec3(value) {}
    Scale(const glm::vec3& value) : glm::vec3(value) {}
};

struct TransformMatrix : glm::mat4
{
    using glm::mat4::mat4;
    TransformMatrix() : glm::mat4(1.0F) {}
    TransformMatrix(const glm::mat4& value) : glm::mat4(value) {}
};

// Local은 부모 기준 값, World는 씬 전체 기준 값을 표시하는 ECS 태그다.
struct Local
{
};

struct World
{
};
