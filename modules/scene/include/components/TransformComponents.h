#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>
#include <stdexcept>

/**
 * @brief 바로 위 부모를 기준으로 장면 물체 원점의 위치를 미터 단위로 저장한다.
 * @details Local은 부모 기준 값이고 World는 Scene 전체 기준 값이다. 예를 들어 로봇 팔 링크의 Local 위치는 부모 관절에서 떨어진 거리다.
 * TransformSystemModule은 이 위치와 부모의 World 행렬을 합쳐 링크가 Scene에서 놓일 위치를 계산한다.
 */
struct Position : public glm::vec3
{
    using glm::vec3::vec3;

    Position() : glm::vec3(0.0F) {}
    Position(const glm::vec3& value) : glm::vec3(value) {}
};

/**
 * @brief 바로 위 부모를 기준으로 이 물체의 방향을 quaternion으로 저장한다.
 * @details Quaternion은 네 수 (w,x,y,z)로 방향을 나타내며 GLM 생성자도 이 순서를 쓴다. 방향을 바꾸지 않는 값은 (1,0,0,0)이다.
 * 관절 각도에서 FK가 계산한 로봇 링크의 방향을 GLB 화면 모델과 물리 계산에 그대로 전달할 수 있다. q와 -q는 부호만 다르고 같은 방향이다.
 * 생성 시 길이 1로 맞추며 모든 성분이 0이거나 유한하지 않은 값은 거부한다. ECS 저장소에서 직접 수정해도 TransformSystemModule이 행렬 계산 전에 다시 검사한다.
 */
struct Rotation : public glm::quat
{
    Rotation() : glm::quat(1.0F, 0.0F, 0.0F, 0.0F) {}

    /**
     * @brief 유효한 회전 방향을 길이 1로 맞춘 뒤 저장한다.
     * @throws std::invalid_argument 성분이 유한하지 않거나 모든 성분이 0인 회전을 전달한 경우.
     */
    Rotation(const glm::quat& value) : glm::quat(Normalized(value)) {}

private:
    static glm::quat Normalized(const glm::quat& value)
    {
        // float 성분을 먼저 제곱하면 큰 입력은 overflow하고 작은 입력은 underflow할 수 있다. double 기반 hypot으로 길이를 구해 작은 유한 회전도 방향을 보존한다.
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
 * @brief 부모를 기준으로 물체의 각 축을 몇 배로 그릴지 저장한다.
 * @details 배율에는 단위가 없고 `(1,1,1)`은 크기를 바꾸지 않는다. TransformSystemModule은 화면 모델 크기에 이 값을 반영하지만 Physics Body의 Collider 치수에는 곱하지 않는다.
 * 따라서 물리 물체는 기본적으로 배율을 `(1,1,1)`로 둔다. 바닥처럼 움직이지 않는 Environment는 화면 Mesh만 크게 하고 충돌 크기는 별도로 미터 단위로 지정할 수 있다.
 */
struct Scale : public glm::vec3
{
    using glm::vec3::vec3;

    Scale() : glm::vec3(1.0F) {}
    explicit Scale(float value) : glm::vec3(value) {}
    Scale(const glm::vec3& value) : glm::vec3(value) {}
};

/**
 * @brief 물체의 위치·회전·크기를 행렬로 합친 결과를 보관한다.
 * @details TRS는 이 세 입력값의 머리글자다. `TransformMatrix, Local`은 현재 물체의 값만 나타내고 `TransformMatrix, World`는 부모와 조상 값까지 더한 Scene 전체 위치와 방향을 나타낸다.
 * 행렬은 입력값에서 계산한 결과라 Position, Rotation, Scale을 바꿔도 즉시 갱신되지 않는다. 물리나 화면이 새 자세를 읽기 전에 `UpdateWorldTransforms`를 실행해야 한다.
 */
struct TransformMatrix : public glm::mat4
{
    using glm::mat4::mat4;

    TransformMatrix() : glm::mat4(1.0F) {}
    TransformMatrix(const glm::mat4& value) : glm::mat4(value) {}
};

/**
 * @brief 함께 저장한 변환 값이 바로 위 부모 기준임을 표시한다.
 * @details Flecs가 Position이나 Rotation 옆에 붙여 저장하는 빈 표식이다. Position과 Local은 Scene 전체가 아니라 부모 관절 기준 위치를 뜻한다.
 */
struct Local
{
};

/**
 * @brief 변환 행렬이 Scene 전체 좌표 기준임을 표시한다.
 * @details TransformMatrix와 함께 저장해 물체 자체의 Local 행렬과 부모·조상까지 누적된 World 행렬을 구분한다.
 */
struct World
{
};

/**
 * @brief 장면 물체가 어느 Scene에 속하고 함께 정리되는지 표시한다.
 * @details 부모를 바꿀 때 이 표식이 붙은 조상을 찾아 Scene 물체가 다른 Scene으로 옮겨지는 것을 막는다. 위치·회전·크기를 나타내는 값이 아니며 변환 계산의 입력으로 쓰이지 않는다.
 * SceneRoot에는 자체 위치·회전·크기가 없지만 일반 부모처럼 조상의 World 행렬을 자식에게 전달한다.
 */
struct SceneRootTag {};
