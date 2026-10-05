#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

// Local 위치 [m]. 부모 Entity의 좌표계 기준이며 GLB와 Robot pose도 같은 meter 기준을 쓴다.
struct Position : public glm::vec3
{
    using glm::vec3::vec3;

    Position() : glm::vec3(0.0F) {}
    Position(const glm::vec3& value) : glm::vec3(value) {}
};

// 부모 기준 Euler X/Y/Z 각 [rad]. TransformSystem은 이를 quaternion으로 바꿔 행렬을 만든다.
// FK/물리에서 계산한 quaternion도 저장 시 Euler로 변환한다. 특정 자세에서는 각 표현이 불연속일 수 있다.
struct Rotation : public glm::vec3
{
    using glm::vec3::vec3;

    Rotation() : glm::vec3(0.0F) {}
    Rotation(const glm::vec3& value) : glm::vec3(value) {}
};

// Local 축별 크기 배율. 단위 없음, (1,1,1)은 모델 크기 그대로.
struct Scale : public glm::vec3
{
    using glm::vec3::vec3;

    Scale() : glm::vec3(1.0F) {}
    explicit Scale(float value) : glm::vec3(value) {}
    Scale(const glm::vec3& value) : glm::vec3(value) {}
};

// TransformSystem이 계산해 둔 행렬. Local은 부모 기준, World는 전체 hierarchy의 누적 결과.
// Local TRS 변경 직후에는 이전 값이다. 물리와 렌더링이 읽기 전에 명시적으로 갱신한다.
struct TransformMatrix : public glm::mat4
{
    using glm::mat4::mat4;

    TransformMatrix() : glm::mat4(1.0F) {}
    TransformMatrix(const glm::mat4& value) : glm::mat4(value) {}
};

// (Component, Local): 부모 기준 값.
struct Local
{
};

// (TransformMatrix, World): 부모 hierarchy를 누적한 Scene 기준 행렬.
struct World
{
};

// 공간 변환과 별도로 Scene 정리 범위를 식별한다. 다른 Scene으로의 재부모화를 막는 기준.
struct SceneRootTag {};
