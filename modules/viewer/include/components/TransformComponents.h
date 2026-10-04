#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

/** @brief Entity 위치를 저장하는 vec3 기반 ECS Component. */
/*
 * [추가 그래픽스 용어 설명]
 * Position은 3차원 위치 (x,y,z)를 저장한다.
 * Local로 붙으면 부모 Entity 기준 위치이고, World는 전체 Scene 기준 위치/행렬을 뜻한다.
 * 현재 HCR controller-ready asset이 meter 기준이므로 해당 robot hierarchy의 Position은 [m]로 해석한다.
 */
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
/*
 * [추가 그래픽스 용어 설명]
 * - Euler Angle: X/Y/Z 축 회전량 세 개로 방향을 표현하는 방식. 여기서는 [rad].
 * - Quaternion: 회전을 네 성분으로 표현해 회전 합성/보간에서 Euler의 일부 문제를 줄이는 방식.
 * - Gimbal Lock/특이점: 특정 Euler 자세에서 두 회전축이 겹쳐 자유도가 줄어드는 현상.
 *
 * 현재 저장 형식이 Euler vec3이므로 RobotTransformAdapter도 quaternion 계산 후 다시 Euler로 변환해 저장한다.
 */
struct Rotation : public glm::vec3
{
    using glm::vec3::vec3;

    Rotation() : glm::vec3(0.0F) {}
    Rotation(const glm::vec3& value) : glm::vec3(value) {}
};

/** @brief Entity의 local scale을 저장하는 vec3 기반 ECS Component. */
/* Scale은 각 축의 크기 배율이며 물리 단위가 없다. (1,1,1)은 원래 크기 그대로라는 뜻이다. */
struct Scale : public glm::vec3
{
    using glm::vec3::vec3;

    Scale() : glm::vec3(1.0F) {}
    explicit Scale(float value) : glm::vec3(value) {}
    Scale(const glm::vec3& value) : glm::vec3(value) {}
};

/** @brief Local 또는 World 변환 결과를 저장하는 4x4 homogeneous transform matrix. */
/*
 * [추가 그래픽스 용어 설명]
 * Homogeneous 4x4 Matrix는 3D Position/Rotation/Scale과 Translation을 하나의 행렬 곱으로 다루기 위한 표현이다.
 * TransformSystem에서 Local = T * R * S, World = ParentWorld * Local 순서로 계산한다.
 */
struct TransformMatrix : public glm::mat4
{
    using glm::mat4::mat4;

    TransformMatrix() : glm::mat4(1.0F) {}
    TransformMatrix(const glm::mat4& value) : glm::mat4(value) {}
};

/** @brief Flecs pair에서 부모 기준 Local 공간임을 나타내는 tag. */
/* Local tag 자체는 값을 저장하지 않고 같은 Position/Rotation/Matrix 타입이 어느 공간 기준인지 구분한다. */
struct Local
{
};

/** @brief Flecs pair에서 Scene 전체 기준 World 공간임을 나타내는 tag. */
/* World transform은 부모 hierarchy를 모두 누적한 최종 결과다. */
struct World
{
};
