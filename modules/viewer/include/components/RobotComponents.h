#pragma once

#include <glm/glm.hpp>



// 관절의 회전축과 허용 범위를 보관한다. 각도 단위는 라디안이다.
struct RobotJoint
{
    glm::vec3 axis{0.0F, 0.0F, 1.0F};
    float minAngle = 0.0F;
    float maxAngle = 0.0F;
    float angle = 0.0F;
};

// 말단장치를 식별하기 위한 태그다.
struct EndEffector
{
};
