#pragma once



// 그리퍼의 개방 정도를 표현한다. 실제 단위와 범위는 그리퍼 제어 코드가 정한다.
struct Gripper
{
    float openness = 0.0F;
};

// 손가락의 기준 방향을 구분하기 위한 데이터다.
struct GripperFinger
{
    float direction = 1.0F;
};

// 이 태그가 붙은 엔티티는 그리퍼가 잡을 수 있는 대상으로 취급한다.
struct Grabbable
{
};
