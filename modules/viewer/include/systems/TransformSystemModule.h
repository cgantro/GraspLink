#pragma once

#include <flecs.h>



// 변환 컴포넌트에서 로컬 행렬을 계산하는 시스템 모듈이다.
class TransformSystemModule
{
public:
    explicit TransformSystemModule(flecs::world& world);

private:
    void RegisterObserver(flecs::world& world);
    void RegisterSystem(flecs::world& world);
};
