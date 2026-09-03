#pragma once

#include <flecs.h>

#include <type_traits>
#include <utility>

namespace PoseLink
{
/*
    flecs::entity를 사용하기위한 래퍼 클래스
*/
class Entity{
public:
    static Entity Create(flecs::world& world, const char* name = nullptr){
        if(name) return Entity(world.entity(name));
    }

    /*
        Component를 Entity에 붙인다
    */
    template<typename T>
    Entity& Set(T&& component){
        // const, volatile, 참조등을 제거한 순수 데이터 타입 추출
        using Component = std::decay_t<T>;

        // 여기서는 T&&로 인수를 받음 -> lval로 ㅂ다음
        m_Handle.set<Component>(
            std::forward<T>(component)  //원래 값 성질(lval, rval) 보존 후 전달
        );

        // 1. 메서드 체이닝
        // 2. 동일 객체 수정 유지
        // 3. 복사 제거
        return *this;
    }

    // 특정 컴포넌트를 가졌는가
    template<typename T>
    bool Has() const{
        return m_Handle.has<T>();
    }

    // 컴포넌트 제거
    template<typename T>
    Entity& Remove(){
        m_Handle.remove<T>();
        return *this;
    }

    void Destroy(){
        if(m_Handle.is_alive()) m_Handle.destruct();
    }

    bool IsAlive(){
        return m_Handle.is_alive();
    }


private:
    explicit Entity(flecs::entity handle):m_Handle(handle){}
private:
    flecs::entity m_Handle;
};
} // namespace PoseLink
