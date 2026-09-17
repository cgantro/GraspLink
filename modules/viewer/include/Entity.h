#pragma once

#include <flecs.h>
#include <glm/glm.hpp>

#include <string>
#include <vector>

class Entity{
public:
    Entity() = default;

    // 이미 존재하는 Flecs Entity Handle을 감싼다.
    explicit Entity(flecs::entity handle);

    Entity(const Entity&) = default;
    Entity& operator=(const Entity&) = default;

    // Transform
    /*
        World 값을 직접 수정하는 API는 제공하지 않는다.
        World는 System이 ParentWorld * Local을 통해 계산해야함
    */
    glm::vec3 GetLocalPosition() const;
    glm::vec3 GetLocalRotation() const;
    glm::vec3 GetLocalScale() const;

    void SetLocalPosition(const glm::vec3& position);
    void SetLocalRotation(const glm::vec3& rotation);
    void SetLocalScale(const glm::vec3& scale);

    // System이 계산한 최종 WorldMatrix 조회
    glm::mat4 GetWorldMatrix() const;
    
    // 계층구조
    /*
        Flecs ChildOf 관계 형성
    */
public:
    Entity& SetParent(const Entity& parent);
    Entity& AddChild(const Entity& child);

    Entity GetParent() const;
    std::vector<Entity> GetChildren() const;
    Entity GetChild(const std::string& name) const; // 이름으로 검색

    // 하위 계층 구조를 DFS로 검색
    Entity FindChildByNameRecursive(const std::string& targetName) const;
public:
    // Generic Component API
    template<typename T>
    Entity& Set(const T& component){
        m_EntityHandle.set<T>(component);
        return *this;
    }

    // Tag Component 추가
    template<typename T>
    Entity& Add(){
        m_EntityHandle.add<T>();
        return *this;
    }

    // Component 제거.
    template<typename T>
    Entity& Remove() {
        m_EntityHandle.remove<T>();
        return *this;
    }

    // Mutable Component 접근
    //  get_mut()을 사용하므로 수정 가능한 참조를 얻는다.
    template<typename T>
    T& Get(){
        return m_EntityHandle.get_mut<T>();
    }

    template<typename T>
    bool Has() const{
        return m_EntityHandle.has<T>();
    }

public:
    // Util
    flecs::entity GetHandle() const{
        return m_EntityHandle;
    }
    bool IsValid() const;
    void Destroy();

    // 필요한 경우 Wrapper를 flecs entity로 전달
    operator flecs::entity() const{
        return m_EntityHandle;
    }

    // if (entity) 형태 지원.
    explicit operator bool() const{
        return IsValid();
    }

      bool operator==(const Entity& other) const{
        return m_EntityHandle == other.m_EntityHandle;
    }

    bool operator!=(const Entity& other) const{
        return !(*this == other);
    }

private:
    flecs::entity m_EntityHandle{flecs::entity::null()};
};