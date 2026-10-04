#pragma once

#include "Entity.h"

#include <memory>

class AssetManager;
class Scene;
class Shader;
struct ModelResource;

/**
 * @brief ModelResource를 실제 Flecs Entity hierarchy로 생성한다.
 *
 * @details
 * GltfLoader가 만든 CPU-side node/mesh 정보를 Scene의 Entity로 옮긴다.
 * 공간 계층(Node)과 렌더링 단위(SubMesh)를 분리하기 때문에 Joint처럼 Mesh가 없는
 * transform node도 보존할 수 있다.
 *
 * ModelResource -> PrefabFactory -> Flecs Entity hierarchy
 *
 * @todo [FUTURE] 동일 모델을 여러 번 생성할 때 이름 충돌을 피할 instance namespace/prefix를 지원한다.
 */
/*
 * [추가 그래픽스/ECS 용어 설명]
 * - Prefab: 같은 ModelResource에서 반복해서 Scene 객체를 만들 수 있게 하는 생성용 원본 개념.
 * - Factory: 복잡한 객체 생성 절차를 한 곳에 모아 호출자가 세부 과정을 몰라도 되게 하는 객체/함수.
 * - ECS(Entity Component System): Entity에 작은 데이터 Component를 붙이고 System이 해당 조합을 처리하는 구조.
 * - Flecs: 이 프로젝트에서 사용하는 ECS library.
 * - Entity Hierarchy: Parent/Child 관계로 연결된 Entity tree. GLB Node hierarchy를 그대로 표현하는 데 사용한다.
 * - Transform-only Node: Mesh 없이 위치/회전/부모자식 구조만 가진 Node. Robot Joint pivot에 중요하다.
 * - SubMesh: 하나의 GPU Mesh 안에서 특정 Material/Index 범위만 따로 그리는 하위 단위.
 *
 * PrefabFactory는 GLB를 다시 읽지 않는다. 이미 GltfLoader가 만든 ModelResource를 Scene/Flecs 표현으로 옮기는 역할만 한다.
 */
class PrefabFactory final
{
public:
    /**
     * @brief 모델 전체를 Scene에 생성하고 root Entity를 반환한다.
     * @param scene Entity를 생성할 Scene.
     * @param model GltfLoader가 만든 CPU-side 모델 데이터.
     * @param assets GPU Mesh/Material을 보관하는 AssetManager.
     * @param shader 생성되는 render entity에 사용할 Shader.
     * @return 생성된 모델 hierarchy의 root Entity.
     */
    static Entity CreateModel(
        Scene& scene,
        const ModelResource& model,
        const AssetManager& assets,
        const std::shared_ptr<Shader>& shader);

private:
    PrefabFactory() = delete;
};
