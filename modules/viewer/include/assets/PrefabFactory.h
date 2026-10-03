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
