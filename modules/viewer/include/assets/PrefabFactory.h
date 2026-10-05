#pragma once

#include "Entity.h"

#include <memory>

class AssetManager;
class Scene;
class Shader;
struct ModelResource;

/**
 * @brief CPU 모델 데이터를 Scene Entity 계층으로 옮김
 *
 * 입력: GltfLoader가 만든 ModelResource
 * 처리: Node → Entity, SubMesh → Render Component
 * Mesh 없는 Node: Joint pivot 보존을 위해 Entity로 유지
 */
class PrefabFactory final
{
public:
    /**
     * @brief 모델 Node 계층과 Render Component 생성
     * @param scene Entity 생성 대상 Scene
     * @param model GltfLoader가 만든 CPU 모델 데이터
     * @param assets GPU Mesh·Material 보관 객체
     * @param shader 생성 Entity가 사용할 Shader
     * @return 모델 root Entity
     */
    static Entity CreateModel(
        Scene& scene,
        const ModelResource& model,
        const AssetManager& assets,
        const std::shared_ptr<Shader>& shader);

private:
    PrefabFactory() = delete;
};
