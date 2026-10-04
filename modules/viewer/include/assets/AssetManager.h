#pragma once

#include "assets/GraphicsTypes.h"

#include <memory>
#include <unordered_map>

class Material;
class Mesh;
class Texture;

/**
 * @brief CPU-side ModelResource를 GPU Mesh/Material/Texture로 변환하고 ResourceID로 캐시한다.
 *
 * @details
 * 데이터 흐름:
 *
 * `GLB -> GltfLoader -> ModelResource -> AssetManager -> GPU resources`
 *
 * 책임 경계:
 * - GltfLoader: 파일 해석, CPU-side IR 생성
 * - AssetManager: OpenGL Mesh/Texture/Material 생성 및 lifetime/cache 관리
 * - PrefabFactory: ModelResource Node를 Flecs Entity hierarchy로 변환
 *
 * ResourceID를 key로 사용하므로 같은 resource가 중복 upload되는 경우 기존 GPU object를 재사용할 수 있다.
 * Mesh/Material/Texture의 실제 ownership은 shared_ptr cache가 가진다.
 *
 * 단위 변환은 수행하지 않는다. Vertex position 등 공간 값은 ModelResource가 가진 값을 그대로 GPU에 업로드한다.
 * 현재 controller-ready HCR asset의 vertex/translation은 meter [m]다.
 *
 * @todo [FUTURE] metallic-roughness, normal, occlusion, emissive texture까지 Material에 연결한다.
 */
class AssetManager final
{
public:
    /** @brief Material 미지정 Primitive에 사용할 fallback GPU Material을 생성한다. */
    AssetManager();

    /**
     * @brief ModelResource의 texture/material/mesh를 GPU resource로 업로드하고 cache에 등록한다.
     * @param model 업로드할 CPU-side model. 각 MeshData::gpuMesh도 생성된 shared_ptr로 채워진다.
     *
     * @note 함수는 geometry 위치 단위를 변경하지 않는다. CPU Vertex 값을 그대로 VBO로 전달한다.
     */
    void UploadModel(ModelResource& model);

    /**
     * @brief ID에 해당하는 GPU Mesh를 조회한다.
     * @param id GltfLoader/ModelResource가 생성한 ResourceID.
     * @return cache에 있으면 shared_ptr<Mesh>, 없으면 nullptr.
     */
    std::shared_ptr<Mesh> GetMesh(ResourceID id) const;

    /**
     * @brief ID에 해당하는 runtime Material을 조회한다.
     * @param id Material ResourceID.
     * @return cache에 있으면 shared_ptr<Material>, 없으면 nullptr.
     */
    std::shared_ptr<Material> GetMaterial(ResourceID id) const;

    /**
     * @brief ID에 해당하는 GPU Texture를 조회한다.
     * @param id Texture ResourceID.
     * @return cache에 있으면 shared_ptr<Texture>, 없으면 nullptr.
     */
    std::shared_ptr<Texture> GetTexture(ResourceID id) const;

    /** @return Material이 없는 Primitive에 사용할 fallback Material shared_ptr. */
    std::shared_ptr<Material> GetDefaultMaterial() const;

    /**
     * @brief Manager가 보관한 GPU resource shared_ptr를 모두 해제한다.
     * @note 다른 객체가 같은 resource를 shared_ptr로 보유 중이면 실제 GPU object lifetime은 그 참조가 사라질 때까지 유지된다.
     */
    void Clear();

private:
    /** @brief ResourceID -> GPU Mesh cache. */
    std::unordered_map<ResourceID, std::shared_ptr<Mesh>> meshes_;

    /** @brief ResourceID -> runtime Material cache. */
    std::unordered_map<ResourceID, std::shared_ptr<Material>> materials_;

    /** @brief ResourceID -> GPU Texture cache. */
    std::unordered_map<ResourceID, std::shared_ptr<Texture>> textures_;

    /** @brief Material 미지정 Primitive용 fallback Material. */
    std::shared_ptr<Material> defaultMaterial_;
};
