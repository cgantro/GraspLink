#pragma once

#include "assets/GraphicsTypes.h"

#include <memory>
#include <unordered_map>

class Material;
class Mesh;
class Texture;

/**
 * @brief CPU-side ModelResource를 GPU 리소스로 변환하고 캐시한다.
 *
 * @details
 * 데이터 흐름은 다음과 같다.
 *
 * GLB -> GltfLoader -> ModelResource -> AssetManager -> Mesh/Material/Texture
 *
 * ResourceID를 key로 캐시하기 때문에 같은 모델이 다시 업로드되어도
 * 이미 생성된 GPU 리소스를 재사용할 수 있다. AssetManager가 실제 GPU 객체의
 * lifetime owner이고, 다른 객체는 shared_ptr로 이를 참조한다.
 *
 * @todo [FUTURE] metallic-roughness, normal, occlusion, emissive texture까지
 *       Material과 연결하는 PBR 업로드 단계를 확장한다.
 */
class AssetManager final
{
public:
    /** @brief fallback 용 기본 Material을 생성한다. */
    AssetManager();

    /**
     * @brief ModelResource의 Texture, Material, Mesh를 GPU 리소스로 업로드한다.
     * @param model 업로드할 CPU-side 모델. MeshData::gpuMesh가 함께 채워진다.
     */
    void UploadModel(ModelResource& model);

    /** @brief ID에 해당하는 Mesh를 조회한다. 없으면 nullptr을 반환한다. */
    std::shared_ptr<Mesh> GetMesh(ResourceID id) const;

    /** @brief ID에 해당하는 Material을 조회한다. 없으면 nullptr을 반환한다. */
    std::shared_ptr<Material> GetMaterial(ResourceID id) const;

    /** @brief ID에 해당하는 Texture를 조회한다. 없으면 nullptr을 반환한다. */
    std::shared_ptr<Texture> GetTexture(ResourceID id) const;

    /** @brief Material이 없는 Primitive에 사용할 fallback Material을 반환한다. */
    std::shared_ptr<Material> GetDefaultMaterial() const;

    /** @brief Manager가 보관한 GPU 리소스 참조를 모두 해제한다. */
    void Clear();

private:
    std::unordered_map<ResourceID, std::shared_ptr<Mesh>> meshes_;
    std::unordered_map<ResourceID, std::shared_ptr<Material>> materials_;
    std::unordered_map<ResourceID, std::shared_ptr<Texture>> textures_;

    std::shared_ptr<Material> defaultMaterial_;
};
