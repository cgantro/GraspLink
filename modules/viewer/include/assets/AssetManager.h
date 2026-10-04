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
/*
 * [추가 그래픽스 용어 설명]
 * - CPU-side Resource: 파일에서 읽었지만 아직 GPU에 올리지 않은 일반 메모리 데이터.
 * - GPU Resource: OpenGL Buffer/Texture처럼 GPU가 직접 렌더링에 사용하는 자원.
 * - Upload: CPU 메모리의 데이터를 GPU 메모리/객체로 복사해 렌더링 가능하게 만드는 과정.
 * - Cache: 같은 ResourceID를 다시 요청했을 때 GPU 객체를 새로 만들지 않고 기존 것을 재사용하는 저장소.
 * - ResourceID: Mesh/Material/Texture를 찾기 위한 경량 식별자. 실제 GPU 객체 자체는 아니다.
 * - Lifetime owner: 자원이 언제 생성되고 언제 파괴되는지 책임지는 객체.
 * - shared_ptr: 여러 객체가 같은 GPU Resource를 공유 참조할 수 있게 하는 C++ 소유권 타입.
 * - Fallback Material: 모델에 Material이 없을 때 대신 사용하는 기본 Material.
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
    // ResourceID -> GPU Mesh cache.
    std::unordered_map<ResourceID, std::shared_ptr<Mesh>> meshes_;
    // ResourceID -> runtime Material cache.
    std::unordered_map<ResourceID, std::shared_ptr<Material>> materials_;
    // ResourceID -> GPU Texture cache.
    std::unordered_map<ResourceID, std::shared_ptr<Texture>> textures_;

    // Material이 지정되지 않은 Primitive에 공통으로 사용할 기본 Material.
    std::shared_ptr<Material> defaultMaterial_;
};
