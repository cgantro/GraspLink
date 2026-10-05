#pragma once

#include "assets/GraphicsTypes.h"

#include <memory>
#include <unordered_map>

class Material;
class Mesh;
class Texture;

/**
 * @brief CPU에 읽힌 모델 자산을 OpenGL 자원으로 만들고 ID별로 공유한다.
 * @details
 * 흐름: GltfLoader가 GLB를 CPU 배열(ModelResource)로 디코드하고, UploadModel이
 * Texture·Material·Mesh를 만든 뒤, PrefabFactory가 Node 계층과 렌더링 Entity를 구성한다.
 * ResourceID 캐시는 같은 ID의 GPU 객체를 재사용한다. ModelResource::meshes의 gpuMesh와
 * Entity의 MeshFilter도 Mesh를 shared_ptr로 보유하므로 캐시를 비워도 그 참조가 남아 있으면
 * 자원은 유지된다. 마지막 Mesh/Texture 참조가 해제될 때 OpenGL 삭제가 일어나므로 Scene과
 * ModelResource 및 이 관리자의 GPU 참조를 정리한 뒤 GL context를 종료해야 한다.
 * 업로드와 마지막 GPU 참조 해제는 유효한 OpenGL context가 현재 thread에 있을 때 수행한다.
 * 캐시와 업로드는 동기화되지 않으므로 동시에 호출하지 않는다.
 */
class AssetManager final
{
public:
    /** @brief 색상 factor만 사용하는 기본 Material을 준비한다. */
    AssetManager();

    /**
     * @brief CPU 모델 데이터를 GPU 자원으로 올리고 공유 참조를 채운다.
     * @param model GltfLoader가 채운 CPU 자산. 성공한 Mesh는 각 gpuMesh에도 보관한다.
     * @throws std::runtime_error 잘못된 Texture 크기·채널, 유효한 Material의 누락된
     *         base color texture, 빈 정점·인덱스 배열 또는 uint32_t 범위를 넘는 Mesh 크기에서 발생한다.
     * @details Texture, Material, Mesh 순으로 처리해 ResourceID 참조를 연결한다. 캐시에
     *          이미 있는 ID는 재사용하고, 픽셀이 비어 있는 Texture는 건너뛴다. 이 Texture를
     *          요구하는 Material은 누락 오류를 낸다. Material이 없는 Primitive는 PrefabFactory가
     *          기본 Material을 선택하지만, 지정 Texture의 실패를 기본 Material로 대체하지 않는다.
     *          채널 수는 Texture::Create2D에서 검사하며 1, 3, 4만 허용한다.
     *          runtime Material에는 baseColorTexture만 연결한다. metallicRoughness·normal·
     *          occlusion·emissive texture ID는 CPU 기록으로 남으며 Material에 연결하지 않는다.
     *          metallic/roughness factor는 Material에 전달된다. 현재 thread의 OpenGL context가 필요하다.
     */
    void UploadModel(ModelResource& model);

    /** @brief ID의 GPU Mesh 공유 참조를 찾는다. @return 캐시에 없으면 빈 참조. */
    std::shared_ptr<Mesh> GetMesh(ResourceID id) const;

    /** @brief ID의 runtime Material 공유 참조를 찾는다. @return 캐시에 없으면 빈 참조. */
    std::shared_ptr<Material> GetMaterial(ResourceID id) const;

    /** @brief ID의 GPU Texture 공유 참조를 찾는다. @return 캐시에 없으면 빈 참조. */
    std::shared_ptr<Texture> GetTexture(ResourceID id) const;

    /** @brief Material이 지정되지 않은 Primitive에 사용할 기본 Material을 반환한다. */
    std::shared_ptr<Material> GetDefaultMaterial() const;

    /**
     * @brief Mesh·Material·Texture 캐시의 소유 참조를 해제한다.
     * @details ModelResource나 Entity의 shared_ptr가 남아 있으면 해당 GPU 자원은 유지된다.
     *          마지막 참조 해제 시 OpenGL 삭제가 발생할 수 있으므로 context가 유효한 동안 호출한다.
     *          기본 Material은 이 캐시와 별도로 보유하므로 유지된다.
     */
    void Clear();

private:
    std::unordered_map<ResourceID, std::shared_ptr<Mesh>> meshes_;
    std::unordered_map<ResourceID, std::shared_ptr<Material>> materials_;
    std::unordered_map<ResourceID, std::shared_ptr<Texture>> textures_;

    std::shared_ptr<Material> defaultMaterial_;
};
