#include "assets/AssetManager.h"

#include "Material.h"
#include "Mesh.h"
#include "Texture.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <limits>
#include <stdexcept>

std::shared_ptr<Texture> AssetManager::GetTexture(ResourceID id) const
{
    auto it = textures_.find(id);
    if (it == textures_.end()) return nullptr;
    return it->second;
}

AssetManager::AssetManager()
{
    defaultMaterial_ = std::make_shared<Material>(
        glm::vec4{0.7F, 0.7F, 0.7F, 1.0F},
        0.0F,
        0.8F);
}

void AssetManager::UploadModel(ModelResource& model)
{
    // 이미지, 재질, 형상 순서로 올린다. 뒤에 처리할 데이터가 앞서 만든 자원을 번호로 찾기 때문이다.

    for (const TextureData& textureData : model.textures)
    {
        if (textures_.find(textureData.uniqueID) != textures_.end()) continue;
        if (textureData.pixels.empty()) continue;

        // 현재 업로드 경로는 모델의 모든 texture 항목을 sRGB 형식으로 만든다. 화면용 재질에는 기본색 이미지만 연결한다.
        // 다른 종류의 이미지를 실제 재질 입력으로 연결하려면 용도에 맞는 색 공간 처리가 필요하다.
        auto texture = Texture::Create2D(
            textureData.width,
            textureData.height,
            textureData.channels,
            textureData.pixels.data(),
            true);

        textures_.emplace(textureData.uniqueID, texture);
    }

    for (const MaterialData& materialData : model.materials)
    {
        if (materials_.find(materialData.uniqueID) != materials_.end()) continue;

        auto material = std::make_shared<Material>(
            materialData.baseColorFactor,
            materialData.metallicFactor,
            materialData.roughnessFactor);

        if (materialData.baseColorTexture.IsValid())
        {
            // 재질 번호 자체가 없는 표면은 PrefabFactory가 기본 Material을 선택한다. 여기서는 재질이 기본색 Texture를 지정했으므로 업로드 결과가 없으면 잘못된 참조로 처리한다.
            const auto textureIterator = textures_.find(materialData.baseColorTexture);
            if (textureIterator == textures_.end())
                throw std::runtime_error("Base color texture was not uploaded");

            material->SetBaseColorTexture(textureIterator->second);
        }

        materials_.emplace(materialData.uniqueID, std::move(material));
    }

    for (MeshData& meshData : model.meshes)
    {
        const auto existing = meshes_.find(meshData.uniqueID);
        if (existing != meshes_.end())
        {
            meshData.gpuMesh = existing->second;
            continue;
        }

        if (meshData.vertices.empty())
            throw std::runtime_error("Cannot upload mesh without vertices: " + meshData.name);
        if (meshData.indices.empty())
            throw std::runtime_error("Cannot upload mesh without indices: " + meshData.name);

        // GPU 그리기 API가 32-bit 원소 개수를 받으므로, 변환 과정에서 큰 값이 잘리지 않게 먼저 범위를 확인한다.
        if (meshData.vertices.size() > std::numeric_limits<std::uint32_t>::max())
            throw std::runtime_error("Mesh vertex count exceeds uint32_t range");
        if (meshData.indices.size() > std::numeric_limits<std::uint32_t>::max())
            throw std::runtime_error("Mesh index count exceeds uint32_t range");

        auto gpuMesh = std::make_shared<Mesh>(
            meshData.vertices.data(),
            static_cast<std::uint32_t>(meshData.vertices.size()),
            meshData.indices.data(),
            static_cast<std::uint32_t>(meshData.indices.size()));

        // ModelResource도 GPU 형상을 공유한다. PrefabFactory가 같은 참조를 Entity에 복사해도 모델이 형상을 계속 보유한다.
        meshData.gpuMesh = gpuMesh;
        meshes_.emplace(meshData.uniqueID, std::move(gpuMesh));
    }
}

std::shared_ptr<Mesh> AssetManager::GetMesh(ResourceID id) const
{
    const auto iterator = meshes_.find(id);
    if (iterator == meshes_.end()) return nullptr;
    return iterator->second;
}

std::shared_ptr<Material> AssetManager::GetMaterial(ResourceID id) const
{
    const auto iterator = materials_.find(id);
    if (iterator == materials_.end()) return nullptr;
    return iterator->second;
}

std::shared_ptr<Material> AssetManager::GetDefaultMaterial() const
{
    return defaultMaterial_;
}

void AssetManager::Clear()
{
    // 이 캐시가 가진 참조만 놓는다. 모델이나 장면이 계속 사용 중인 GPU 객체는 그 참조가 사라질 때까지 유지된다.
    meshes_.clear();
    materials_.clear();
    textures_.clear();
}
