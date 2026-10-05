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
    // 각 단계는 뒤 단계의 ResourceID 참조를 해석할 수 있도록 순서대로 실행한다.

    for (const TextureData& textureData : model.textures)
    {
        if (textures_.find(textureData.uniqueID) != textures_.end()) continue;
        if (textureData.pixels.empty()) continue;

        // 현재 runtime에서 색상 입력으로 쓰는 baseColor texture이므로 sRGB로 저장한다.
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
            // 미지정 Material은 PrefabFactory가 기본 Material을 고른다. 지정 참조의 누락은
            // 손상된 자산 연결이므로 기본 색으로 숨기지 않고 실패시킨다.
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

        // Mesh/OpenGL draw count가 uint32_t 기반이므로 narrowing 전에 범위를 검증한다.
        if (meshData.vertices.size() > std::numeric_limits<std::uint32_t>::max())
            throw std::runtime_error("Mesh vertex count exceeds uint32_t range");
        if (meshData.indices.size() > std::numeric_limits<std::uint32_t>::max())
            throw std::runtime_error("Mesh index count exceeds uint32_t range");

        auto gpuMesh = std::make_shared<Mesh>(
            meshData.vertices.data(),
            static_cast<std::uint32_t>(meshData.vertices.size()),
            meshData.indices.data(),
            static_cast<std::uint32_t>(meshData.indices.size()));

        // ModelResource도 Mesh를 공유해 PrefabFactory가 Entity로 복사한 뒤에도 자원 수명을 잇는다.
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
    // 캐시 소유권만 놓는다. ModelResource/Entity가 참조 중이면 GPU 객체는 그쪽 수명에 남는다.
    meshes_.clear();
    materials_.clear();
    textures_.clear();
}
