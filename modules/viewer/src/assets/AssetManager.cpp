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
    // Material이 없는 Primitive도 보이도록 중성 회색 fallback을 준비한다.
    defaultMaterial_ = std::make_shared<Material>(
        glm::vec4{0.7F, 0.7F, 0.7F, 1.0F},
        0.0F,
        0.8F);
}

void AssetManager::UploadModel(ModelResource& model)
{
    /*
        CPU -> GPU 업로드 순서가 중요하다.
        Material이 Texture를 참조하므로 Texture를 먼저 생성하고,
        Material이 준비된 뒤 Mesh buffer를 생성한다.
    */

    // TextureData의 decoded pixel을 GPU 2D texture로 올린다.
    for (const TextureData& textureData : model.textures)
    {
        if (textures_.find(textureData.uniqueID) != textures_.end()) continue;
        if (textureData.pixels.empty()) continue;

        // 현재 model texture는 baseColor 용도로만 사용하므로 sRGB=true로 업로드한다.
        // TODO(FUTURE): normal/metallicRoughness 같은 data texture가 들어오면 semantic별 color space를 분리한다.
        auto texture = Texture::Create2D(
            textureData.width,
            textureData.height,
            textureData.channels,
            textureData.pixels.data(),
            true);

        textures_.emplace(textureData.uniqueID, texture);
    }

    // CPU Material factor와 Texture ResourceID를 runtime Material로 연결한다.
    for (const MaterialData& materialData : model.materials)
    {
        if (materials_.find(materialData.uniqueID) != materials_.end()) continue;

        auto material = std::make_shared<Material>(
            materialData.baseColorFactor,
            materialData.metallicFactor,
            materialData.roughnessFactor);

        if (materialData.baseColorTexture.IsValid())
        {
            const auto textureIterator = textures_.find(materialData.baseColorTexture);
            if (textureIterator == textures_.end())
                throw std::runtime_error("Base color texture was not uploaded");

            material->SetBaseColorTexture(textureIterator->second);
        }

        // TODO(FUTURE): remaining PBR texture ResourceID들도 여기에서 Material에 연결한다.
        materials_.emplace(materialData.uniqueID, std::move(material));
    }

    // Vertex/Index CPU arrays를 OpenGL VBO/EBO/VAO로 업로드한다.
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

        // PrefabFactory도 바로 사용할 수 있도록 CPU IR에 weak cache 역할의 shared_ptr를 채운다.
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
    // shared_ptr reference count가 0이 되면 각 GPU wrapper의 destructor가 OpenGL resource를 정리한다.
    meshes_.clear();
    materials_.clear();
    textures_.clear();
}
