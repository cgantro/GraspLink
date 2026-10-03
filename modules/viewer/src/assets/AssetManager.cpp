#include "assets/AssetManager.h"

#include "Material.h"
#include "Mesh.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <limits>
#include <stdexcept>

AssetManager::AssetManager(){
    defaultMaterial_ = std::make_shared<Material>(glm::vec4{0.7F,0.7F,0.7F,1.0F}, 0.0F, 0.8F);   
}

void AssetManager::UploadModel(ModelResource& model){
    // Material
    for(const MaterialData& materialData : model.materials){
        // 이미 GPU Resource가 생성되어 있으면 만들지 않는다.
        if(materials_.find(materialData.uniqueID) != materials_.end()) continue;

        auto material = std::make_shared<Material>(
            materialData.baseColorFactor,
            materialData.metallicFactor,
            materialData.roughnessFactor
        );

        materials_.emplace(materialData.uniqueID,std::move(material));
    }

    // Mesh
    for(MeshData& meshData: model.meshes){
        // 동일 Mesh가 이미 있는 경우 재사용
        const auto existing = meshes_.find(meshData.uniqueID);
        if(existing != meshes_.end()){
            meshData.gpuMesh = existing->second; continue;
        }

        if(meshData.vertices.empty()) throw std::runtime_error("Cannot upload mesh without vertices: " + meshData.name);
        if(meshData.indices.empty())  throw std::runtime_error("Cannot upload mesh without indices: " + meshData.name);

        // Mesh 생성자가, uint32_t를 사용 -> 범위 확인
        if(meshData.vertices.size() > std::numeric_limits<std::uint32_t>::max()) 
            throw std::runtime_error("Mesh vertex count exceeds uint32_t range");
        
        if(meshData.indices.size() > std::numeric_limits<std::uint32_t>::max())
            throw std::runtime_error("Mesh index count exceeds uint32_t range");

        auto gpuMesh = std::make_shared<Mesh>(
            meshData.vertices.data(),
            static_cast<std::uint32_t>(meshData.vertices.size()),
            meshData.indices.data(),
            static_cast<std::uint32_t>(meshData.indices.size())
        );

        // ModelResource에서도 GPU Resource를 참조 가능하게 저장
        meshData.gpuMesh = gpuMesh;

        // AssetManager가 Resource Lifetime 관리
        meshes_.emplace(meshData.uniqueID,std::move(gpuMesh));
    }
}

/**
 * @brief 
 * 
 * @param id 
 * @return std::shared_ptr<Mesh> 
 */
std::shared_ptr<Mesh> AssetManager::GetMesh(ResourceID id) const {
    const auto iterator = meshes_.find(id);
    if(iterator == meshes_.end()) return nullptr;
    return iterator->second;
}

/**
 * @brief 
 * 
 * @param id 
 * @return std::shared_ptr<Material> 
 */
std::shared_ptr<Material>
AssetManager::GetMaterial(ResourceID id) const{
    const auto iterator =
        materials_.find(id);
    if (iterator == materials_.end()) return nullptr;
    return iterator->second;
}

/**
 * @brief 
 * 
 * @return std::shared_ptr<Material> 
 */
std::shared_ptr<Material>
AssetManager::GetDefaultMaterial() const{
    return defaultMaterial_;
}


void AssetManager::Clear(){
    meshes_.clear();
    materials_.clear();
}