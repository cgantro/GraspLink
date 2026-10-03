#include "assets/PrefabFactory.h"

#include "assets/AssetManager.h"
#include "assets/GraphicsTypes.h"

#include "Material.h"
#include "Mesh.h"
#include "Shader.h"

#include "components/RenderComponents.h"

#include "scene/Scene.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
/**
 * @brief 
 * 
 * @param model 
 * @param assets 
 * @param materialIndex 
 * @return std::shared_ptr<Material> 
 */
std::shared_ptr<Material> ResolveMaterial(const ModelResource& model, const AssetManager& assets, int materialIndex){
    // glTF material == -1 -> Material 미지정 Primitive
    if(materialIndex < 0) return assets.GetDefaultMaterial();
    if(materialIndex >= static_cast<int>(model.materials.size())) throw std::runtime_error("Invalid material index");
    
    const MaterialData& materialData = model.materials[static_cast<std::size_t>(materialIndex)];
    std::shared_ptr<Material> material = assets.GetMaterial(materialData.uniqueID);
    if(!material) throw std::runtime_error("Material has not been uploaded");
    return material;
}

void CreateRenderEntities(Scene& scene, const ModelResource& model, 
                        const AssetManager& assets,const std::shared_ptr<Shader>& shader, 
                        const NodeData& node, std::size_t nodeIndex, Entity& nodeEntity){
    if(node.meshIndex < 0) return;
    if(node.meshIndex >= static_cast<int>(model.meshes.size()))  
        throw std::runtime_error("Invalid mesh index in model node");
    
    const MeshData& meshData = model.meshes[static_cast<std::size_t>(node.meshIndex)];
    if(!meshData.gpuMesh) throw std::runtime_error("Mesh has not been uploaded: " + meshData.name);

    /*
        glTF Mesh는 여러 Primitives를 가진다.
        현재 ECS
            MeshFilter 1개, MeshRenderer 1개 구조
        Primitive별 Render Entity를 만든다.
    */
    for(std::size_t primitiveIndex = 0; primitiveIndex < meshData.subMeshes.size(); primitiveIndex++){
        const SubMeshInfo& subMesh = meshData.subMeshes[primitiveIndex];

        // Node 이름만 사용 시에, 겹칠 수 있음
        const std::string renderEntityName = node.name + "_Primitive_" + std::to_string(nodeIndex) + "_" + std::to_string(primitiveIndex);

        Entity renderEntity = scene.CreateEntity(renderEntityName);

        /*
            Render Entity의 Local Transform은 Identity
            즉, Node World Transform * Identity가 되므로 원래 glTF Node 위치에 그대로 렌더링
        */

        renderEntity.SetParent(nodeEntity);
        std::shared_ptr<Material> material = ResolveMaterial(model,assets,subMesh.defaultMaterialIndex);

        renderEntity.Set<MeshFilter>(MeshFilter{meshData.gpuMesh,subMesh.indexStart,subMesh.indexCount})
                    .Set<MeshRenderer>(MeshRenderer{shader, material, true});
    }
}
} // namespace

Entity PrefabFactory::CreateModel(Scene& scene, const ModelResource& model,
                                const AssetManager& assets, const std::shared_ptr<Shader>& shader){
    
    if(!shader) throw std::runtime_error("PrefabFactory requires a shader");

    if(model.nodes.empty()) throw std::runtime_error("Model has no nodes");

    if(model.rootNodeIndex < 0 || model.rootNodeIndex >= static_cast<int>(model.nodes.size()))
        throw std::runtime_error("Model has invalid root node");
    
    /*
        NodeData idx와 Flecs Entity idx를 동일 대응 시킨다.
        nodes[3] = J1
        entities[3] = J1 Flec Entity
    */
    std::vector<Entity> entities(model.nodes.size());

    // 1. Entity 생성 + Local Transform 복원
    for(std::size_t i = 0; i < model.nodes.size(); i++){
        const NodeData& node = model.nodes[i];
        Entity entity = scene.CreateEntity(node.name);

        entity.SetLocalPosition(node.translation);
        entity.SetLocalRotation(node.rotation);
        entity.SetLocalScale(node.scale);

        entities[i] = entity;
    }

    // 2. Parent / Child 계층 구조 복원
    for(std::size_t i = 0; i < model.nodes.size(); i++){
        const NodeData& node = model.nodes[i];
        // Root는 SceneRoot 아래에 둔다.
        // Scene::CreateEntity가 이미 SceneRoot의 Child로 생성해 둔다.
        if(node.parentIndex < 0) continue;

        if(node.parentIndex >= static_cast<int>(entities.size())) throw std::runtime_error("Invalid parent index");

        entities[i].SetParent(entities[static_cast<std::size_t>(node.parentIndex)]);
    }

    // 3. Mesh Primitive -> Render Entity 생성
    for(std::size_t i = 0; i < model.nodes.size(); i++){
        CreateRenderEntities(scene,model,assets,shader,model.nodes[i],i,entities[i]);
    }

    return entities[static_cast<std::size_t>(model.rootNodeIndex)];
}
