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
 * @brief SubMesh의 material index를 실제 runtime Material로 해석한다.
 * @param model MaterialData 배열을 가진 CPU-side model.
 * @param assets GPU Material cache를 가진 AssetManager.
 * @param materialIndex ModelResource::materials index. -1이면 fallback material을 뜻한다.
 * @return SubMesh를 그릴 runtime Material.
 * @throws std::runtime_error 범위를 벗어난 material index 또는 아직 업로드되지 않은 Material인 경우.
 */
std::shared_ptr<Material> ResolveMaterial(
    const ModelResource& model,
    const AssetManager& assets,
    int materialIndex)
{
    // glTF에서 material=-1은 "Material 미지정"이므로 fallback을 사용한다.
    if (materialIndex < 0) return assets.GetDefaultMaterial();

    if (materialIndex >= static_cast<int>(model.materials.size()))
        throw std::runtime_error("Invalid material index");

    const MaterialData& materialData =
        model.materials[static_cast<std::size_t>(materialIndex)];

    std::shared_ptr<Material> material = assets.GetMaterial(materialData.uniqueID);
    if (!material)
        throw std::runtime_error("Material has not been uploaded");

    return material;
}

/**
 * @brief 하나의 공간 Node 아래에 SubMesh별 Render Entity를 만든다.
 *
 * @details
 * glTF Mesh 하나는 서로 다른 Material을 가진 여러 Primitive를 포함할 수 있다.
 * 현재 ECS의 MeshRenderer는 하나의 Material만 가지므로 Primitive마다 별도 Render Entity를 만들고,
 * 모든 Render Entity를 원본 Node의 child로 둔다. Render Entity의 Local Transform은 identity이므로
 * 부모 Node의 World Transform을 그대로 상속한다.
 *
 * @param scene Render Entity를 생성할 Scene.
 * @param model CPU-side ModelResource.
 * @param assets GPU Mesh/Material cache.
 * @param shader Primitive를 그릴 공통 Shader.
 * @param node 현재 공간 Node 데이터.
 * @param nodeIndex ModelResource::nodes에서의 index. 이름 충돌 방지를 위해 render entity 이름에 사용한다.
 * @param nodeEntity 현재 공간 Node에 대응하는 Flecs Entity.
 *
 * @todo [FUTURE] 인스턴스 namespace가 도입되면 render entity 이름 조합 정책을 별도 helper로 분리한다.
 */
void CreateRenderEntities(
    Scene& scene,
    const ModelResource& model,
    const AssetManager& assets,
    const std::shared_ptr<Shader>& shader,
    const NodeData& node,
    std::size_t nodeIndex,
    Entity& nodeEntity)
{
    // Mesh가 없는 Joint/Transform-only Node도 hierarchy에는 반드시 남겨야 한다.
    if (node.meshIndex < 0) return;

    if (node.meshIndex >= static_cast<int>(model.meshes.size()))
        throw std::runtime_error("Invalid mesh index in model node");

    const MeshData& meshData = model.meshes[static_cast<std::size_t>(node.meshIndex)];
    if (!meshData.gpuMesh)
        throw std::runtime_error("Mesh has not been uploaded: " + meshData.name);

    for (std::size_t primitiveIndex = 0;
         primitiveIndex < meshData.subMeshes.size();
         ++primitiveIndex)
    {
        const SubMeshInfo& subMesh = meshData.subMeshes[primitiveIndex];

        const std::string renderEntityName =
            node.name + "_Primitive_" +
            std::to_string(nodeIndex) + "_" +
            std::to_string(primitiveIndex);

        Entity renderEntity = scene.CreateEntity(renderEntityName);
        renderEntity.SetParent(nodeEntity);

        std::shared_ptr<Material> material =
            ResolveMaterial(model, assets, subMesh.defaultMaterialIndex);

        renderEntity
            .Set<MeshFilter>(MeshFilter{
                meshData.gpuMesh,
                subMesh.indexStart,
                subMesh.indexCount})
            .Set<MeshRenderer>(MeshRenderer{
                shader,
                material,
                true});
    }
}
} // namespace

Entity PrefabFactory::CreateModel(
    Scene& scene,
    const ModelResource& model,
    const AssetManager& assets,
    const std::shared_ptr<Shader>& shader)
{
    if (!shader)
        throw std::runtime_error("PrefabFactory requires a shader");

    if (model.nodes.empty())
        throw std::runtime_error("Model has no nodes");

    if (model.rootNodeIndex < 0 ||
        model.rootNodeIndex >= static_cast<int>(model.nodes.size()))
    {
        throw std::runtime_error("Model has invalid root node");
    }

    /*
        NodeData index와 Flecs Entity index를 1:1로 대응시킨다.

        model.nodes[3] = J1
        entities[3]    = J1 Entity

        pointer tree를 따로 만들지 않아도 parentIndex로 hierarchy를 복원할 수 있다.
    */
    std::vector<Entity> entities(model.nodes.size());

    // 1) 모든 공간 Node를 먼저 만들고 GLB Local TRS를 복원한다.
    for (std::size_t i = 0; i < model.nodes.size(); ++i)
    {
        const NodeData& node = model.nodes[i];
        Entity entity = scene.CreateEntity(node.name);

        entity.SetLocalPosition(node.translation);
        entity.SetLocalRotation(node.rotation);
        entity.SetLocalScale(node.scale);

        entities[i] = entity;
    }

    // 2) parentIndex를 이용해 ChildOf hierarchy를 복원한다.
    for (std::size_t i = 0; i < model.nodes.size(); ++i)
    {
        const NodeData& node = model.nodes[i];

        // Root는 Scene::CreateEntity가 이미 SceneRoot 아래에 둔 상태다.
        if (node.parentIndex < 0) continue;

        if (node.parentIndex >= static_cast<int>(entities.size()))
            throw std::runtime_error("Invalid parent index");

        entities[i].SetParent(
            entities[static_cast<std::size_t>(node.parentIndex)]);
    }

    // 3) Mesh가 있는 Node마다 Primitive 단위 Render Entity를 생성한다.
    for (std::size_t i = 0; i < model.nodes.size(); ++i)
    {
        CreateRenderEntities(
            scene,
            model,
            assets,
            shader,
            model.nodes[i],
            i,
            entities[i]);
    }

    return entities[static_cast<std::size_t>(model.rootNodeIndex)];
}
