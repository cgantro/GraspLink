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
 * @brief SubMesh 재질 조회
 * @param model MaterialData를 가진 CPU 모델
 * @param assets GPU Material 저장소
 * @param materialIndex 재질 번호. -1이면 기본 재질
 * @return SubMesh에 연결할 Material
 * @throws std::runtime_error 번호가 범위를 벗어나거나 GPU 업로드가 안 된 경우
 */
std::shared_ptr<Material> ResolveMaterial(
    const ModelResource& model,
    const AssetManager& assets,
    int materialIndex)
{
    // glTF 재질 없음 (-1): 기본 Material 사용
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
 * @brief Node 아래에 Render Entity 연결
 *
 * Mesh 하나에 여러 Primitive·Material 가능
 * MeshRenderer는 Material 하나만 사용 → Primitive마다 Render Entity 생성
 * 생성한 Entity는 Node의 자식 → 부모 Transform 상속
 */
void CreateRenderEntities(
    Scene& scene,
    const ModelResource& model,
    const AssetManager& assets,
    const std::shared_ptr<Shader>& shader,
    const NodeData& node,
    std::size_t nodeIndex,
    Entity& nodeEntity,
    bool renderSinglePrimitiveOnNode)
{
    // Mesh 없는 Node도 유지: 로봇 Joint pivot 보존
    if (node.meshIndex < 0) return;

    if (node.meshIndex >= static_cast<int>(model.meshes.size()))
        throw std::runtime_error("Invalid mesh index in model node");

    const MeshData& meshData = model.meshes[static_cast<std::size_t>(node.meshIndex)];
    if (!meshData.gpuMesh)
        throw std::runtime_error("Mesh has not been uploaded: " + meshData.name);

    if (renderSinglePrimitiveOnNode && meshData.subMeshes.size() == 1)
    {
        const SubMeshInfo& subMesh = meshData.subMeshes.front();
        nodeEntity
            .set<MeshFilter>(MeshFilter{meshData.gpuMesh, subMesh.indexStart, subMesh.indexCount})
            .set<MeshRenderer>(MeshRenderer{
                shader,
                ResolveMaterial(model, assets, subMesh.defaultMaterialIndex),
                true});
        return;
    }

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
    const std::shared_ptr<Shader>& shader,
    bool renderSinglePrimitiveOnNode)
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

    /* Node와 Entity의 같은 index 사용: parentIndex로 부모 복원 */
    std::vector<Entity> entities(model.nodes.size());

    // 1. 모든 Node 생성 + GLB Local 위치·회전·크기 복원
    for (std::size_t i = 0; i < model.nodes.size(); ++i)
    {
        const NodeData& node = model.nodes[i];
        Entity entity = scene.CreateEntity(node.name);

        entity.SetLocalPosition(node.translation);
        entity.SetLocalRotation(node.rotation);
        entity.SetLocalScale(node.scale);

        entities[i] = entity;
    }

    // 2. parentIndex로 부모·자식 연결
    for (std::size_t i = 0; i < model.nodes.size(); ++i)
    {
        const NodeData& node = model.nodes[i];

        // Root: Scene::CreateEntity가 이미 SceneRoot에 연결
        if (node.parentIndex < 0) continue;

        if (node.parentIndex >= static_cast<int>(entities.size()))
            throw std::runtime_error("Invalid parent index");

        entities[i].SetParent(
            entities[static_cast<std::size_t>(node.parentIndex)]);
    }

    // 3. Mesh가 있는 Node에 Primitive별 Render Component 연결
    for (std::size_t i = 0; i < model.nodes.size(); ++i)
    {
        CreateRenderEntities(
            scene,
            model,
            assets,
            shader,
            model.nodes[i],
            i,
            entities[i],
            renderSinglePrimitiveOnNode);
    }

    return entities[static_cast<std::size_t>(model.rootNodeIndex)];
}
