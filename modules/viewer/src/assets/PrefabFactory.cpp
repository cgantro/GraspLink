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
// 재질이 없으면 기본 Material. 잘못된 index나 미업로드 재질은 오류로 처리한다.
std::shared_ptr<Material> ResolveMaterial(
    const ModelResource& model,
    const AssetManager& assets,
    int materialIndex)
{
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

// Primitive 하나면 Node에 직접 렌더 Component를 붙인다.
// 여러 Primitive면 재질별로 자식 Entity를 만들어 같은 Node 변환과 GPU Mesh를 공유한다.
void CreateRenderEntities(
    Scene& scene,
    const ModelResource& model,
    const AssetManager& assets,
    const std::shared_ptr<Shader>& shader,
    const NodeData& node,
    std::size_t nodeIndex,
    Entity& nodeEntity)
{
    if (node.meshIndex < 0) return;

    if (node.meshIndex >= static_cast<int>(model.meshes.size()))
        throw std::runtime_error("Invalid mesh index in model node");

    const MeshData& meshData = model.meshes[static_cast<std::size_t>(node.meshIndex)];
    if (!meshData.gpuMesh)
        throw std::runtime_error("Mesh has not been uploaded: " + meshData.name);

    if (meshData.subMeshes.size() == 1)
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

        // 자식의 Local TRS는 기본값이다. Node의 pivot과 변환을 그대로 이어받는다.
        Entity renderEntity = scene.CreateEntity(renderEntityName);
        renderEntity.SetParent(nodeEntity);

        std::shared_ptr<Material> material =
            ResolveMaterial(model, assets, subMesh.defaultMaterialIndex);

        renderEntity
            .set<MeshFilter>(MeshFilter{
                meshData.gpuMesh,
                subMesh.indexStart,
                subMesh.indexCount})
            .set<MeshRenderer>(MeshRenderer{
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

    // 먼저 모든 Node를 만들어 부모가 배열에서 뒤에 있어도 연결할 수 있게 한다.
    std::vector<Entity> entities(model.nodes.size());

    // GLB Local 값을 저장한다. 부모 연결 뒤 TransformSystem이 Scene 기준 World 행렬을 계산한다.
    for (std::size_t i = 0; i < model.nodes.size(); ++i)
    {
        const NodeData& node = model.nodes[i];
        Entity entity = scene.CreateEntity(node.name);

        entity.SetLocalPosition(node.translation);
        entity.SetLocalRotation(node.rotation);
        entity.SetLocalScale(node.scale);

        entities[i] = entity;
    }

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
