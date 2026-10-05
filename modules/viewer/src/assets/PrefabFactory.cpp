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
 * @brief Primitive가 지정한 runtime Material을 ResourceID로 찾는다.
 * @details materialIndex가 -1일 때만 기본 Material로 대체한다. 배열 범위를 벗어난 index와
 *          업로드 캐시에 없는 유효한 Material은 모델 연결 오류이므로 예외로 알린다.
 */
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

/**
 * @brief Node의 Mesh Primitive를 렌더 Component로 연결한다.
 * @details MeshData::gpuMesh는 AssetManager::UploadModel이 채운다. 모든 Primitive는 합쳐진
 *          GPU Mesh를 공유하고, indexStart/indexCount로 각자 그릴 uint32 index 원소 구간을 고른다.
 *          구간의 원점은 Mesh index 배열이며 Renderer가 draw할 때 byte offset으로 바꾼다.
 *          단일 Primitive는 같은 변환을 쓰도록 Node Entity 자체에 붙이고, 여러 Primitive는
 *          재질과 draw 구간이 서로 다를 수 있어 Primitive별 자식 Entity로 나눈다.
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

        // 자식의 Local TRS는 Scene::CreateEntity 기본값(위치·회전 0, 크기 1)이다.
        // 별도 보정 변환 없이 Node의 pivot과 변환을 이어받게 한다.
        Entity renderEntity = scene.CreateEntity(renderEntityName);
        renderEntity.SetParent(nodeEntity);

        // Material은 ResourceID 캐시에서 공유한다. 지정되지 않은 경우만 기본 재질을 쓴다.
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
    // Scene이 활성화되어야 Entity를 만들 수 있다. 호출자는 GLB GPU 업로드도 먼저 마쳐야 한다.
    if (!shader)
        throw std::runtime_error("PrefabFactory requires a shader");

    if (model.nodes.empty())
        throw std::runtime_error("Model has no nodes");

    if (model.rootNodeIndex < 0 ||
        model.rootNodeIndex >= static_cast<int>(model.nodes.size()))
    {
        throw std::runtime_error("Model has invalid root node");
    }

    // 1차 생성: 부모 Entity가 배열에서 뒤에 있어도 연결 가능하도록 모든 Node handle을 확보한다.
    std::vector<Entity> entities(model.nodes.size());

    // 1차 변환 복사: GLB 값은 부모 기준 Local이다. 위치 [m], Euler 회전 [rad], 크기 배율을 보존한다.
    for (std::size_t i = 0; i < model.nodes.size(); ++i)
    {
        const NodeData& node = model.nodes[i];
        Entity entity = scene.CreateEntity(node.name);

        entity.SetLocalPosition(node.translation);
        entity.SetLocalRotation(node.rotation);
        entity.SetLocalScale(node.scale);

        entities[i] = entity;
    }

    // 2차 연결: 이미 확보한 handle로 부모 관계를 설정한다. SetParent는 Local TRS를 보존한다.
    for (std::size_t i = 0; i < model.nodes.size(); ++i)
    {
        const NodeData& node = model.nodes[i];

        // 부모가 없는 root는 Scene::CreateEntity가 SceneRoot에 연결했으므로 그대로 둔다.
        if (node.parentIndex < 0) continue;

        if (node.parentIndex >= static_cast<int>(entities.size()))
            throw std::runtime_error("Invalid parent index");

        entities[i].SetParent(
            entities[static_cast<std::size_t>(node.parentIndex)]);
    }

    // 3차 렌더 연결: 계층 구성 후 Mesh 유효성·GPU 업로드·Material 참조를 확인한다.
    // 어느 단계에서 예외가 나도 생성된 Entity를 삭제하는 rollback은 하지 않는다.
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
