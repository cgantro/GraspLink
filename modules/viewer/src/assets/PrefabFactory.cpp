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
#include <cstdint>
#include <vector>

namespace
{
/**
 * @brief 표면 묶음에 지정된 재질을 모델의 재질 목록과 자원 캐시에서 찾는다.
 * @details -1은 glTF 표면에 재질 번호가 없다는 뜻이므로 이 경우만 기본 Material을 쓴다. 목록 범위를 벗어난 번호나 GPU에 올리지 않은 재질은 잘못된 모델 연결이므로 예외를 던진다.
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

void ValidateModelBeforeSceneChanges(
    const ModelResource& model,
    const AssetManager& assets,
    const std::shared_ptr<Shader>& shader)
{
    if (!shader)
        throw std::runtime_error("PrefabFactory requires a shader");
    if (model.nodes.empty())
        throw std::runtime_error("Model has no nodes");
    if (model.rootNodeIndex < 0 || model.rootNodeIndex >= static_cast<int>(model.nodes.size()))
        throw std::runtime_error("Model has invalid root node");

    std::vector<std::uint8_t> parentState(model.nodes.size(), 0);
    for (std::size_t start = 0; start < model.nodes.size(); ++start)
    {
        std::vector<std::size_t> path;
        std::size_t current = start;
        while (parentState[current] == 0)
        {
            parentState[current] = 1;
            path.push_back(current);
            const int parent = model.nodes[current].parentIndex;
            if (parent < -1 || parent >= static_cast<int>(model.nodes.size()))
                throw std::runtime_error("Invalid parent index");
            if (parent < 0)
                break;
            current = static_cast<std::size_t>(parent);
        }
        if (parentState[current] == 1 && model.nodes[current].parentIndex >= 0)
            throw std::runtime_error("Model contains a parent cycle");
        for (const std::size_t index : path)
            parentState[index] = 2;
    }

    for (const NodeData& node : model.nodes)
    {
        if (node.meshIndex < -1 || node.meshIndex >= static_cast<int>(model.meshes.size()))
            throw std::runtime_error("Invalid mesh index in model node");
        if (node.meshIndex < 0)
            continue;

        const MeshData& mesh = model.meshes[static_cast<std::size_t>(node.meshIndex)];
        if (!mesh.gpuMesh)
            throw std::runtime_error("Mesh has not been uploaded: " + mesh.name);
        for (const SubMeshInfo& subMesh : mesh.subMeshes)
        {
            const std::size_t begin = subMesh.indexStart;
            const std::size_t count = subMesh.indexCount;
            if (count == 0 || count % 3 != 0 || begin > mesh.indices.size() ||
                count > mesh.indices.size() - begin)
                throw std::runtime_error("Invalid primitive index range");
            if (subMesh.defaultMaterialIndex < -1)
                throw std::runtime_error("Invalid material index");
            (void)ResolveMaterial(model, assets, subMesh.defaultMaterialIndex);
            for (std::size_t index = begin; index < begin + count; ++index)
                if (mesh.indices[index] >= mesh.vertices.size())
                    throw std::runtime_error("Invalid primitive vertex index");
        }
    }
}

/**
 * @brief 부품이 가리키는 형상과 그 표면별 그리기 범위를 Entity에 연결한다.
 * @details AssetManager::UploadModel은 삼각형 형상을 GPU에 올려 gpuMesh를 채운다. 한 형상의 표면 묶음들은 같은 GPU Mesh를 공유하고 indexStart/indexCount로 각자 사용할 연결 번호 구간을 고른다.
 * 시작 위치는 번호 배열의 원소 개수이며 Renderer가 GPU 명령을 만들 때 byte 위치로 바꾼다. 표면이 하나면 부품 Entity가 직접 그린다.
 * 여러 표면은 재질이나 연결 번호 범위가 다를 수 있어 자식 Entity로 나눈다.
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

        // 새 자식의 부모 기준 위치와 회전은 0, 크기는 1이다. 별도 이동 없이 부모 부품의 위치·회전을 물려받게 한다.
        Entity renderEntity = scene.CreateEntity(renderEntityName);
        renderEntity.SetParent(nodeEntity);

        // 재질은 자원 번호로 찾아 공유한다. 파일에 재질을 지정하지 않은 표면만 기본 재질을 쓴다.
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
    // 참조와 계층을 먼저 검증해 잘못된 Asset 때문에 Scene에 일부 Entity만 남는 일을 막는다.
    ValidateModelBeforeSceneChanges(model, assets, shader);

    // 첫 단계: 부모 부품이 배열에서 자식 뒤에 있어도 되도록 모든 Entity를 먼저 만든다.
    std::vector<Entity> entities(model.nodes.size());

    // 둘째 단계: 파일의 부모 기준 위치 [m], 길이 1인 회전값, 크기 배율을 Entity에 복사한다.
    for (std::size_t i = 0; i < model.nodes.size(); ++i)
    {
        const NodeData& node = model.nodes[i];
        Entity entity = scene.CreateEntity(node.name);

        entity.SetLocalPosition(node.translation);
        entity.SetLocalRotation(node.rotation);
        entity.SetLocalScale(node.scale);

        entities[i] = entity;
    }

    // 셋째 단계: 미리 만든 Entity 번호로 부모 관계를 설정한다. SetParent는 부모 기준 위치·회전·크기를 유지한다.
    for (std::size_t i = 0; i < model.nodes.size(); ++i)
    {
        const NodeData& node = model.nodes[i];

        // 부모가 없는 최상위 Entity는 CreateEntity가 장면의 최상위 기준에 이미 연결했다.
        if (node.parentIndex < 0) continue;

        if (node.parentIndex >= static_cast<int>(entities.size()))
            throw std::runtime_error("Invalid parent index");

        entities[i].SetParent(
            entities[static_cast<std::size_t>(node.parentIndex)]);
    }

    // 마지막 단계: 계층을 만든 뒤 형상 번호와 GPU 업로드, 재질 참조가 유효한지 확인해 화면 그리기 정보를 붙인다.
    // 중간에 실패해도 앞서 만든 Entity를 장면에서 지우는 되돌리기는 하지 않는다.
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
