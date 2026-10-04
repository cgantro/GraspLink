#pragma once

#include "Entity.h"

#include <memory>

class AssetManager;
class Scene;
class Shader;
struct ModelResource;

/**
 * @brief ModelResource의 Node/Mesh 정보를 실제 Flecs Entity hierarchy로 생성하는 factory.
 *
 * @details
 * GltfLoader가 만든 CPU-side NodeData를 Viewer ECS 표현으로 옮긴다.
 *
 * 변환 규칙:
 * - NodeData::translation -> `(Position, Local)` 그대로 복사
 * - NodeData::rotation    -> `(Rotation, Local)` Euler radians 그대로 복사
 * - NodeData::scale       -> `(Scale, Local)` 그대로 복사
 * - parentIndex/childrenIndices -> Flecs ChildOf hierarchy
 * - meshIndex/SubMeshInfo -> 렌더용 child Entity의 MeshFilter/MeshRenderer
 *
 * 이 단계에서도 길이/각도 단위를 재변환하지 않는다. 따라서 controller-ready HCR asset의 translation은 [m],
 * rotation은 GltfLoader가 정규화한 Euler [rad] 상태로 ECS에 들어간다.
 *
 * 공간 Node와 렌더링 SubMesh Entity를 분리하기 때문에 J1~J6처럼 Mesh가 없는 transform-only Joint Node도
 * hierarchy에 그대로 남는다. RobotTransformAdapter는 이 Joint Entity를 이름으로 찾아 rotation을 갱신한다.
 *
 * @todo [FUTURE] 동일 모델을 여러 번 생성할 때 이름 충돌을 피할 instance namespace/prefix를 지원한다.
 */
class PrefabFactory final
{
public:
    /**
     * @brief 모델 전체를 Scene에 Flecs hierarchy로 생성하고 root Entity를 반환한다.
     * @param scene Entity를 생성할 Scene. lifetime/World는 Scene이 관리한다.
     * @param model GltfLoader가 만든 CPU-side ModelResource.
     * @param assets Mesh/Material/Texture GPU resource를 조회할 AssetManager.
     * @param shader 생성되는 render entity에 연결할 Shader.
     * @return 생성된 model hierarchy의 root Entity wrapper.
     *
     * @note ModelResource에 저장된 local transform 수치를 그대로 Component에 복사하며 단위 변환을 하지 않는다.
     */
    static Entity CreateModel(
        Scene& scene,
        const ModelResource& model,
        const AssetManager& assets,
        const std::shared_ptr<Shader>& shader);

private:
    /** @brief Static factory이므로 instance 생성을 금지한다. */
    PrefabFactory() = delete;
};
