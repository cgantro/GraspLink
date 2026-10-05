#pragma once

#include "Entity.h"

#include <memory>

class AssetManager;
class Scene;
class Shader;
struct ModelResource;

/**
 * @brief 업로드된 GLB 모델을 Scene의 Flecs Entity 계층으로 구성한다.
 * @details
 * 흐름: GltfLoader가 만든 NodeData의 Local TRS와 부모 관계를 Entity에 복사하고,
 * 업로드된 Mesh와 Material을 렌더 Component로 연결한다. Mesh가 없는 Node도 관절 pivot과
 * 자식 좌표계의 기준이므로 유지한다. AssetManager::UploadModel은 이 함수보다 먼저 호출한다.
 * 모든 Node Entity를 먼저 만든 뒤 parentIndex로 연결하므로 부모가 nodes 배열에서 자식보다
 * 뒤에 있어도 계층을 구성할 수 있다. SetParent는 Local TRS를 유지하며 World 행렬은
 * TransformSystemModule의 다음 갱신에서 부모부터 계산된다.
 *
 * 단일 Primitive Mesh는 Node Entity에 렌더 Component를 직접 붙인다. Primitive가 여러 개면
 * 기본 Local TRS를 가진 자식 Entity를 Primitive마다 만들고, Node의 변환을 상속하게 한다.
 * 각 MeshFilter 범위는 합쳐진 index 배열의 index 원소 위치와 개수이며 byte 단위가 아니다.
 *
 * Scene이 Entity hierarchy를 소유하며 반환 Entity wrapper는 수명을 연장하지 않는다. Scene 정리
 * 또는 Flecs World 파괴 뒤 wrapper는 무효다. Mesh·Material·Shader는 렌더 Component가 공유
 * 참조를 보유하지만, GPU 자원의 마지막 참조 해제까지 OpenGL context가 살아 있어야 한다.
 * 생성 중 오류가 나면 예외를 던지며, 앞서 만든 Entity를 되돌리는 원자적 정리는 하지 않는다.
 */
class PrefabFactory final
{
public:
    /**
     * @brief 모델 Node 계층과 렌더 Entity를 만들고 root wrapper를 반환한다.
     * @param scene Entity를 생성할 활성 Scene. Entity와 계층의 소유자는 Scene의 Flecs World다.
     * @param model GLB에서 읽고 AssetManager::UploadModel로 GPU 자원을 준비한 모델.
     * @param assets 업로드된 ResourceID를 GPU Mesh·Material 참조로 해석하는 관리자.
     * @param shader 각 Primitive의 MeshRenderer가 공유할 Shader.
     * @return ModelResource::rootNodeIndex에 해당하는 Scene 소유 Entity wrapper.
     * @throws std::runtime_error shader, Node 계층, Mesh 연결·업로드 또는 Material 참조가
     *         잘못되었거나 필요한 GPU 자원이 준비되지 않았을 때 발생한다. 실패 시 이미 생성된
     *         Entity는 Scene에 남을 수 있다.
     * @details Node 위치는 부모 기준 [m], 회전은 Euler [rad], 크기는 배율이다. GPU 자원은
     *          ResourceID 캐시에서 찾아 공유한다. Material index가 -1인 Primitive만 기본
     *          Material을 사용하며, 유효하지 않은 index나 업로드되지 않은 Material은 오류다.
     *          Mesh가 여러 Primitive를 가지면 각 하위 Entity가 해당 Primitive의 index 범위를
     *          그리며, 하나뿐이면 Node Entity 자체가 그 범위를 그린다.
     */
    static Entity CreateModel(
        Scene& scene,
        const ModelResource& model,
        const AssetManager& assets,
        const std::shared_ptr<Shader>& shader);

private:
    PrefabFactory() = delete;
};
