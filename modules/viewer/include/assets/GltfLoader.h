#pragma once

#include "assets/GraphicsTypes.h"

#include <filesystem>

/**
 * @brief glTF/GLB 파일을 읽어 OpenGL/Flecs에 의존하지 않는 ModelResource로 변환한다.
 *
 * @details
 * 데이터 경로는 다음과 같다.
 *
 * `GLB -> TinyGLTF -> GltfLoader -> ModelResource -> AssetManager/PrefabFactory`
 *
 * 이 Loader는 파일 해석만 담당하고 OpenGL object나 Flecs Entity를 만들지 않는다.
 *
 * Transform 변환 규칙:
 * - glTF Node translation/scale은 numeric 값을 별도 배율 없이 그대로 NodeData에 복사한다.
 * - 현재 controller-ready HCR-12A + 2F-85 GLB는 meter 단위로 정규화되어 있으므로 translation/vertex position은 [m]다.
 * - glTF quaternion 저장 순서는 [x,y,z,w], GLM 생성자는 (w,x,y,z)이므로 순서를 바꿔 구성한다.
 * - 현재 ECS Rotation이 Euler vec3이기 때문에 quaternion/matrix rotation을 Euler radians [rad]로 변환해 NodeData::rotation에 저장한다.
 * - Node matrix가 있으면 glm::decompose로 translation/rotation/scale을 분리한다.
 * - Mesh vertex는 Mesh local frame에 그대로 두고 Node hierarchy transform과 분리한다.
 *
 * Geometry/Material 변환 규칙:
 * - glTF FLOAT VEC3/VEC2 accessor를 CPU glm vector 배열로 복사한다.
 * - index는 U8/U16/U32 입력을 모두 내부 uint32_t로 정규화한다.
 * - 여러 Primitive는 하나의 MeshData vertex/index 배열과 SubMeshInfo range로 합친다.
 * - Material/Texture는 ResourceID로 연결하고 실제 GPU upload는 AssetManager가 수행한다.
 *
 * @todo [FUTURE] sparse accessor, multi-root scene의 완전한 표현, 전체 PBR texture/sampler semantics를 확장한다.
 * @todo [FUTURE] ECS Rotation을 quaternion으로 바꾸면 quaternion -> Euler 변환을 제거한다.
 */
class GltfLoader final
{
public:
    /**
     * @brief Binary glTF(.glb) 파일을 CPU-side ModelResource로 로드한다.
     * @param path 읽을 GLB 파일 경로.
     * @return Node hierarchy, CPU mesh/index, material, decoded texture를 포함하는 ModelResource.
     * @throws std::runtime_error 파일 load 실패, 잘못된 accessor/index, 지원하지 않는 primitive/image format 등.
     *
     * @note 위치/길이 단위를 자동으로 mm->m 변환하지 않는다. 입력 asset이 어떤 단위로 정규화되어 있는지가 그대로 유지된다.
     */
    static ModelResource LoadGLB(const std::filesystem::path& path);

private:
    /** @brief Static utility class이므로 instance 생성을 금지한다. */
    GltfLoader() = delete;
};
