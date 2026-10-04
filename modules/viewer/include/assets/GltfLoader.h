#pragma once

#include "assets/GraphicsTypes.h"

#include <filesystem>

/**
 * @brief glTF/GLB 파일을 읽어 렌더러 독립적인 ModelResource로 변환한다.
 *
 * @details
 * GLB -> TinyGLTF -> GltfLoader -> ModelResource 순서로 동작한다.
 * 이 계층에서는 OpenGL 객체를 생성하지 않는다. 파일 해석과 GPU 업로드를
 * 분리하면 Loader를 테스트하기 쉽고 다른 렌더링 API로 교체하기도 쉬워진다.
 *
 * 주요 책임:
 * - Node hierarchy와 local transform 복원
 * - Vertex/Index/Primitive 읽기
 * - Material factor와 texture image 읽기
 * - ModelResource용 ResourceID 생성
 *
 * @todo [FUTURE] 현재 지원 범위를 넘어 multi-root scene, 더 다양한 accessor format,
 *       전체 PBR texture/sampler semantics가 필요해지면 이 계층에서 확장한다.
 */
class GltfLoader final
{
public:
    /**
     * @brief Binary glTF(.glb) 파일을 로드한다.
     * @param path GLB 파일 경로.
     * @return 파일 내용을 CPU-side ModelResource로 변환한 결과.
     * @throws std::runtime_error 잘못된 파일, 지원하지 않는 accessor/primitive 등이 들어온 경우.
     */
    static ModelResource LoadGLB(const std::filesystem::path& path);

private:
    GltfLoader() = delete;
};
