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
/*
 * [추가 그래픽스 용어 설명]
 * - glTF: 3D Mesh/Material/Texture/Node/Animation 등을 교환하기 위한 표준 3D 포맷.
 * - GLB: glTF의 JSON과 binary 데이터를 하나의 binary 파일에 묶은 형식.
 * - TinyGLTF: glTF/GLB 파일 구조를 읽기 위해 사용하는 외부 C++ library.
 * - Loader: 파일 포맷을 읽어 프로그램 내부 데이터 구조로 바꾸는 계층.
 * - Node Hierarchy: 부모/자식 Node 관계. Robot Joint처럼 Mesh가 없는 transform-only Node도 포함된다.
 * - Primitive: 하나의 Material과 draw 방식으로 그릴 수 있는 Mesh의 하위 렌더링 단위.
 * - Accessor: glTF에서 buffer의 어느 위치/형식으로 Vertex/Index 값을 읽을지 설명하는 메타데이터.
 * - IR(Intermediate Representation): 파일 포맷과 GPU/ECS 사이에서 사용하는 중간 데이터 표현. 여기서는 ModelResource.
 *
 * 이 Loader는 asset 숫자의 물리 단위를 임의로 바꾸지 않는다.
 * 현재 controller-ready HCR GLB가 meter로 정규화되어 있으므로 그 파일의 translation/vertex 값이 [m]인 것은 asset contract 때문이다.
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
