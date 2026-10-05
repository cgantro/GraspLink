#pragma once

#include "assets/GraphicsTypes.h"

#include <filesystem>

/**
 * @brief GLB 이진 장면을 렌더러가 사용할 CPU 데이터로 변환한다.
 * @details
 * TinyGLTF가 GLB의 JSON/BIN 청크와 포함 이미지를 읽고, 이 클래스는 Accessor를 따라
 * BufferView의 byteOffset과 byteStride를 적용해 정점·인덱스를 복사한다. 위치·법선은
 * FLOAT VEC3, UV는 FLOAT VEC2, tangent는 FLOAT VEC4의 xyz만 받는다. 속성 Accessor의
 * 정규화 정수 표현, Sparse Accessor, 삼각형 이외 Primitive는 지원하지 않는다.
 * 이미지 픽셀과 재질·메시 ID는 CPU 데이터로 반환하며 GPU 업로드는 AssetManager 책임이다.
 * 재질은 base color factor/texture, metallic·roughness factor, emissive factor를 읽는다.
 * 선택 Scene은 단일 root 아래의 트리여야 한다. Mesh가 없는 Node도 Local 변환과 계층 보존을
 * 위해 결과에 남는다.
 */
class GltfLoader final
{
public:
    /**
     * @brief GLB 파일을 읽어 메시·재질·이미지·Node 계층을 CPU 자료구조로 만든다.
     * @param path 읽을 GLB 파일 경로.
     * @return 파일 경로를 포함한 ResourceID와 CPU 배열, 선택 Scene root를 담은 ModelResource.
     * @throws std::runtime_error 파일 해석 실패, 잘못된 범위, 비유한 값 또는 지원하지 않는
     * GLTF 표현을 만나면 발생한다. 자세한 원인은 예외 메시지에 담긴다.
     * @throws std::out_of_range 잘못된 Accessor 번호가 내부 벡터 조회에 전달되면 발생한다.
     * @details
     * 정점 데이터는 Accessor의 byteOffset에서 시작해 byteStride 간격으로 복사하며,
     * 접근자 전체가 해당 BufferView와 버퍼 안에 드는지 확인한다. byte는 정렬되지 않은
     * float 주소를 역참조하지 않도록 memcpy로 복사하지만, glTF 정렬 규칙을 별도 검사하지는 않는다.
     * 서로 다른 Primitive는 Mesh 배열에 이어 붙이고 로컬 index에 새 정점 시작 위치를 더한다.
     * index가 생략되면
     * 정점 순서 0..N-1을 사용한다. 재질 index는 Primitive 범위 정보에 보존된다.
     * glTF matrix는 Local translation/rotation/scale로 분해한다. shear와 perspective는
     * 내부 TRS에 담을 수 없어 거부한다. 회전은 matrix 분해와 glTF TRS 입력 모두 단위 quaternion으로
     * 보관한다. glTF 입력 순서 [x,y,z,w]를 GLM 생성자 순서 (w,x,y,z)에 맞춰 옮긴다.
     * 반환 시 GPU 자원은 생성되지 않는다.
     */
    static ModelResource LoadGLB(const std::filesystem::path& path);

private:
    GltfLoader() = delete;
};
