#pragma once

#include <cstddef>
#include <memory>

namespace grasplink::graphics
{
class Mesh;
class Shader;
class Material;
}

namespace grasplink::rendering
{

/**
 * @brief 장면 물체가 화면에 그릴 3D 모양과 그중 사용할 삼각형 범위를 지정한다.
 * @details
 * Mesh는 표면 모양을 이루는 꼭짓점과 삼각형 자료다. GLB 모델 하나의 여러 부분이 같은 Mesh를 재사용할 수 있다.
 * RenderSystem은 이 선택과 물체의 변환 행렬을 함께 읽어 Renderer에 넘긴다. indexOffset은 시작할 index 원소 번호이며 byte 수가 아니다.
 * shared_ptr은 AssetManager의 이름 검색 목록과 별개로 Mesh 사용 수명을 공유한다. 이 물체가 참조를 보유하면 검색 목록을 비워도 Mesh가 남는다.
 * 마지막 참조를 해제할 때 GPU 메모리를 지우므로 OpenGL Context가 살아 있어야 한다.
 */
struct MeshFilter
{
    /// AssetManager가 GPU에 올린 표면 모양의 공유 참조다. 참조가 비어 있으면 RenderSystem은 이 물체를 그리지 않는다.
    std::shared_ptr<::grasplink::graphics::Mesh> mesh;

    /// 삼각형 목록에서 그리기를 시작할 uint32 index 번호다. byte 위치로 직접 지정하지 않는다.
    std::size_t indexOffset = 0U;

    /// 사용할 uint32 index 개수다. 0이면 Mesh 전체를 그리며 이때 시작 번호도 0이어야 한다.
    std::size_t indexCount = 0U;
};

/**
 * @brief 장면 물체의 표면 색을 계산할 프로그램과 화면에 그릴지 여부를 지정한다.
 * @details
 * Shader는 GPU가 꼭짓점과 표면 색을 계산하는 프로그램이고 Material은 색과 Texture 같은 표면 설정이다.
 * RenderSystem은 카메라에서 보이는 색을 만들 때 이 설정을 읽는다. visible이 false이면 Mesh를 그리지 않는다.
 * 이 타입은 설정만 저장하며 실제 선택과 그리기는 RenderSystem과 Renderer가 수행한다.
 * AssetManager에서 가져온 Shader와 Material의 수명도 공유 참조로 이어져 사용 중인 물체가 사라질 때까지 유지된다.
 */
struct MeshRenderer
{
    /// 카메라 화면을 만들 때 꼭짓점 위치와 색을 계산하는 GPU 프로그램의 공유 참조다.
    std::shared_ptr<::grasplink::graphics::Shader> shader;

    /// Renderer가 표면 색과 지원된 Texture를 선택할 때 사용하는 Material 공유 참조다.
    std::shared_ptr<::grasplink::graphics::Material> material;

    /// false이면 화면에 이 Mesh를 그리지 않는다.
    bool visible = true;
};

} // namespace grasplink::rendering
