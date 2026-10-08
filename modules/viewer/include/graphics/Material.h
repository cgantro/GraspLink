#pragma once

#include <glm/glm.hpp>
#include <memory>

namespace grasplink::graphics
{

class Texture;

/**
 * @brief 표면에 칠할 기본색과 빛 반사 모양을 정하는 값을 보관한다.
 * @details
 * 재질은 Mesh 표면에 칠할 색과 빛 반응을 정하는 자료다. AssetManager가 파일 값으로 재질을 만들고 Renderer가 GPU Shader에 전달한다.
 * 기본색은 RGBA 배율과 선택적 Texture(표면 이미지)를 곱해 정한다. Renderer는 금속성·거칠기 계수만 전달하고 그 계수 이미지와 발광 계수·이미지는 화면 계산에 사용하지 않는다.
 * Robot.glsl은 금속성·거칠기 숫자로 간단한 반사 근사를 계산하며 glTF의 전체 물리 기반 빛 반사식은 구현하지 않는다.
 * GPU 이미지는 여러 재질이 공유할 수 있다. 마지막 공유 참조가 사라질 때 GPU에서 삭제되므로 GPU 명령 실행 환경(context)이 그때까지 현재 스레드에서 활성화되어야 한다.
 */
class Material final
{
public:
    /**
     * @brief 표면 기본색과 반사 특성을 정한다.
     * @param baseColorFactor 표면색·투명도 배율. RGB는 빛 계산용 선형 색이고 alpha는 투명도 값이다.
     * @param metallicFactor 금속처럼 반사하는 정도 [0..1], 단위 없음.
     * @param roughnessFactor 반사 경계가 흐려지는 정도 [0..1], 단위 없음.
     * @details 이 생성자는 값을 그대로 보관한다. Robot.glsl에서 금속성은 0~1, 거칠기는 0.05~1로 제한해 조명 계산에 쓴다.
     */
    Material(const glm::vec4& baseColorFactor, float metallicFactor, float roughnessFactor);

    const glm::vec4& BaseColorFactor() const;

    float MetallicFactor() const;

    float RoughnessFactor() const;

    /**
     * @brief 기본색을 읽을 GPU 이미지를 재질과 공유한다.
     * @param texture 연결할 이미지. nullptr이면 현재 연결을 제거한다.
     * @details 이미지의 마지막 공유 참조가 사라질 때 GPU에서 삭제한다. 그때 OpenGL 실행 환경(context)이 현재 스레드에서 활성화되어야 한다.
     */
    void SetBaseColorTexture(const std::shared_ptr<Texture>& texture);

    std::shared_ptr<Texture> GetBaseColorTexture() const;

    bool HasBaseColorTexture() const;

private:
    // 기본값 (1,1,1,1)을 texture 색과 곱해도 색과 alpha가 바뀌지 않는다.
    glm::vec4 m_BaseColorFactor{1.0F};

    // 0은 비금속, 1은 금속이다. Renderer가 Robot.glsl에 전달해 반사색 계산에 사용한다.
    float m_MetallicFactor = 1.0F;

    // 0에 가까우면 작은 밝은 반사, 1에 가까우면 넓고 흐린 반사가 된다. Robot.glsl은 0.05보다 작은 값을 막는다.
    float m_RoughnessFactor = 1.0F;

    // GPU 이미지의 공유 소유권이다. 마지막 참조가 사라질 때 Texture가 GL ID를 지우므로 그때까지 context가 필요하다.
    std::shared_ptr<Texture> m_BaseColorTexture;
};

} // namespace grasplink::graphics
