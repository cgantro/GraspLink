#pragma once

#include <cstdint>

/**
 * @brief Directional-light shadow mapping용 depth texture와 framebuffer를 관리한다.
 *
 * @details
 * Shadow pass에서 light 관점의 가장 가까운 depth를 texture에 기록하고 Main pass에서 현재 fragment의
 * light-space depth와 비교해 shadow 여부를 판단한다. Begin()/End()는 framebuffer와 viewport 상태를
 * 임시 변경했다가 원래 render target으로 복원한다.
 *
 * `size` 단위는 depth texture 한 변의 pixel 수이며 world-space 거리 단위와 무관하다.
 *
 * @todo [FUTURE] shadow resolution과 polygon offset 값을 graphics settings로 이동한다.
 * @todo [FUTURE] 넓은 작업공간이 필요해지면 cascaded shadow map 또는 dynamic light frustum을 검토한다.
 */
class ShadowMap final
{
public:
    /**
     * @brief 정사각형 depth map을 생성한다.
     * @param size texture width/height [pixel]. 기본 2048x2048.
     */
    explicit ShadowMap(int size = 2048);

    /** @brief 소유한 depth texture와 framebuffer object를 해제한다. */
    ~ShadowMap();

    /** @brief OpenGL object ownership 중복을 막기 위해 copy construction을 금지한다. */
    ShadowMap(const ShadowMap&) = delete;

    /** @brief OpenGL object ownership 중복을 막기 위해 copy assignment를 금지한다. */
    ShadowMap& operator=(const ShadowMap&) = delete;

    /**
     * @brief Shadow framebuffer/viewport를 활성화하고 depth buffer를 초기화한다.
     * @note 호출 시점의 framebuffer ID와 viewport를 내부에 저장해 End()에서 복원한다.
     */
    void Begin();

    /** @brief Begin() 이전 framebuffer/viewport 상태를 복원한다. */
    void End();

    /**
     * @brief 생성된 depth texture를 지정 texture unit에 바인딩한다.
     * @param slot OpenGL texture unit index. pixel/byte 단위가 아니다.
     */
    void Bind(std::uint32_t slot) const;

private:
    /** @brief Depth-only shadow pass framebuffer object ID. */
    std::uint32_t m_Framebuffer = 0;

    /** @brief Shadow depth texture object ID. */
    std::uint32_t m_DepthTexture = 0;

    /** @brief Depth texture width/height [pixel]. */
    int m_Size = 2048;

    /** @brief Begin() 이전에 bind되어 있던 framebuffer ID. End() 복원용. */
    int m_PreviousFramebuffer = 0;

    /** @brief Begin() 이전 viewport {x,y,width,height} [pixel]. End() 복원용. */
    int m_PreviousViewport[4]{};
};
