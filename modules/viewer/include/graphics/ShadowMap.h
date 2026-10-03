#pragma once

#include <cstdint>

/**
 * @brief Directional light shadow mapping용 depth texture와 framebuffer를 관리한다.
 *
 * @details
 * Shadow map은 light 관점에서 scene의 가장 가까운 depth를 texture에 기록한다.
 * Main pass에서는 현재 fragment의 light-space depth와 저장된 depth를 비교해 가려졌는지 판단한다.
 * Begin/End는 shadow pass 동안 바뀌는 framebuffer와 viewport를 저장/복원한다.
 *
 * @todo [FUTURE] shadow resolution과 polygon offset 값을 graphics settings로 이동한다.
 * @todo [FUTURE] 넓은 작업공간이 필요해지면 cascaded shadow map 또는 dynamic light frustum을 검토한다.
 */
class ShadowMap final
{
public:
    /** @brief 정사각형 depth map을 생성한다. */
    explicit ShadowMap(int size = 2048);

    /** @brief depth texture와 framebuffer를 해제한다. */
    ~ShadowMap();

    ShadowMap(const ShadowMap&) = delete;
    ShadowMap& operator=(const ShadowMap&) = delete;

    /** @brief shadow framebuffer/viewport를 활성화하고 depth buffer를 초기화한다. */
    void Begin();

    /** @brief 이전 framebuffer/viewport 상태를 복원한다. */
    void End();

    /** @brief 생성된 depth texture를 지정 texture unit에 바인딩한다. */
    void Bind(std::uint32_t slot) const;

private:
    std::uint32_t m_Framebuffer = 0;
    std::uint32_t m_DepthTexture = 0;
    int m_Size = 2048;

    int m_PreviousFramebuffer = 0;
    int m_PreviousViewport[4]{};
};
