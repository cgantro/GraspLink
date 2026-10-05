
/**
 * @file ShadowDepth.glsl
 * @brief 광원 카메라로 그린 지오메트리의 깊이를 shadow map에 기록한다.
 * @details
 * 정점 셰이더는 정점의 로컬 좌표에 모델 변환을 적용한 뒤 광원 시점 행렬을 곱해
 * 동차 clip 좌표를 만든다. 래스터라이저는 이를 clip 및 perspective divide를 거쳐
 * 화면 좌표와 깊이로 변환하고, fragment 셰이더가 별도 색 출력을 내지 않아도
 * 활성화된 depth test와 depth attachment에 깊이를 기록한다. 후속 조명 셰이더는
 * 같은 광원 공간 변환 결과에서 얻은 깊이를 이 값과 비교해 가림을 판정한다.
 */
#type vertex
#version 330 core

layout(location = 0) in vec3 a_Position;

uniform mat4 u_Model;
uniform mat4 u_LightSpaceMatrix;

void main()
{

    // World 좌표를 광원 clip 공간으로 옮겨 depth map을 만든다.
    gl_Position =
        u_LightSpaceMatrix *
        u_Model *
        vec4(a_Position, 1.0);
}

#type fragment
#version 330 core

void main()
{
    // 색 출력 없이 rasterizer가 깊이 attachment에 표면 깊이를 기록한다.
}
