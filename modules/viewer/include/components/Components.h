#pragma once

/**
 * @brief Viewer Entity가 사용하는 렌더링 및 공간 변환 Component 선언을 한 번에 포함한다.
 * @details
 * 이 헤더를 포함하면 MeshFilter와 MeshRenderer 같은 그리기 설정, 위치·회전·크기와 행렬을 담는 변환 타입을 함께 사용할 수 있다.
 * 실제 데이터 저장은 Flecs World가 하고, 값을 계산하거나 그리는 동작은 각 시스템이 수행한다. 이 파일은 타입 선언을 모아 제공할 뿐 저장이나 렌더링을 실행하지 않는다.
 */
#include "RenderComponents.h"
#include "TransformComponents.h"
