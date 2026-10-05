#pragma once

/**
 * @brief Viewer Entity에서 쓰는 ECS Component 헤더를 한 번에 포함한다.
 * @details
 * MeshFilter와 MeshRenderer 같은 렌더 데이터, Transform 계열 데이터를 함께 선언하는 집합 헤더다.
 * Component 동작은 각 타입과 시스템에 있고 이 헤더 자체는 ECS 저장이나 렌더링을 수행하지 않는다.
 */
#include "RenderComponents.h"
#include "TransformComponents.h"
