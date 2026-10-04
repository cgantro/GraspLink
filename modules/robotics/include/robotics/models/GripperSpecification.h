#pragma once

#include "robotics/models/ModelTypes.h"

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace grasplink::robotics::models
{

/**
 * @file GripperSpecification.h
 * @brief Gripper의 고정된 기구학/명령 규격을 표현하는 공통 타입.
 *
 * @details
 * 로보틱스 용어를 모르는 경우 아래처럼 읽으면 된다.
 *
 * - Joint(관절): 두 부품 사이에서 회전이 일어나는 축/힌지.
 * - Link(링크): Joint와 Joint 사이의 단단한 부품.
 * - Actuator(구동기): 실제 움직임을 만드는 모터/감속기 같은 구동원.
 * - Linkage(링키지): 여러 Link와 Joint가 기계적으로 연결되어 함께 움직이는 구조.
 * - Master joint: 여러 관절 움직임을 계산할 때 기준으로 삼는 대표 관절.
 * - Mimic joint: 독립 명령을 받지 않고 master joint의 움직임을 일정 비율/부호로 따라가는 관절.
 * - Pivot(피벗): 관절이 실제로 회전하는 중심점.
 * - Axis(축): 어떤 방향의 선을 중심으로 회전하는지를 나타내는 방향벡터.
 * - Bind pose: GLB를 처음 불러왔을 때의 기준 자세. 여기의 pivot/축은 그 기준 자세를 기준으로 한다.
 * - Free-space: 손가락이 물체와 접촉하지 않고 공중에서 열리고 닫히는 상태.
 * - Under-actuated: 관절 수보다 구동기 수가 적어, 접촉 후 일부 관절이 수동적으로 적응하는 구조.
 *
 * Robotiq 2F-85처럼 하나의 모터/구동 명령으로 여러 손가락 관절이 연동되는 그리퍼는
 * 각 관절을 따로 제어하기보다 master angle q 하나에서 나머지 각도를 파생시키는 편이 자연스럽다.
 */

/**
 * @brief 그리퍼 안의 관절 하나가 master 관절을 어떻게 따라 움직이는지 정의하는 정적 규격.
 *
 * @details
 * 이 구조체는 "현재 관절각"을 저장하는 상태값이 아니다.
 * 모델이 원래 어떤 이름/회전중심/회전축/허용범위를 갖고, master 움직임을 어떤 관계로 따라가는지 정의한다.
 *
 * 자유공간에서는 보통 다음 식으로 목표각을 계산한다.
 *
 * `jointAngle = masterAngle * masterMultiplier`
 *
 * 예를 들어 masterAngle=0.4 rad일 때 multiplier가 +1이면 +0.4 rad,
 * -1이면 -0.4 rad를 목표각으로 사용한다.
 */
struct GripperJointSpecification
{
    /**
     * @brief 이 관절의 논리 이름.
     * @details Viewer에서는 같은 이름의 GLB/Flecs Joint Node를 찾아 실제 화면상의 관절과 연결한다.
     */
    std::string_view name;

    /**
     * @brief Bind pose에서의 회전 중심점 [x,y,z], 단위 meter [m].
     * @details Gripper root 좌표계를 기준으로 한 값이다. Mesh 중심이 아니라 실제 힌지/관절 중심을 뜻한다.
     */
    Vec3 bindPivotMeters;

    /**
     * @brief 이 관절이 회전하는 local 축.
     * @details 예: {0,0,-1}이면 해당 Joint의 local -Z축을 중심으로 회전한다. 방향값이므로 물리 단위는 없다.
     */
    Axis3 axis;

    /**
     * @brief Master angle q에 곱하는 mimic 계수.
     *
     * @details
     * +1.0: master와 같은 방향/같은 크기로 회전.
     * -1.0: master와 반대 방향/같은 크기로 회전.
     *  0.5: master의 절반 크기로 회전하는 식의 일반화도 가능하다.
     *
     * 여기서 mimic은 "관절이 독립 명령을 받지 않고 다른 관절을 따라간다"는 뜻이다.
     */
    double masterMultiplier = 1.0;

    /** @brief 이 관절이 허용하는 최소 회전각 [rad]. 0 rad는 모델이 정의한 기준자세의 0도 상태다. */
    double minPositionRadians = 0.0;

    /** @brief 이 관절이 허용하는 최대 회전각 [rad]. */
    double maxPositionRadians = 0.0;

    /**
     * @brief true이면 이 관절을 자유공간 기구학 계산의 대표 master joint로 취급한다.
     * @note 실제 제품에 이 이름의 독립 모터가 하나 더 있다는 뜻이 아니다. 시뮬레이션 계산 기준을 표시하는 플래그다.
     */
    bool actuatorMaster = false;
};

/**
 * @brief 특정 gripper model의 명령 범위와 모든 linkage joint 규격을 한 묶음으로 제공하는 모델 정의.
 *
 * @details
 * Controller가 "Robotiq 2F-85이면 255가 닫힘이고 관절은 6개다" 같은 모델별 숫자를 하드코딩하지 않도록
 * 제조사/모델명, raw command 범위, master 기준각, 관절 배열을 한 곳에 모은다.
 *
 * `non-owning view`는 이 구조체가 joints 배열을 직접 소유/복사하지 않고 주소만 가리킨다는 뜻이다.
 * 따라서 `joints`가 가리키는 constexpr/static 배열이 이 구조체보다 오래 살아 있어야 한다.
 */
struct GripperSpecification
{
    /** @brief 제조사 표시 이름. 예: "Robotiq". */
    std::string_view manufacturer;

    /** @brief 제품 모델명. 예: "2F-85". */
    std::string_view model;

    /** @brief 위치 명령의 최소 raw 값. raw 값은 mm/rad가 아니라 장치 프로토콜상의 정수 명령값이다. */
    std::uint8_t positionRequestMin = 0;

    /** @brief 위치 명령의 최대 raw 값. 2F-85에서는 255가 fully closed 쪽이다. */
    std::uint8_t positionRequestMax = 255;

    /** @brief 속도 명령의 최소 raw 값. 실제 mm/s 값으로 직접 해석하지 않는다. */
    std::uint8_t speedRequestMin = 0;

    /** @brief 속도 명령의 최대 raw 값. */
    std::uint8_t speedRequestMax = 255;

    /** @brief 힘 명령의 최소 raw 값. 실제 Newton[N] 값 자체가 아니다. */
    std::uint8_t forceRequestMin = 0;

    /** @brief 힘 명령의 최대 raw 값. */
    std::uint8_t forceRequestMax = 255;

    /**
     * @brief Free-space에서 완전히 닫힌 자세를 표현할 때 사용하는 master linkage 기준각 [rad].
     * @note 실제 내부 motor shaft angle이 아니라 시뮬레이션 기구학의 대표 joint angle이다.
     */
    double nominalMasterClosedRadians = 0.0;

    /** @brief 모델에 포함된 GripperJointSpecification 배열의 첫 원소 주소. 소유권은 없다. */
    const GripperJointSpecification* joints = nullptr;

    /** @brief joints 배열의 원소 개수. 즉 시뮬레이션에서 다루는 linkage joint 수. */
    std::size_t jointCount = 0;
};

} // namespace grasplink::robotics::models
