# HCR-12A + Robotiq 2F-85 시뮬레이션 사양 및 가동범위 정리

> 기준일: 2026-09-17  
> 대상 자산: `HCR12A_R00(2).glb` / 정규화된 HCR12A runtime GLB / Robotiq 2F-85 CAD 형상  
> 목적: C++ 로봇 시뮬레이터에서 관절 제한, 속도 제한, TCP/그리퍼 제약, 자산 계층을 일관되게 적용하기 위한 기준 문서

## 1. 결론 — 시뮬레이터에서 우선 적용할 값

- 로봇: Hanwha Robotics HCR-12A, 6-DOF, reach 1300 mm, payload 12 kg. [H1]
- 관절 제한: 현재 제품 페이지와 2025 HCR 2G 사용자 매뉴얼 값이 J2/J3에서 다르다. 현재 GLB가 HCR-12A 2G CAD 계열이므로 **기본 시뮬레이터 limit은 2025 사용자 매뉴얼의 비대칭 범위**를 사용하고, 제품 페이지 값은 별도 revision 값으로 기록한다. [H1][H3]
- 관절 최대속도: J1/J2 130°/s, J3~J6 200°/s. [H1][H3]
- 그리퍼: Robotiq 2F-85, 최대 opening 85 mm, grip force 20~235 N, finger speed 20~150 mm/s, nominal payload 5 kg. [R1]
- HCR tool flange와 2F-85용 GRP-CPL-062 coupling은 모두 **ISO 9409-1-50-4-M6** 규격과 맞는다. [H3][R1]
- 2F-85는 독립적인 추가 wrist 회전축이 아니다. **그리퍼 전체의 tool-axis 회전은 HCR J6가 담당**하고, 2F-85의 “rotational design”은 손가락 링크 메커니즘을 의미한다. [R4]
- 현재 GLB의 `LeftFingerJoint`/`RightFingerJoint`는 실제 Robotiq linkage pivot을 완전하게 표현하지 않는다. 현실적인 finger motion이 필요하면 ROS-Industrial의 linkage/mimic 구조를 참고해 모델을 다시 분할해야 한다. [S1]

## 2. HCR-12A 기본 사양

| 항목 | 값 | 비고 |
|---|---:|---|
| 자유도 | 6 DOF | 6개의 회전 관절 |
| 정격 payload | 12 kg | Tool + coupling + object를 함께 고려해야 함 |
| Reach / work radius | 1300 mm | base 기준 최대 작업반경 |
| 로봇 본체 중량 | 약 52.5~53 kg | 매뉴얼 52.5 kg / 현행 웹 53 kg |
| 반복정밀도 | ±0.07~0.1 mm | 현행 웹 ±0.07 mm / 2025 매뉴얼 ±0.1 mm |
| Linear speed | 1 m/s | 현행 제품 페이지 표기 |
| Footprint | 220 × 227 mm | 현행 제품 페이지/매뉴얼 |
| 설치 | Floor / Wall(max 30°) / Ceiling | 현행 제품 페이지 |
| Noise | ≤ 70 dBA | 현행 제품 페이지 |

### 2.1 관절 가동범위와 최대속도 — 권장 시뮬레이터 값

| Joint | 2025 2G manual limit | rad limit | Max speed | rad/s | 현행 제품 페이지 |
|---|---:|---:|---:|---:|---:|
| J1 | -180° ~ 180° | -3.141593 ~ 3.141593 | 130°/s | 2.268928 | ±180° |
| J2 | -165° ~ 135° | -2.879793 ~ 2.356194 | 130°/s | 2.268928 | ±150° |
| J3 | -85° ~ 245° | -1.483530 ~ 4.276057 | 200°/s | 3.490659 | ±165° |
| J4 | -190° ~ 190° | -3.316126 ~ 3.316126 | 200°/s | 3.490659 | ±190° |
| J5 | -170° ~ 170° | -2.967060 ~ 2.967060 | 200°/s | 3.490659 | ±170° |
| J6 | -360° ~ 360° | -6.283185 ~ 6.283185 | 200°/s | 3.490659 | ±360° |

**Revision 차이:** [H1]의 현재 제품 페이지는 J2=±150°, J3=±165°로 대칭 범위를 표시하지만, [H3]의 HCR-A(2G) User Manual v2.2는 J2=-165°~+135°, J3=-85°~+245°로 제시한다. 시뮬레이터에서 특정 실기기와 1:1 대응해야 한다면 해당 기기의 controller/manual revision을 확인해야 한다.

### 2.2 작업공간과 payload

- 최대 reach는 1300 mm이다. [H1][H3]
- base 중심축 주변에는 구조적으로 도달할 수 없는 내부 workspace 영역이 존재한다. [H3]
- 12 kg은 로봇팔의 정격 payload이며, tool flange에 장착되는 gripper/coupling의 질량과 payload CoG 거리까지 포함해 판단해야 한다. [H3]
- 실제 grasp 가능한 물체 질량은 로봇팔 12 kg보다 2F-85의 nominal 5 kg rating이 먼저 제한이 될 가능성이 높다. Robotiq 역시 마찰계수, 가속도, safety factor에 따라 허용 payload가 감소한다고 명시한다. [R1]

## 3. HCR-12A Tool flange / I/O / 환경조건

### 3.1 Tool flange

- HCR-12A tool flange: ISO 9409-1-50-4-M6. [H3]
- Tool 체결: 4 × M6. [H3]
- 2F-85 GRP-CPL-062 coupling도 ISO 9409-1-50-4-M6 지원. [R1]
- 따라서 기계 규격 관점에서 HCR-12A ↔ 2F-85 coupling 조합은 직접 호환되는 규격이다.

### 3.2 Tool I/O

| 항목 | HCR-12A |
|---|---|
| Tool I/O #1 | M8 8-pin, 24 V, DI 2ch, DO 2ch, Analog In 2ch |
| Tool I/O #2 | M8 8-pin, 24 V, DI 2ch, DO 2ch, RS-485 2ch |
| EtherCAT | M8 6-pin |
| Tool power | Max 3 A total, 1.5 A/pin (current product page) |
| External communication | TCP/IP, Modbus TCP |

2F-85는 기본적으로 Modbus RTU(RS-485) 기반 제어를 사용하므로, 실제 하드웨어 연동까지 확장할 경우 HCR의 tool-side RS-485/전원 구성과 함께 검토할 수 있다. [R3]

### 3.3 환경/설치 조건

| 항목 | 값 |
|---|---:|
| Operating temperature | 0~45 °C |
| Operating humidity | 20~80% RH, non-condensing |
| Robot arm IP | IP54 (manual / detailed spec) |
| Controller IP | IP20 |
| Teach pendant IP | IP20 |

> 주의: 현재 한화 제품 페이지 상단 인증 영역에는 IP66 아이콘이 표시되지만, 같은 페이지의 상세 installation spec과 2G 매뉴얼은 Robot Arm IP54를 제시한다. 특정 하드웨어 revision의 보호등급은 납품 사양서를 우선 확인해야 한다. [H1][H3]

## 4. Robotiq 2F-85 기본 사양

| 항목 | 2F-85 |
|---|---:|
| Gripper opening / stroke | 85 mm |
| Minimum object diameter (encompassing) | 43 mm |
| Maximum height | 162.8 mm |
| Maximum width | 148.6 mm |
| Mass | 925 g (제품 자료에서는 약 0.9 kg로 반올림) |
| Grasp force | 20~235 N |
| Finger speed | 20~150 mm/s |
| Position repeatability | 0.05 mm |
| Force repeatability | ±10% |
| Position resolution | 0.4 mm |
| Friction grasp payload | 5 kg |
| Form-fit grasp payload | 5 kg |
| IP rating | IP40 |
| Operating temperature | -10~50 °C |
| Operating humidity | 20~80% RH, non-condensing |

### 4.1 외력/모멘트 제한

| Parameter | Limit (2F-85) |
|---|---:|
| Fx, Fy, Fz | 50 N |
| Mx | 5 N·m |
| My | 5 N·m |
| Mz | 3 N·m |

이 값은 gripper 자체의 grasp force와 별개이며, robot acceleration과 safety factor를 포함해 계산해야 한다. [R1]

### 4.2 전기 사양

| 항목 | 값 |
|---|---:|
| Supply | 24 V DC ±10% |
| Absolute max | 28 V DC |
| Quiescent power | < 1 W |
| Peak current | 1 A |

## 5. 2F-85 제어값과 시뮬레이션 상태 모델

Robotiq controller는 실제 linkage를 직접 각도 명령으로 노출하지 않고 position/speed/force의 고수준 register로 추상화한다. [R3]

| Register/State | 의미 | 범위 |
|---|---|---:|
| rPR | Target position | 0=open, 255=closed |
| rSP | Speed | 0=min, 255=max |
| rFR | Force | 0=min, 255=max |
| gPO | Encoder-based actual position | 0=open, 255=closed |
| gOBJ | Object detection | 0=moving, 1=contact opening, 2=contact closing, 3=target/no object |

- 2F-85 position request는 0에서 85 mm open, 255에서 mechanical closed stop이다. [R3]
- 문서상 opening/count는 약 0.4 mm이며 0~255 관계는 quasi-linear로 설명된다. [R3]
- Force request가 0이면 lowest-force mode 및 re-grasp off, 1~127 low-torque, 128~255 high-torque 영역으로 설명된다. [R3]
- 실제 물체 접촉은 `gOBJ`와 실제 finger position을 함께 확인하는 것이 더 안정적이라고 Robotiq가 권장한다. [R3]

## 6. 2F-85의 실제 손가락 운동과 현재 GLB의 차이

### 6.1 실제 제품

2F-85는 **rotational adaptive gripper**다. 두 손가락이 단순 prismatic slider처럼 평행이동하는 구조가 아니며, 여러 link/knuckle가 연동되어 parallel grasp와 encompassing grasp를 자동으로 만든다. [R4]

ROS-Industrial C3 visualization model에서는 시뮬레이션을 위해 다음과 같이 모델링한다. 이 값은 **제조사 공식 operating range가 아니라 URDF simulation reference**다. [S1]

| URDF joint | Range / relation |
|---|---|
| `finger_joint` | 0 ~ 0.8 rad |
| right outer knuckle | 약 0 ~ 0.81 rad, main joint mimic |
| inner knuckle | 0 ~ 0.8757 rad, main joint mimic |
| inner finger | 0 ~ 0.8757 rad, 반대 부호 mimic |

### 6.2 현재 HCR12A GLB 자산

원본 `HCR12A_R00(2).glb`에서 2F-85는 `2F85 → GripperLink → LeftFingerJoint / RightFingerJoint`로 그룹이 만들어져 있지만, 두 FingerJoint 노드 자체의 local transform은 identity이다. 개별 finger CAD 부품의 위치/회전이 child mesh에 직접 들어가 있으므로 **현재 FingerJoint는 실제 힌지 pivot을 나타내는 관절 프레임이 아니다.**

따라서 다음 두 방법 중 하나를 선택해야 한다.

1. **프로젝트 단순 모델:** `opening_mm ∈ [0, 85]`를 상태값으로 사용하고, 시각화는 두 finger group에 경험적으로 보정한 단일 회전을 적용한다. grasp 판정은 실제 각도보다 fingertip 간 거리로 판단한다.
2. **기구학 재현 모델:** gripper mesh를 outer knuckle / outer finger / inner knuckle / inner finger / pad 단위로 다시 분리하고 ROS-Industrial mimic 관계를 구현한다. 실제 Robotiq motion에 더 가깝다.

현재 프로젝트 목적이 6-DOF arm IK + grasp/attach라면 1번으로 시작하고, gripper linkage 자체가 평가 대상일 때 2번으로 확장하는 편이 적절하다.

## 7. HCR-12A + 2F-85 통합 시뮬레이션 규칙

### 7.1 자유도 정의

- Robot arm DOF = 6 (J1~J6).
- Gripper opening = 별도 actuator state 1개로 관리하되, **로봇 pose DOF에는 포함하지 않는다.**
- Tool-axis 회전 = J6. 2F-85가 별도 7번째 rotary wrist를 제공하는 것은 아니다.

### 7.2 payload

2F-85 자체 질량은 약 0.925 kg이고 coupling 질량은 별도로 더해진다. HCR의 12 kg payload는 tool assembly 전체를 포함하므로 robot payload 계산에는 gripper/coupling/object를 합산해야 한다. 반면 2F-85의 nominal grasp payload는 5 kg이므로, 정적 이상조건에서도 object payload는 일반적으로 5 kg 이하에서 먼저 제한된다. 실제 허용량은 마찰계수와 로봇 가속도로 더 낮아진다. [R1]

### 7.3 속도 제한

- 각 joint command는 위의 max joint speed를 넘지 않도록 clamp한다.
- Cartesian EE speed는 기본적으로 1 m/s를 상한으로 둔다. [H1]
- Gripper finger speed는 20~150 mm/s 범위를 사용한다. [R1]

### 7.4 limit 처리

- IK 해를 적용하기 전 joint-space limit 검사.
- limit 밖의 해는 reject 또는 nearest feasible solution으로 재탐색.
- J6는 ±360° 범위를 갖지만 continuous joint로 무한 회전하는 것으로 처리하지 않는다. cable/tool 조건을 고려해 문서 범위 내 clamp하는 것이 안전하다.
- J2/J3 revision mismatch는 설정파일로 분리해 `manual_2025`와 `current_web` profile을 선택할 수 있게 하는 것을 권장한다.

## 8. 권장 C++ 상수

```cpp
struct JointLimit {
    double minRad;
    double maxRad;
    double maxVelRadSec;
};

constexpr JointLimit HCR12A_LIMITS[6] = {
    { -3.141593,  3.141593,  2.268928 }, // J1
    { -2.879793,  2.356194,  2.268928 }, // J2
    { -1.483530,  4.276057,  3.490659 }, // J3
    { -3.316126,  3.316126,  3.490659 }, // J4
    { -2.967060,  2.967060,  3.490659 }, // J5
    { -6.283185,  6.283185,  3.490659 }, // J6
};

constexpr double HCR12A_REACH_M = 1.300;
constexpr double HCR12A_PAYLOAD_KG = 12.0;
constexpr double HCR12A_MAX_TCP_SPEED_MPS = 1.0;

constexpr double GRIPPER_2F85_MAX_OPENING_M = 0.085;
constexpr double GRIPPER_2F85_MIN_SPEED_MPS = 0.020;
constexpr double GRIPPER_2F85_MAX_SPEED_MPS = 0.150;
constexpr double GRIPPER_2F85_MIN_FORCE_N = 20.0;
constexpr double GRIPPER_2F85_MAX_FORCE_N = 235.0;
constexpr double GRIPPER_2F85_MASS_KG = 0.925;
constexpr double GRIPPER_2F85_NOMINAL_PAYLOAD_KG = 5.0;
```

## 9. 현재 GLB에서 확인된 runtime node 기준

정규화된 모델 기준 제어 대상은 다음과 같다.

```text
RobotRoot
└─ Base
   └─ J1 → Link1
      └─ J2 → Link2
         └─ J3 → Link3
            └─ J4 → Link4
               └─ J5 → Link5
                  └─ J6 → Link6
                     └─ Gripper
                        ├─ LeftFingerJoint
                        └─ RightFingerJoint
```

GLB에서 추출된 주요 bind translation은 다음과 같다. 이 값은 **asset local coordinate 값**이며 DH parameter나 제조사 공식 link length로 해석하면 안 된다.

| Node | Local translation (m) | 추가 bind rotation |
|---|---|---|
| J1 | (0, 0.310, 0.100) | quat xyzw ≈ (0.7071, 0, 0, -0.7071) |
| J2 | (-0.001, 0, 0) | identity |
| J3 | (0.001, 0, 0.705) | identity |
| J4 | (0, 0.100, -0.705) | identity |
| J5 | (0, -0.854, 0.705) | identity |
| J6 | (0, -0.0015, 0) | quat xyzw = (0.5, 0.5, -0.5, 0.5) |
| EndEffector (원본) | (0, 0, 0.059) | identity |

정규화된 runtime GLB에서는 `EndEffector → GripperMount → 2F85` 고정변환을 `Gripper` node에 합쳐 두었다. 관절 동작 구현 시 bind transform을 덮어쓰지 말고 `bind * joint_delta` 형태로 유지해야 한다.

## 10. 구현 체크리스트

- [ ] J1~J6 angle limit clamp 적용
- [ ] Joint max velocity clamp 적용
- [ ] J2/J3 limit profile revision 분리
- [ ] FK 결과 TCP reach sanity check (1.3 m)
- [ ] Gripper opening state 0~85 mm 관리
- [ ] Gripper velocity 20~150 mm/s 범위 관리
- [ ] Object attach 조건에서 fingertip distance + collision/contact 사용
- [ ] Gripper payload 5 kg 및 arm residual payload 검증
- [ ] J6 회전과 gripper opening을 서로 다른 state로 분리
- [ ] 현재 FingerJoint pivot 한계를 문서화하고 linkage fidelity가 필요할 때 모델 재분할

## 11. 출처

- **[H1] Hanwha Robotics — HCR-12A current product page**  
  https://www.hanwharobotics.com/kr/product/view?menuSeq=2&prdtSeq=79
- **[H2] Hanwha Robotics — Download page (HCR 2세대 사용자 매뉴얼/2D·3D 도면 공식 배포 위치)**  
  https://www.hanwharobotics.com/kr/newsroom/download?menuSeq=85
- **[H3] HCR-A (2G) Series User Manual Ver. 2.2, Feb. 2025 — readable mirror used to cross-check manual values**  
  https://www.scribd.com/document/1077603468/Hcr-2g-Userguide-en-v2-2-1
- **[R1] Robotiq — 2F-85/2F-140 General Instruction Manual, Specifications (HTML)**  
  https://assets.robotiq.com/website-assets/support_documents/document/online/2F-85_2F-140_Instruction_Manual_Gen_HTML_20190524.zip/2F-85_2F-140_Instruction_Manual_Gen_HTML/Content/6.%20Specifications.htm
- **[R2] Robotiq — 2F-85/2F-140 General Instruction Manual, Installation / environmental conditions (HTML)**  
  https://assets.robotiq.com/website-assets/support_documents/document/online/2F-85_2F-140_Instruction_Manual_Gen_HTML_20190524.zip/2F-85_2F-140_Instruction_Manual_Gen_HTML/Content/3.%20Installation.htm
- **[R3] Robotiq — 2F-85/2F-140 control/register mapping (HTML)**  
  https://assets.robotiq.com/website-assets/support_documents/document/online/2F-85_2F-140_TM_InstructionManual_HTML5_20190503.zip/2F-85_2F-140_TM_InstructionManual_HTML5/Content/4.%20Control.htm
- **[R4] Robotiq — Adaptive Gripper product guidance (current website)**  
  https://blog.robotiq.com/knowledge/what-is-the-best-adaptive-gripper-for-your-application-5-1736280751960
- **[S1] ROS-Industrial Robotiq C3 model — 2F-85 Xacro, joint/mimic ranges (simulation reference; not manufacturer operating spec)**  
  https://github.com/ros-industrial-attic/robotiq/blob/kinetic-devel/robotiq_2f_85_gripper_visualization/urdf/robotiq_arg2f_85_model_macro.xacro

### 출처 해석 원칙

- 제조사 현행 제품 페이지와 사용자 매뉴얼 값이 충돌하는 경우, 문서에서 값을 합쳐 하나로 단정하지 않고 revision 차이로 분리했다.
- Robotiq의 joint angle 값은 제조사 manual이 아니라 ROS-Industrial visualization model의 simulation reference이므로 공식 mechanical limit로 취급하지 않는다.
- GLB transform 값은 업로드된/정규화된 asset에서 추출한 값이며 제조사 DH parameter가 아니다.