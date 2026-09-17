# HCR-12A Simulator Kinematics Contract

## 1. 모델의 의미

Simulator는 Hanwha HCR-12A의 공개 사양인 6DoF, 1.3 m reach와 joint motion range를
따르는 교육용 kinematic model이다. 실제 controller를 구동하거나 HCR-12A safety limit를
대체하지 않는다. CAD STEP 파일의 link-local mesh frame과 각 joint pivot/axis를 확정한 뒤
`RobotSpecification::MakeHcr12aNominal()`의 provisional offset을 교체해야 한다.

| Joint | Software limit |
|---|---:|
| J1 | -180° to +180° |
| J2 | -150° to +150° |
| J3 | -165° to +165° |
| J4 | -190° to +190° |
| J5 | -170° to +170° |
| J6 | -360° to +360° |

모든 길이는 metre, 모든 internal angle은 radian, quaternion field order는 `(w,x,y,z)`다.

## 2. Frame과 FK

`T^A_B`는 B-frame의 point를 A-frame으로 옮기는 rigid transform이다. 관절 i의 axis와
offset은 parent-link local frame에서 정의하며 FK는 다음을 순서대로 누적한다.

```text
T_world_link_i = T_world_parent × Rot(axis_i, q_i) × Trans(offset_i)
T_world_tcp    = T_world_joint6 × T_joint6_tcp
```

`ForwardKinematics`는 renderer matrix가 아닌 `RigidTransform` snapshot만 반환한다. Viewer는
그 결과를 Flecs `Transform`으로 변환할 뿐, OpenGL/Flecs가 robot math를 소유하지 않는다.

## 3. XYZ command와 DLS IK

ESP32 target `p=(x,y,z)`는 fixed TCP quaternion과 합쳐 `T_world_tcp,target`가 된다. Solver는
현재 자세에서 geometric Jacobian `J`를 구하고, position[m]과 orientation axis-angle[rad]의
scale 차이를 orientation weight로 조정한 뒤 다음을 반복한다.

```text
Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e
```

`λ`는 singularity 근처에서 관절 update가 폭주하는 것을 막는 damping이다. 각 step은 최대
joint step 및 joint limit로 clamp한다. target이 conservative reach sphere 밖이거나 반복 안에
수렴하지 않으면 마지막 finite/limit-safe joint state를 유지하고 `IkUnreachable`을 보고한다.

## 4. Kinematic grasp

초기 grasp는 contact physics가 아니다. Object-relative desired TCP를
`T_world_object × T_object_tcp,grasp`로 계산하고, TCP의 position/orientation error가 모두
threshold 안일 때만 attach한다. attach 순간 저장하는 값은 다음과 같다.

```text
T_tcp_object = inverse(T_world_tcp) × T_world_object
```

그 뒤 object pose는 `T_world_tcp × T_tcp_object`로 갱신된다. 이 local offset 때문에 attach
전후 world pose가 튀지 않는다. 새 remote target은 기존 attach를 자동 release한다.
