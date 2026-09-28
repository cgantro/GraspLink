# Robot Control Simulation Goals

## 목표

시뮬레이터의 제어 로직을 렌더링 구현이나 특정 하드웨어에 직접 결합하지 않고,
동일한 Control Core를 Simulation과 Real Hardware에서 재사용할 수 있는 구조를 목표로 한다.

```text
Target / Controller
        ↓
   JointCommand
        ↓
 Limit / Safety
        ↓
 IRobotHardware
   ├─ SimRobotHardware
   └─ RealRobotHardware
        ↓
    JointState
```

## 우선순위

### 필수

1. **Hardware / Simulation Interface 분리**
   - 공통 `IRobotHardware` 정의
   - `SimRobotHardware`와 추후 `RealRobotHardware` 구현 분리
   - 상위 Controller는 구현체 종류를 알지 않도록 구성

2. **Joint Angle Limit**
   - Joint별 `minAngle`, `maxAngle`
   - 범위를 벗어난 명령을 적용 전에 검사

3. **Velocity / Acceleration Limit**
   - Joint별 `maxVelocity`, `maxAcceleration`
   - 목표 각도를 즉시 적용하지 않고 시간에 따라 제한된 상태 변화로 반영
   - HCR-12A 최대 속도는 사양 문서 값을 사용
   - 최대 가속도는 확인된 제조사 기준값이 없으면 별도 설정값으로 관리하고 임의의 제조사 사양처럼 취급하지 않음

### 목표

4. **Fixed Control Loop**
   - Rendering FPS와 제어 주기를 분리
   - 제어 루프는 고정된 `dt` 기준으로 Joint State를 갱신
   - 주기는 설정값으로 관리

### 확장

5. **E-Stop**
   - 활성화 시 신규 JointCommand 적용 중지
   - 제어 상태를 정지 상태로 전환
   - 실제 하드웨어의 안전 기능을 대체하는 것이 아니라 시뮬레이션용 상태 모델로 취급

6. **Zero Offset**
   - 논리적 Joint Zero와 센서/엔코더 기준점 사이의 차이를 보정할 수 있도록 offset 계층 추가

## 기본 상태 / 설정

```cpp
struct JointState {
    double positionRad;
    double velocityRadSec;
    double accelerationRadSec2;
};

struct JointLimit {
    double minRad;
    double maxRad;
    double maxVelRadSec;
    double maxAccRadSec2;
};
```

내부 단위는 다음으로 통일한다.

```text
Angle        rad
Velocity     rad/s
Acceleration rad/s²
Time         s
```

## 구현 순서

```text
1. IRobotHardware / SimRobotHardware 분리
2. Joint Angle Limit
3. Velocity Limit
4. Acceleration Limit
5. Fixed Control Loop
6. E-Stop
7. Zero Offset
```

E-Stop과 Zero Offset은 기본 제어 구조와 제한 처리가 완료된 뒤 확장한다.
