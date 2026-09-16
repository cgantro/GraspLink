# GraspLink Embedded Controller

## 1. 역할

Embedded Controller는 Simulator에 직접 로봇 joint command를 보내는 장치가 아니다.

주요 책임은 다음 세 가지다.

1. 사용자 입력에서 Target Position 생성
2. Target Command를 UDP로 Simulator에 전달
3. Simulator 상태를 수신해 로컬 UI로 표시

```text
Input Device
→ Target Controller
→ Network
→ Simulator
→ Robot State
→ Embedded Feedback UI
```

---

## 2. 대상 하드웨어

초기 하드웨어:

- ESP32 계열 보드
- Breadboard
- Button
- Potentiometer
- OLED
- RGB LED
- Active Buzzer

보유 중인 PIR, 조도 센서, 온습도 센서, Avoidance Module, Relay는 1차 prototype 범위에서 제외한다. 기능과 직접 연결되지 않는 센서를 억지로 포함하지 않는다.

IMU 역시 1차 범위에서 제외한다. 이후 orientation target이 필요할 때 별도 확장한다.

---

## 3. Zephyr 사용 범위

### DeviceTree

보드별 GPIO/ADC/I2C 연결 정보를 application logic과 분리한다.

```text
board overlay
→ device binding
→ application
```

정확한 board 모델이 결정된 뒤 `embedded/controller/boards/`에 overlay를 둔다.

### GPIO interrupt

Button은 polling 대신 interrupt 기반 입력을 우선한다.

ISR에서는 상태 머신, OLED update, socket send를 직접 수행하지 않는다.

```text
GPIO ISR
→ event / work / queue
→ application context
```

### ADC

Potentiometer 값을 읽어 현재 선택된 축의 target coordinate로 mapping한다.

```text
ADC raw
→ normalize
→ workspace range mapping
→ target x/y/z
```

### I2C

OLED에 현재 좌표와 connection/robot 상태를 표시한다.

### Thread / Work / Queue

초기 책임 경계:

```text
Input path
  Button / ADC
      ↓
Target State
      ↓
Network TX

Network RX
      ↓
Robot State
      ↓
Display / LED / Buzzer
```

실제 thread 개수는 구현 복잡도에 따라 최소화한다. 단순히 RTOS를 사용했다는 이유로 불필요한 thread를 많이 만들지 않는다.

---

## 4. Target 입력 UX

Potentiometer 하나와 Button을 이용해 3축 좌표를 입력한다.

예시:

```text
MODE: X
X: 0.30
Y: 0.10
Z: 0.45
```

Button short press:

```text
X → Y → Z → X
```

별도 SEND 입력 방식은 실제 button 수와 wiring을 확인한 뒤 결정한다. 하나의 button만 사용할 경우 long press로 SEND를 구현할 수 있다.

---

## 5. 상태 피드백

Simulator에서 다음과 같은 논리 상태를 전달한다.

```text
IDLE
TARGET_RECEIVED
MOVING
GRASP_SUCCESS
GRASP_FAILED
```

표시 예:

- OLED: 상태 문자열과 target coordinate
- RGB LED: 연결/이동/성공 상태
- Active Buzzer: 성공 또는 오류 event

색상과 buzzer pattern은 구현 단계에서 확정한다.

---

## 6. 네트워크 원칙

초기 transport는 UDP다.

이유:

- target/state message가 작다.
- 로컬 네트워크 prototype에서 구현이 단순하다.
- 이후 sequence/timestamp를 사용한 stale packet 처리와 loss 실험으로 확장하기 쉽다.

명령은 queue에 무한히 쌓지 않고 최신 target을 우선한다.

향후 packet에 다음 field를 포함한다.

```text
message type
sequence
timestamp
target id
position x/y/z
```

Simulator → ESP32 상태 packet은 command와 구분한다.

---

## 7. 디렉토리 책임

```text
embedded/controller/
├─ CMakeLists.txt
├─ prj.conf
├─ boards/
│  └─ <board>.overlay
└─ src/
   ├─ main.c
   ├─ input/       # 예정
   ├─ display/     # 예정
   ├─ network/     # 예정
   └─ app/         # 예정
```

초기에는 `main.c` 하나로 bring-up하고, 실제 책임이 생길 때 파일을 분리한다. 파일 수를 먼저 늘리지 않는다.

---

## 8. 1차 완료 기준

Embedded 쪽 완료 조건은 다음과 같다.

- Zephyr build/flash 가능
- DeviceTree overlay로 실제 peripheral binding
- Button interrupt 동작
- Potentiometer ADC 동작
- OLED 출력
- X/Y/Z target 조작
- UDP command 송신
- UDP state 수신
- 상태를 OLED/LED/Buzzer에 반영

위 항목이 끝나면 Embedded Controller는 GraspLink prototype의 독립된 구성요소로 간주한다.
