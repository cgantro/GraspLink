# Build and Run

## 1. PC Simulator

현재 root CMake의 executable target은 다음과 같다.

```text
grasplink_simulator
```

### 요구 환경

- CMake 3.20+
- C++17 compiler
- Git
- OpenGL development environment

현재 CMake가 사용하는 dependency:

- GLFW 3.4
- GLM 1.0.1
- Flecs 4.1.5
- GLAD
- OpenGL

### Configure

```bash
cmake -S . -B build -DGRASPLINK_BUILD_GRAPHICS=ON
```

### Build

Single-config generator:

```bash
cmake --build build -j
```

Visual Studio 등 multi-config generator:

```bash
cmake --build build --config Release
```

### Run

Single-config 예:

```bash
cd build
./grasplink_simulator
```

Windows multi-config 예:

```text
build/Release/grasplink_simulator.exe
```

CMake post-build 단계에서 `assets/shaders`를 executable directory의 `shaders/`로 복사한다.

---

## 2. Embedded Controller

Embedded application은 다음에 있다.

```text
embedded/controller
```

요구 환경:

- VS Code 사용 가능
- Zephyr development environment
- west
- CMake / Ninja
- ESP32 toolchain 지원

정확한 `-b <board>` 값은 실제 ESP32 보드 모델을 확인한 뒤 결정한다.

### Build

Zephyr workspace에서 예:

```bash
west build -b <board> embedded/controller
```

이미 다른 board로 build한 directory가 있다면 pristine build를 사용한다.

```bash
west build -p always -b <board> embedded/controller
```

### Flash

```bash
west flash
```

### Serial log

보드/runner 환경에 따라 Zephyr 또는 ESP32 serial monitor를 사용한다. 첫 bring-up의 성공 기준은 다음 로그를 확인하는 것이다.

```text
GraspLink embedded controller booted
```

---

## 3. Board overlay

보드별 DeviceTree overlay는:

```text
embedded/controller/boards/
```

아래에 둔다.

초기 연결 예정:

- Button → GPIO interrupt
- Potentiometer → ADC
- OLED → I2C
- RGB LED → GPIO/PWM 검토
- Active Buzzer → GPIO/PWM 검토

핀 번호는 실제 보드의 Zephyr DTS와 핀맵을 확인한 뒤 작성한다.

---

## 4. 현재 구현 상태 주의

`embedded/controller`는 현재 bring-up용 skeleton이다. GPIO/ADC/I2C 장치 binding과 UDP 구현은 보드 확인 후 추가한다.

PC 쪽도 Robot Model/FK/IK/UDP pipeline 전체가 완료된 상태는 아니다.

따라서 문서의 최종 구조와 현재 실행 가능한 기능을 구분한다.

---

## 5. 권장 개발 순서

```text
PC Simulator가 기존 OpenGL scene을 정상 렌더링
        ↓
ESP32 Zephyr build / flash / serial log
        ↓
Button + OLED
        ↓
ADC Target Position
        ↓
UDP Target Command
        ↓
Simulator Target Object
        ↓
Robot FK / IK / Grasp
        ↓
State Feedback
```

자세한 4일 계획은 [ROADMAP.md](ROADMAP.md)를 따른다.
