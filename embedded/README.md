# Embedded

GraspLink의 임베디드 영역이다.

1차 대상은 **ESP32 + Zephyr RTOS 기반 Target Controller**다. 이 장치는 로봇 자체를 제어하는 보드가 아니라, Simulator에 목표 위치를 전달하고 Simulator의 수행 상태를 다시 표시하는 외부 제어 노드다.

```text
Button / Potentiometer
        ↓
ESP32 + Zephyr
        ↓ UDP Target Command
C++ GraspLink Simulator
        ↓ UDP Robot State
OLED / RGB LED / Buzzer
```

## 디렉토리

```text
embedded/
└─ controller/
   ├─ CMakeLists.txt
   ├─ prj.conf
   ├─ boards/       # 보드 모델 확정 후 devicetree overlay 추가
   └─ src/
      └─ main.c
```

## 구현 순서

1. Zephyr sample 빌드/flash 확인
2. GPIO button interrupt
3. I2C OLED
4. ADC potentiometer
5. X/Y/Z Target Position 상태 머신
6. thread/work/message queue로 입력·UI·network 책임 분리
7. UDP Target Command 송신
8. Simulator 상태 수신
9. OLED/RGB LED/Buzzer 피드백

정확한 ESP32 보드 모델과 핀맵이 확정되기 전에는 board overlay를 임의로 작성하지 않는다.
