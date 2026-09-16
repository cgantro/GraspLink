# Board overlays

정확한 ESP32 보드 모델이 확정되면 이 디렉토리에 Zephyr devicetree overlay를 추가한다.

예정 연결:

- Button: GPIO interrupt
- Potentiometer: ADC
- OLED: I2C
- RGB LED: GPIO/PWM 필요 여부 확인
- Active Buzzer: GPIO/PWM 필요 여부 확인

핀 번호와 alias는 실제 보드의 DTS/핀맵을 확인한 뒤 작성한다. 보드 확인 전에는 임의의 GPIO 번호를 문서나 코드에 고정하지 않는다.
