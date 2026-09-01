# 플랫폼과 의존성

- 언어/빌드: C++17 이상, CMake 3.20 이상
- Windows: Winsock UDP와 별도 실험 도구 `poselink_net_proxy` 사용
- Linux: POSIX UDP socket 사용. impairment 실험은 필요한 권한 아래 `tc netem` 사용
- Viewer: OpenGL, GLFW 3.4, bundled GLAD, GLM 1.0.1. GLFW와 GLM은 CMake `FetchContent`로 받음
- Vision: OpenCV core/calib3d/aruco/videoio/imgcodecs는 선택 dependency

현재 `POSELINK_BUILD_GRAPHICS` option만 존재합니다. 이후 module target은 graphics와 분리하여, OpenCV 없이도 synthetic source와 protocol/buffer tests를 빌드할 수 있게 구성합니다. 실제 camera 단계에서만 OpenCV option과 dependency를 추가합니다.

서로 다른 PC에서 측정하려면 NTP/PTP 같은 clock synchronization 또는 별도 offset/RTT 추정이 필요합니다. 그 전에는 수신 시계 기반 arrival jitter와 동일 머신 실험만 정량 근거로 사용합니다.
