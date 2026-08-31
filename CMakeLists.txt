cmake_minimum_required(VERSION 3.20)
project(PoseLink VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
option(POSELINK_BUILD_VIEWER "Enable OpenGL viewer mode" ON)
option(POSELINK_BUILD_VISION "Enable OpenCV when installed" ON)

include(cmake/Dependencies.cmake)

add_library(poselink_pose INTERFACE)
target_include_directories(poselink_pose INTERFACE modules/pose/include)

add_library(poselink_transport STATIC modules/transport/src/Protocol.cpp modules/transport/src/UdpSocket.cpp)
target_include_directories(poselink_transport PUBLIC modules/transport/include)
target_link_libraries(poselink_transport PUBLIC poselink_pose)
if(WIN32)
  target_link_libraries(poselink_transport PUBLIC ws2_32)
endif()

add_library(poselink_streaming STATIC modules/streaming/src/PoseBuffer.cpp modules/streaming/src/PoseReceiver.cpp)
target_include_directories(poselink_streaming PUBLIC modules/streaming/include)
target_link_libraries(poselink_streaming PUBLIC poselink_pose poselink_transport)

add_library(poselink_vision STATIC modules/vision/src/SyntheticPoseSource.cpp modules/vision/src/ArUcoPoseSource.cpp)
target_include_directories(poselink_vision PUBLIC modules/vision/include)
target_link_libraries(poselink_vision PUBLIC poselink_pose)

add_executable(poselink
  apps/poselink/src/main.cpp
  apps/poselink/src/VisionMode.cpp
  apps/poselink/src/ProxyMode.cpp)
target_include_directories(poselink PRIVATE apps/poselink/include)
target_link_libraries(poselink PRIVATE poselink_transport poselink_streaming poselink_vision)

if(POSELINK_BUILD_VISION)
  find_package(OpenCV QUIET COMPONENTS core calib3d aruco videoio imgcodecs)
  if(OpenCV_FOUND)
    target_compile_definitions(poselink PRIVATE POSELINK_WITH_OPENCV=1)
    target_link_libraries(poselink_vision PUBLIC ${OpenCV_LIBS})
  endif()
endif()

if(POSELINK_BUILD_VIEWER)
  poselink_enable_viewer(poselink)
endif()

enable_testing()
add_executable(poselink_tests tests/test_main.cpp)
target_link_libraries(poselink_tests PRIVATE poselink_streaming)
add_test(NAME poselink_tests COMMAND poselink_tests)
