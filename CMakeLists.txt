cmake_minimum_required(VERSION 3.20)

project(GraspLink VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/utf-8)
endif()

option(GRASPLINK_BUILD_GRAPHICS "Build the OpenGL HCR-12A simulator" ON)
option(GRASPLINK_BUILD_TESTS "Build deterministic GraspLink tests" ON)

# Domain libraries deliberately have no OpenGL/Flecs dependency. This permits
# protocol and robot-math verification on CI workers without a GPU.
add_library(grasplink_kinematics STATIC
  modules/kinematics/src/RobotKinematics.cpp
  modules/kinematics/src/Grasping.cpp
)
target_include_directories(grasplink_kinematics PUBLIC
  modules/common/include
  modules/kinematics/include
)

target_include_directories(grasplink_transport PUBLIC
  modules/common/include
  modules/transport/include
)
if(WIN32)
  target_link_libraries(grasplink_transport PUBLIC ws2_32)
endif()

add_library(grasplink_streaming STATIC
  modules/streaming/src/PoseBuffer.cpp
  modules/streaming/src/PoseReceiver.cpp
)
target_include_directories(grasplink_streaming PUBLIC
  modules/common/include
  modules/transport/include
  modules/streaming/include
)
target_link_libraries(grasplink_streaming PUBLIC grasplink_transport)

add_library(grasplink_vision STATIC
  modules/vision/src/SyntheticPoseSource.cpp
)
target_include_directories(grasplink_vision PUBLIC
  modules/common/include
  modules/vision/include
)

if(GRASPLINK_BUILD_GRAPHICS)
  include(cmake/Dependencies.cmake)
  grasplink_add_graphics_library()

  add_executable(grasplink_simulator
    apps/viewer/main.cpp
    apps/viewer/ViewerApp.cpp
    apps/viewer/ViewerApp.h
    apps/viewer/CadVisualRig.cpp
    apps/viewer/CadVisualRig.h
    apps/viewer/SimulatorConfig.cpp
    apps/viewer/SimulatorConfig.h
    modules/viewer/include/Entity.h
    modules/viewer/include/systems/RenderSystem.h
    modules/viewer/src/systems/RenderSystem.cpp
  )
  target_include_directories(grasplink_simulator PRIVATE
    apps/viewer
    modules/common/include
    modules/kinematics/include
    modules/streaming/include
    modules/transport/include
    modules/viewer/include
    modules/viewer/include/components
    modules/viewer/include/graphics
    modules/viewer/include/systems
    modules/vision/include
  )
  target_link_libraries(grasplink_simulator PRIVATE
    grasplink_graphics
    grasplink_kinematics
    grasplink_streaming
    grasplink_transport
    grasplink_vision
    flecs::flecs_static
  )
  add_custom_command(TARGET grasplink_simulator POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
      ${CMAKE_SOURCE_DIR}/assets/shaders
      $<TARGET_FILE_DIR:grasplink_simulator>/shaders
    COMMAND ${CMAKE_COMMAND} -E make_directory
      $<TARGET_FILE_DIR:grasplink_simulator>/assets/hcr12a
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${CMAKE_SOURCE_DIR}/assets/hcr12a/HCR12A_R00.glb
      $<TARGET_FILE_DIR:grasplink_simulator>/assets/hcr12a/HCR12A_R00.glb
  )
  target_compile_definitions(grasplink_simulator PRIVATE
    GRASPLINK_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
  )
endif()

if(GRASPLINK_BUILD_TESTS)
  enable_testing()
  add_executable(grasplink_tests tests/test_main.cpp)
  target_include_directories(grasplink_tests PRIVATE
    modules/common/include
    modules/kinematics/include
    modules/streaming/include
    modules/transport/include
  )
  target_link_libraries(grasplink_tests PRIVATE
    grasplink_kinematics
    grasplink_streaming
    grasplink_transport
  )
  add_test(NAME grasplink_tests COMMAND grasplink_tests)
endif()
