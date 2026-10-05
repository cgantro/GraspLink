cmake_minimum_required(VERSION 3.20)

project(GraspLink VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/utf-8)
endif()

include(cmake/Dependencies.cmake)
grasplink_add_common_dependencies()
include(CTest)


# ============================================================
# Robotics
# ============================================================
#
# Robot/Gripper model + Hardware/Simulation backend contract.
# Viewer/Flecs/GLM에 의존하지 않는 robotics domain library다.
#
add_library(grasplink_robotics STATIC
  modules/robotics/src/backends/simulation/SimRobotController.cpp
  modules/robotics/src/kinematics/RobotKinematics.cpp
)

target_include_directories(grasplink_robotics PUBLIC
  modules/robotics/include
)

target_sources(grasplink_robotics PUBLIC
  modules/robotics/include/robotics/core/ControlTypes.h
  modules/robotics/include/robotics/core/IRobotController.h
  modules/robotics/include/robotics/core/IGripperController.h

  modules/robotics/include/robotics/models/ModelTypes.h
  modules/robotics/include/robotics/models/RobotSpecification.h
  modules/robotics/include/robotics/models/GripperSpecification.h
  modules/robotics/include/robotics/models/hanwha/Hcr12a.h
  modules/robotics/include/robotics/models/robotiq/TwoF85.h

  modules/robotics/include/robotics/backends/simulation/SimRobotController.h

  modules/robotics/include/robotics/runtime/FixedControlLoop.h
  modules/robotics/include/robotics/kinematics/RobotKinematics.h
)


# ============================================================
# Physics
# ============================================================
#
# Jolt Physics dependency를 먼저 준비한다.
#
# 중요:
# add_library(grasplink_physics ...)보다 먼저 Jolt target이 생성되어 있어야
# 아래 target_link_libraries(... Jolt)가 정상적으로 동작한다.
#
grasplink_add_physics_dependencies()


# Physics library 자체를 먼저 만든다.
#
# target_sources(), target_include_directories(),
# target_link_libraries()는 모두 이 add_library 이후에 호출해야 한다.
#
add_library(grasplink_physics STATIC
  modules/physics/src/PhysicsWorld.cpp
)

target_include_directories(grasplink_physics PUBLIC
  modules/physics/include
)

target_sources(grasplink_physics PUBLIC
  modules/physics/include/PhysicsTypes.h
  modules/physics/include/PhysicsWorld.h
)

target_link_libraries(grasplink_physics PUBLIC
  Jolt
  glm::glm
)


# ============================================================
# Graphics / Viewer
# ============================================================

option(
  GRASPLINK_BUILD_GRAPHICS
  "Build the OpenGL viewer"
  ON
)

if(GRASPLINK_BUILD_GRAPHICS)

  # OpenGL / GLFW / Flecs / TinyGLTF 등의
  # Viewer 전용 dependency를 준비한다.
  grasplink_add_graphics_dependencies()

  # OpenGL 관련 공통 graphics library 생성.
  grasplink_add_graphics_library()

  # UI backend는 GUI 모듈 의존성으로 별도 구성.
  grasplink_add_gui_dependencies()


  # ----------------------------------------------------------
  # Viewer
  # ----------------------------------------------------------

  set(GRASPLINK_VIEWER_SOURCES
    modules/viewer/src/Entity.cpp

    modules/viewer/src/systems/RenderSystemModule.cpp
    modules/viewer/src/systems/TransformSystemModule.cpp
    modules/viewer/src/scene/Scene.cpp
    modules/viewer/src/scene/SceneManager.cpp

    modules/viewer/src/assets/GltfLoader.cpp
    modules/viewer/src/assets/AssetManager.cpp
    modules/viewer/src/assets/PrefabFactory.cpp

    modules/viewer/src/robotics/RobotTransformAdapter.cpp
  )

  set(GRASPLINK_VIEWER_HEADERS
    modules/viewer/include/Entity.h

    modules/viewer/include/components/Components.h
    modules/viewer/include/components/RenderComponents.h
    modules/viewer/include/components/TransformComponents.h

    modules/viewer/include/systems/RenderSystemModule.h
    modules/viewer/include/systems/TransformSystemModule.h

    modules/viewer/include/scene/Scene.h
    modules/viewer/include/scene/SceneManager.h

    modules/viewer/include/assets/GraphicsTypes.h
    modules/viewer/include/assets/ResourceID.h
    modules/viewer/include/assets/GltfLoader.h
    modules/viewer/include/assets/AssetManager.h
    modules/viewer/include/assets/PrefabFactory.h

    modules/viewer/include/viewer/robotics/RobotTransformAdapter.h
  )


  add_library(grasplink_viewer STATIC
    ${GRASPLINK_VIEWER_SOURCES}
    ${GRASPLINK_VIEWER_HEADERS}
  )

  target_include_directories(grasplink_viewer PUBLIC
    modules/viewer/include
    modules/viewer/include/components
    modules/viewer/include/graphics
    modules/viewer/include/systems
  )


  target_link_libraries(grasplink_viewer PUBLIC
    grasplink_robotics
    grasplink_graphics
    flecs::flecs_static

    PRIVATE
      tinygltf
  )

  target_compile_definitions(grasplink_viewer PRIVATE
    GLM_ENABLE_EXPERIMENTAL
  )

  add_library(grasplink_simulation STATIC
    modules/simulation/src/SimulationSceneBuilder.cpp
    modules/simulation/src/robotics/RobotPhysicsAdapter.cpp
    modules/simulation/src/systems/PhysicsSystemModule.cpp
    modules/simulation/include/simulation/SimulationSceneBuilder.h
    modules/simulation/include/simulation/components/PhysicsComponents.h
    modules/simulation/include/simulation/components/RobotCollisionProxy.h
    modules/simulation/include/simulation/robotics/RobotPhysicsAdapter.h
    modules/simulation/include/simulation/systems/PhysicsSystemModule.h
  )

  target_include_directories(grasplink_simulation PUBLIC
    modules/simulation/include
  )

  target_link_libraries(grasplink_simulation PUBLIC
    grasplink_viewer
    grasplink_physics
    grasplink_robotics
  )

  target_compile_definitions(grasplink_simulation PRIVATE
    GLM_ENABLE_EXPERIMENTAL
  )

  add_library(grasplink_gui STATIC
    modules/gui/src/GuiModule.cpp
    modules/gui/include/gui/GuiModule.h
  )

  target_include_directories(grasplink_gui PUBLIC
    modules/gui/include
  )

  target_link_libraries(grasplink_gui PUBLIC
    grasplink_simulation
    grasplink_imgui_backend
  )

  target_compile_definitions(grasplink_gui PRIVATE
    GLM_ENABLE_EXPERIMENTAL
  )


  # ----------------------------------------------------------
  # Simulator Application
  # ----------------------------------------------------------

  set(GRASPLINK_APP_SOURCES
    apps/viewer/main.cpp
    apps/viewer/ViewerApp.cpp
    apps/viewer/DebugSceneSetup.cpp
  )

  set(GRASPLINK_APP_HEADERS
    apps/viewer/ViewerApp.h
    apps/viewer/DebugSceneSetup.h
  )

  add_executable(grasplink_simulator
    ${GRASPLINK_APP_SOURCES}
    ${GRASPLINK_APP_HEADERS}
  )

  target_include_directories(grasplink_simulator PRIVATE
    apps/viewer
  )

  target_link_libraries(grasplink_simulator PRIVATE
    grasplink_gui
    grasplink_simulation
    grasplink_viewer
    grasplink_robotics
    grasplink_physics
  )


  # ----------------------------------------------------------
  # Runtime Assets
  # ----------------------------------------------------------
  #
  # 실행 파일 옆에 Shader / GLB 파일을 복사한다.
  #
  add_custom_command(
    TARGET grasplink_simulator
    POST_BUILD

    COMMAND ${CMAKE_COMMAND} -E copy_directory
      ${CMAKE_SOURCE_DIR}/assets/shaders
      $<TARGET_FILE_DIR:grasplink_simulator>/shaders

    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${CMAKE_SOURCE_DIR}/assets/HCR12A_R00.glb
      $<TARGET_FILE_DIR:grasplink_simulator>/HCR12A_R00.glb

    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${CMAKE_SOURCE_DIR}/assets/HCR12A_2F-85.glb
      $<TARGET_FILE_DIR:grasplink_simulator>/HCR12A_2F-85.glb

    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${CMAKE_SOURCE_DIR}/assets/plane.glb
      $<TARGET_FILE_DIR:grasplink_simulator>/plane.glb
  )


endif()
