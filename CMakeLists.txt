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
# Diagnostics
# ============================================================
#
# Logger와 Profiler는 domain/graphics 모듈과 독립된 비동기 기록 기능을 제공한다.
#
add_library(grasplink_diagnostics STATIC
  modules/diagnostics/src/Logger.cpp
  modules/diagnostics/src/Profiler.cpp
  modules/diagnostics/include/diagnostics/Logger.h
  modules/diagnostics/include/diagnostics/Profiler.h
)

target_include_directories(grasplink_diagnostics PUBLIC
  modules/diagnostics/include
)

find_package(Threads REQUIRED)
target_link_libraries(grasplink_diagnostics PUBLIC Threads::Threads)


# ============================================================
# Robotics
# ============================================================
#
# 이 라이브러리는 로봇과 그리퍼 모델 사양, 하드웨어·시뮬레이션 backend의 공통 계약을 제공한다.
# Viewer, Flecs, GLM에 의존하지 않으므로 화면이나 ECS가 없는 로봇 코드에서도 사용할 수 있다.
#
add_library(grasplink_robotics STATIC
  modules/robotics/src/backends/simulation/SimRobotController.cpp
  modules/robotics/src/backends/simulation/detail/LinearPathPlanner.cpp
  modules/robotics/src/backends/simulation/SimGripperController.cpp
  modules/robotics/src/kinematics/RobotKinematics.cpp
  modules/robotics/src/kinematics/RobotInverseKinematics.cpp
  modules/robotics/src/kinematics/GripperKinematics.cpp
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
  modules/robotics/include/robotics/backends/simulation/detail/LinearPathPlanner.h
  modules/robotics/include/robotics/backends/simulation/SimGripperController.h

  modules/robotics/include/robotics/runtime/FixedControlLoop.h
  modules/robotics/include/robotics/kinematics/RobotKinematics.h
  modules/robotics/include/robotics/kinematics/RobotInverseKinematics.h
  modules/robotics/include/robotics/kinematics/detail/PoseMath.h
  modules/robotics/include/robotics/kinematics/GripperKinematics.h
)


# ============================================================
# Physics
# ============================================================
#
# Jolt Physics 의존성을 먼저 구성한다. 아래 물리 라이브러리가 링크할 Jolt target이 이 단계에서 만들어진다.
#
# 따라서 이 설정은 add_library(grasplink_physics ...)보다 앞에 있어야 이후 target_link_libraries(... Jolt)가 링크할 대상을 찾을 수 있다.
#
grasplink_add_physics_dependencies()


# 물리 라이브러리 target을 먼저 만든다.
#
# target_sources(), target_include_directories(), target_link_libraries()는 이 add_library 호출 뒤에 실행해야 설정이 해당 라이브러리에 적용된다.
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

if(BUILD_TESTING)
  add_executable(grasplink_physics_tests tests/PhysicsWorldTests.cpp)
  target_link_libraries(grasplink_physics_tests PRIVATE grasplink_physics)
  add_test(NAME PhysicsWorld COMMAND grasplink_physics_tests)

  add_executable(grasplink_control_tests tests/ControlRuntimeTests.cpp)
  target_link_libraries(grasplink_control_tests PRIVATE grasplink_robotics)
  add_test(NAME ControlRuntime COMMAND grasplink_control_tests)

  add_executable(grasplink_kinematics_tests tests/RobotKinematicsTests.cpp)
  target_link_libraries(grasplink_kinematics_tests PRIVATE grasplink_robotics)
  add_test(NAME RobotKinematics COMMAND grasplink_kinematics_tests)

  add_executable(grasplink_inverse_kinematics_tests tests/RobotInverseKinematicsTests.cpp)
  target_link_libraries(grasplink_inverse_kinematics_tests PRIVATE grasplink_robotics)
  add_test(NAME RobotInverseKinematics COMMAND grasplink_inverse_kinematics_tests)

  add_executable(grasplink_robot_motion_tests tests/RobotMotionTests.cpp)
  target_link_libraries(grasplink_robot_motion_tests PRIVATE grasplink_robotics)
  add_test(NAME RobotMotion COMMAND grasplink_robot_motion_tests)

  add_executable(grasplink_robot_panel_motion_tests tests/RobotPanelMotionTests.cpp)
  target_include_directories(grasplink_robot_panel_motion_tests PRIVATE modules/gui/include)
  target_link_libraries(grasplink_robot_panel_motion_tests PRIVATE grasplink_robotics glm::glm)
  add_test(NAME RobotPanelMotion COMMAND grasplink_robot_panel_motion_tests)

  add_executable(grasplink_gripper_runtime_tests tests/GripperRuntimeTests.cpp)
  target_link_libraries(grasplink_gripper_runtime_tests PRIVATE grasplink_robotics)
  add_test(NAME GripperRuntime COMMAND grasplink_gripper_runtime_tests)

  add_executable(grasplink_gripper_kinematics_tests tests/GripperKinematicsTests.cpp)
  target_link_libraries(grasplink_gripper_kinematics_tests PRIVATE grasplink_robotics)
  add_test(NAME GripperKinematics COMMAND grasplink_gripper_kinematics_tests)
endif()

# ============================================================
# Graphics / Viewer
# ============================================================

option(
  GRASPLINK_BUILD_GRAPHICS
  "Build the OpenGL viewer"
  ON
)

if(GRASPLINK_BUILD_GRAPHICS)

  # Viewer가 사용하는 OpenGL, GLFW, Flecs, TinyGLTF 등의 외부 의존성을 구성한다.
  grasplink_add_graphics_dependencies()

  # Viewer와 앱이 공유할 OpenGL 그래픽 라이브러리 target을 만든다.
  grasplink_add_graphics_library()

  # ImGui UI backend는 공통 그래픽 코드와 분리해 GUI 모듈 의존성으로 구성한다.
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
    modules/viewer/src/robotics/GripperTransformAdapter.cpp
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
    modules/viewer/include/viewer/robotics/GripperTransformAdapter.h
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
    modules/simulation/src/robotics/CollisionGeometry.cpp
    modules/simulation/src/robotics/GripperColliders.cpp
    modules/simulation/src/robotics/GripperGraspAdapter.cpp
    modules/simulation/src/robotics/RobotPhysicsAdapter.cpp
    modules/simulation/src/systems/PhysicsSystemModule.cpp
    modules/simulation/include/simulation/SimulationSceneBuilder.h
    modules/simulation/include/simulation/components/PhysicsComponents.h
    modules/simulation/include/simulation/components/RobotCollisionProxy.h
    modules/simulation/include/simulation/robotics/RobotPhysicsAdapter.h
    modules/simulation/include/simulation/robotics/GripperColliders.h
    modules/simulation/include/simulation/robotics/GripperGraspAdapter.h
    modules/simulation/src/robotics/CollisionGeometry.h
    modules/simulation/include/simulation/systems/PhysicsSystemModule.h
  )

  target_include_directories(grasplink_simulation PUBLIC
    modules/simulation/include
  )

  target_link_libraries(grasplink_simulation PUBLIC
    grasplink_viewer
    grasplink_physics
    grasplink_robotics
    grasplink_diagnostics
  )

  target_compile_definitions(grasplink_simulation PRIVATE
    GLM_ENABLE_EXPERIMENTAL
  )

  add_library(grasplink_gui STATIC
    modules/gui/src/GuiModule.cpp
    modules/gui/include/gui/GuiModule.h
    modules/gui/src/panels/GripperPanel.cpp
    modules/gui/include/gui/panels/GripperPanel.h
    modules/gui/src/panels/RobotPanel.cpp
    modules/gui/include/gui/panels/RobotPanel.h
    modules/gui/src/panels/PhysicsDebugPanel.cpp
    modules/gui/include/gui/panels/PhysicsDebugPanel.h
    modules/gui/src/overlays/ColliderOverlay.cpp
    modules/gui/include/gui/overlays/ColliderOverlay.h
  )

  target_include_directories(grasplink_gui PUBLIC
    modules/gui/include
  )

  target_link_libraries(grasplink_gui PUBLIC
    grasplink_robotics
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
    apps/viewer/ViewerRobotCollisionGuard.cpp
    apps/viewer/DebugSceneSetup.cpp
  )

  set(GRASPLINK_APP_HEADERS
    apps/viewer/ViewerApp.h
    apps/viewer/ViewerRobotCollisionGuard.h
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
    grasplink_diagnostics
  )


  # ----------------------------------------------------------
  # Runtime Assets
  # ----------------------------------------------------------
  #
  # 실행 파일이 자산을 찾을 수 있도록 Shader와 GLB 파일을 실행 파일 디렉터리에 복사한다.
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

  if(BUILD_TESTING)
    add_executable(grasplink_gui_composition_tests tests/GuiCompositionTests.cpp)
    target_link_libraries(grasplink_gui_composition_tests PRIVATE grasplink_gui)
    add_test(NAME GuiComposition COMMAND grasplink_gui_composition_tests)
    add_executable(grasplink_quaternion_transform_tests tests/QuaternionTransformTests.cpp)
    target_link_libraries(grasplink_quaternion_transform_tests PRIVATE grasplink_viewer)
    add_test(NAME QuaternionTransforms COMMAND grasplink_quaternion_transform_tests)
    add_executable(grasplink_graphics_tests tests/GraphicsResourceTests.cpp)
    target_link_libraries(grasplink_graphics_tests PRIVATE grasplink_graphics)
    add_test(NAME GraphicsResources COMMAND grasplink_graphics_tests)
    set_tests_properties(GraphicsResources PROPERTIES TIMEOUT 60)

    add_executable(grasplink_gltf_tests tests/GltfLoaderTests.cpp)
    target_link_libraries(grasplink_gltf_tests PRIVATE grasplink_viewer)
    target_compile_definitions(grasplink_gltf_tests PRIVATE GRASPLINK_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets")
    add_test(NAME GltfLoader COMMAND grasplink_gltf_tests)

    add_executable(grasplink_robot_pose_integration_tests tests/RobotPoseIntegrationTests.cpp)
    target_link_libraries(grasplink_robot_pose_integration_tests PRIVATE grasplink_viewer)
    add_test(NAME RobotPoseIntegration COMMAND grasplink_robot_pose_integration_tests)
    set_tests_properties(RobotPoseIntegration PROPERTIES WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

    add_executable(grasplink_physics_ecs_tests tests/PhysicsEcsIntegrationTests.cpp)
    target_link_libraries(grasplink_physics_ecs_tests PRIVATE grasplink_simulation)
    add_test(NAME PhysicsEcsIntegration COMMAND grasplink_physics_ecs_tests)

    add_executable(grasplink_collision_tests tests/RobotCollisionGeometryTests.cpp)
    target_link_libraries(grasplink_collision_tests PRIVATE grasplink_simulation)
    add_test(NAME RobotCollisionGeometry COMMAND grasplink_collision_tests)
    set_tests_properties(RobotCollisionGeometry PROPERTIES WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

    add_executable(grasplink_gripper_collision_tests tests/GripperCollisionIntegrationTests.cpp)
    target_link_libraries(grasplink_gripper_collision_tests PRIVATE grasplink_simulation)
    add_test(NAME GripperCollisionIntegration COMMAND grasplink_gripper_collision_tests)
    set_tests_properties(GripperCollisionIntegration PROPERTIES
      WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}" TIMEOUT 60)

    add_executable(grasplink_gripper_pose_tests tests/GripperPoseIntegrationTests.cpp)
    target_link_libraries(grasplink_gripper_pose_tests PRIVATE grasplink_simulation)
    add_test(NAME GripperPoseIntegration COMMAND grasplink_gripper_pose_tests)
    set_tests_properties(GripperPoseIntegration PROPERTIES
      WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}" TIMEOUT 60)

    add_executable(grasplink_gripper_grasp_tests tests/GripperGraspIntegrationTests.cpp)
    target_link_libraries(grasplink_gripper_grasp_tests PRIVATE grasplink_simulation)
    add_test(NAME GripperGraspIntegration COMMAND grasplink_gripper_grasp_tests)
    set_tests_properties(GripperGraspIntegration PROPERTIES
      WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}" TIMEOUT 60)

    add_test(NAME ViewerSmoke COMMAND grasplink_simulator --smoke-test)
    add_test(NAME ViewerDemoSmoke COMMAND grasplink_simulator --smoke-test --physics-demo)
    set_tests_properties(ViewerSmoke ViewerDemoSmoke PROPERTIES
      WORKING_DIRECTORY "$<TARGET_FILE_DIR:grasplink_simulator>" TIMEOUT 60)
  endif()

endif()
