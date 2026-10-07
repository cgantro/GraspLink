cmake_minimum_required(VERSION 3.20)

project(GraspLink VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/utf-8 "$<$<COMPILE_LANGUAGE:CXX>:/EHsc>")
endif()

include(cmake/Dependencies.cmake)
grasplink_add_common_dependencies()
include(CTest)

option(GRASPLINK_ENABLE_TRACY "Enable Tracy runtime profiling" ON)
option(GRASPLINK_BUILD_BENCHMARKS "Build FK, IK, and path-planning benchmarks" ${BUILD_TESTING})

if(GRASPLINK_ENABLE_TRACY)
  set(TRACY_ENABLE ON CACHE BOOL "Enable Tracy runtime profiling" FORCE)
  set(TRACY_ON_DEMAND ON CACHE BOOL "Capture Tracy data only when a profiler connects" FORCE)
  FetchContent_Declare(
    tracy
    GIT_REPOSITORY https://github.com/wolfpld/tracy.git
    GIT_TAG v0.14.1
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(tracy)
endif()

if(BUILD_TESTING)
  set(BUILD_GMOCK OFF CACHE BOOL "Disable Google Mock" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "Disable GoogleTest installation" FORCE)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.18.0
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(googletest)
endif()

if(BUILD_TESTING AND GRASPLINK_BUILD_BENCHMARKS)
  set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "Disable Google Benchmark tests" FORCE)
  set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "Disable Google Benchmark GoogleTest tests" FORCE)
  set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "Disable Google Benchmark installation" FORCE)
  FetchContent_Declare(
    googlebenchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG v1.9.5
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(googlebenchmark)
endif()


# ============================================================
# Diagnostics
# ============================================================
#
# Logger는 domain/graphics 모듈과 독립된 비동기 log와 metric 기록을 제공한다.
#
add_library(grasplink_diagnostics STATIC
  modules/diagnostics/src/Logger.cpp
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
  modules/robotics/src/planning/WristAlignmentPlanner.cpp
)

target_include_directories(grasplink_robotics PUBLIC
  modules/robotics/include
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

target_link_libraries(grasplink_physics PUBLIC
  Jolt
  glm::glm
)

if(BUILD_TESTING)
  include(GoogleTest)

  function(grasplink_add_test target name source)
    set(discover_tests FALSE)
    list(FIND ARGN GTEST_DISCOVERY discovery_index)
    if(NOT discovery_index EQUAL -1)
      set(discover_tests TRUE)
      list(REMOVE_AT ARGN ${discovery_index})
    endif()
    add_executable(${target} ${source})
    target_link_libraries(${target} PRIVATE ${ARGN} GTest::gtest_main)
    if(discover_tests)
      gtest_discover_tests(${target} TEST_PREFIX "${name}.")
    else()
      add_test(NAME ${name} COMMAND ${target})
    endif()
  endfunction()

  grasplink_add_test(grasplink_physics_tests PhysicsWorld tests/PhysicsWorldTests.cpp grasplink_physics)
  grasplink_add_test(grasplink_control_tests ControlRuntime tests/ControlRuntimeTests.cpp grasplink_robotics GTEST_DISCOVERY)
  grasplink_add_test(grasplink_kinematics_tests RobotKinematics tests/RobotKinematicsTests.cpp grasplink_robotics GTEST_DISCOVERY)
  grasplink_add_test(grasplink_inverse_kinematics_tests RobotInverseKinematics tests/RobotInverseKinematicsTests.cpp grasplink_robotics GTEST_DISCOVERY)
  grasplink_add_test(grasplink_robot_motion_tests RobotMotion tests/RobotMotionTests.cpp grasplink_robotics GTEST_DISCOVERY)
  grasplink_add_test(grasplink_robot_panel_motion_tests RobotPanelMotion tests/RobotPanelMotionTests.cpp grasplink_robotics GTEST_DISCOVERY)
  grasplink_add_test(grasplink_gripper_runtime_tests GripperRuntime tests/GripperRuntimeTests.cpp grasplink_robotics GTEST_DISCOVERY)
  grasplink_add_test(grasplink_gripper_kinematics_tests GripperKinematics tests/GripperKinematicsTests.cpp grasplink_robotics GTEST_DISCOVERY)
endif()

# ============================================================
# Graphics / Viewer
# ============================================================

option(
  GRASPLINK_BUILD_GRAPHICS
  "Build the OpenGL viewer"
  ON
)

grasplink_add_scene_dependencies()

add_library(grasplink_scene STATIC
  modules/viewer/src/Entity.cpp
  modules/viewer/src/scene/Scene.cpp
  modules/viewer/src/systems/TransformSystemModule.cpp
  modules/viewer/src/robotics/RobotTransformAdapter.cpp
  modules/viewer/src/robotics/GripperTransformAdapter.cpp
)
target_include_directories(grasplink_scene PUBLIC
  modules/viewer/include
  modules/viewer/include/components
  modules/viewer/include/systems
)
target_link_libraries(grasplink_scene PUBLIC flecs::flecs_static glm::glm grasplink_robotics)
target_compile_definitions(grasplink_scene PRIVATE GLM_ENABLE_EXPERIMENTAL)

add_library(grasplink_model_data STATIC modules/viewer/src/assets/GltfLoader.cpp)
target_include_directories(grasplink_model_data PUBLIC modules/viewer/include)
target_link_libraries(grasplink_model_data PUBLIC glm::glm PRIVATE tinygltf)
target_compile_definitions(grasplink_model_data PRIVATE GLM_ENABLE_EXPERIMENTAL)

add_library(grasplink_simulation STATIC
  modules/simulation/src/RandomScenario.cpp
  modules/simulation/src/SimulationSceneBuilder.cpp
  modules/simulation/src/robotics/CollisionGeometry.cpp
  modules/simulation/src/robotics/GripperColliders.cpp
  modules/simulation/src/robotics/GripperGraspAdapter.cpp
  modules/simulation/src/robotics/RobotCollisionGeometryBuilder.cpp
  modules/simulation/src/robotics/RobotPhysicsAdapter.cpp
  modules/simulation/src/systems/PhysicsSystemModule.cpp
)
target_include_directories(grasplink_simulation PUBLIC modules/simulation/include)
target_link_libraries(grasplink_simulation PUBLIC
  grasplink_scene
  grasplink_model_data
  grasplink_physics
  grasplink_robotics
  grasplink_diagnostics
)
target_compile_definitions(grasplink_simulation PRIVATE GLM_ENABLE_EXPERIMENTAL)

add_library(grasplink_application STATIC apps/viewer/PickPlaceMission.cpp)
target_include_directories(grasplink_application PUBLIC apps/viewer)
target_link_libraries(grasplink_application PUBLIC grasplink_robotics glm::glm)
target_compile_definitions(grasplink_application PRIVATE GLM_ENABLE_EXPERIMENTAL)

if(BUILD_TESTING)
  grasplink_add_test(grasplink_pick_place_mission_tests PickPlaceMission tests/PickPlaceMissionTests.cpp grasplink_application GTEST_DISCOVERY)
  grasplink_add_test(grasplink_random_scenario_tests RandomScenario tests/RandomScenarioTests.cpp grasplink_simulation GTEST_DISCOVERY)
  grasplink_add_test(grasplink_gltf_tests GltfLoader tests/GltfLoaderTests.cpp grasplink_model_data)
  target_compile_definitions(grasplink_gltf_tests PRIVATE GRASPLINK_TEST_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets")

  grasplink_add_test(grasplink_physics_ecs_tests PhysicsEcsIntegration tests/PhysicsEcsIntegrationTests.cpp grasplink_simulation)
  grasplink_add_test(grasplink_collision_tests RobotCollisionGeometry tests/RobotCollisionGeometryTests.cpp grasplink_simulation)
  target_compile_definitions(grasplink_collision_tests PRIVATE GRASPLINK_TEST_SCENE_SUPPORT)
  set_tests_properties(RobotCollisionGeometry PROPERTIES WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

  grasplink_add_test(grasplink_gripper_collision_tests GripperCollisionIntegration tests/GripperCollisionIntegrationTests.cpp grasplink_simulation)
  target_compile_definitions(grasplink_gripper_collision_tests PRIVATE GRASPLINK_TEST_SCENE_SUPPORT)
  set_tests_properties(GripperCollisionIntegration PROPERTIES WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}" TIMEOUT 60)

  grasplink_add_test(grasplink_gripper_pose_tests GripperPoseIntegration tests/GripperPoseIntegrationTests.cpp grasplink_simulation)
  target_compile_definitions(grasplink_gripper_pose_tests PRIVATE GRASPLINK_TEST_SCENE_SUPPORT)
  set_tests_properties(GripperPoseIntegration PROPERTIES WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}" TIMEOUT 60)

  grasplink_add_test(grasplink_gripper_grasp_tests GripperGraspIntegration tests/GripperGraspIntegrationTests.cpp grasplink_simulation)
  target_compile_definitions(grasplink_gripper_grasp_tests PRIVATE GRASPLINK_TEST_SCENE_SUPPORT)
  set_tests_properties(GripperGraspIntegration PROPERTIES WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}" TIMEOUT 60)
endif()

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
    modules/viewer/src/systems/RenderSystemModule.cpp
    modules/viewer/src/assets/AssetManager.cpp
    modules/viewer/src/assets/PrefabFactory.cpp
  )

  add_library(grasplink_viewer STATIC
    ${GRASPLINK_VIEWER_SOURCES}
  )

  target_include_directories(grasplink_viewer PUBLIC
    modules/viewer/include
    modules/viewer/include/components
    modules/viewer/include/graphics
    modules/viewer/include/systems
  )


  target_link_libraries(grasplink_viewer PUBLIC
    grasplink_scene
    grasplink_model_data
    grasplink_robotics
    grasplink_graphics
  )

  target_compile_definitions(grasplink_viewer PRIVATE
    GLM_ENABLE_EXPERIMENTAL
  )

  add_library(grasplink_gui STATIC
    modules/gui/src/GuiModule.cpp
    modules/gui/src/panels/GripperPanel.cpp
    modules/gui/src/panels/RobotPanel.cpp
    modules/gui/src/panels/PhysicsDebugPanel.cpp
    modules/gui/src/overlays/ColliderOverlay.cpp
  )

  target_include_directories(grasplink_gui PUBLIC
    modules/gui/include
  )

  target_link_libraries(grasplink_gui PUBLIC
    grasplink_robotics
    grasplink_simulation
    grasplink_graphics
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
    apps/viewer/PickPlaceScenario.cpp
  )

  add_executable(grasplink_simulator
    ${GRASPLINK_APP_SOURCES}
  )

  target_include_directories(grasplink_simulator PRIVATE
    apps/viewer
  )

  target_link_libraries(grasplink_simulator PRIVATE
    grasplink_gui
    grasplink_application
    grasplink_simulation
    grasplink_viewer
    grasplink_robotics
    grasplink_physics
    grasplink_diagnostics
  )
  target_compile_definitions(grasplink_simulator PRIVATE
    GRASPLINK_ENABLE_TRACY=$<BOOL:${GRASPLINK_ENABLE_TRACY}>
  )
  if(GRASPLINK_ENABLE_TRACY)
    target_link_libraries(grasplink_simulator PRIVATE Tracy::TracyClient)
  endif()


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
      ${CMAKE_SOURCE_DIR}/assets/HCR12A_2F-85.glb
      $<TARGET_FILE_DIR:grasplink_simulator>/HCR12A_2F-85.glb

    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${CMAKE_SOURCE_DIR}/assets/plane.glb
      $<TARGET_FILE_DIR:grasplink_simulator>/plane.glb
  )

  if(BUILD_TESTING)
    grasplink_add_test(grasplink_gui_composition_tests GuiComposition tests/GuiCompositionTests.cpp grasplink_gui)
    grasplink_add_test(grasplink_quaternion_transform_tests QuaternionTransforms tests/QuaternionTransformTests.cpp grasplink_viewer)
    grasplink_add_test(grasplink_graphics_tests GraphicsResources tests/GraphicsResourceTests.cpp grasplink_graphics)
    set_tests_properties(GraphicsResources PROPERTIES TIMEOUT 60)

    grasplink_add_test(grasplink_robot_pose_integration_tests RobotPoseIntegration tests/RobotPoseIntegrationTests.cpp grasplink_viewer)
    set_tests_properties(RobotPoseIntegration PROPERTIES WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

    add_test(NAME ViewerSmoke COMMAND grasplink_simulator --smoke-test)
    set_tests_properties(ViewerSmoke PROPERTIES
      WORKING_DIRECTORY "$<TARGET_FILE_DIR:grasplink_simulator>" TIMEOUT 60)

  endif()

endif()

if(BUILD_TESTING AND GRASPLINK_BUILD_BENCHMARKS)
  add_executable(grasplink_robotics_benchmarks benchmarks/RoboticsBenchmarks.cpp)
  target_link_libraries(grasplink_robotics_benchmarks PRIVATE
    grasplink_robotics
    benchmark::benchmark
  )
  add_test(NAME RoboticsBenchmarks COMMAND grasplink_robotics_benchmarks --benchmark_min_time=0.01s)
  set_tests_properties(RoboticsBenchmarks PROPERTIES
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    TIMEOUT 30
    FAIL_REGULAR_EXPRESSION "ERROR OCCURRED;inf/s;nan/s;NaN")
endif()
