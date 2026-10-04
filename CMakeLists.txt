cmake_minimum_required(VERSION 3.20)

project(GraspLink VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/utf-8)
endif()

# Hardware/Simulation 공통 Controller 계층.
# Viewer/Flecs/GLM에 의존하지 않고 model specification + backend contract를 제공한다.
add_library(grasplink_control STATIC
  modules/control/src/simulation/SimRobotController.cpp
)
target_include_directories(grasplink_control PUBLIC
  modules/control/include
)
target_sources(grasplink_control PUBLIC
  modules/control/include/control/ControlTypes.h
  modules/control/include/control/IRobotController.h
  modules/control/include/control/IGripperController.h
  modules/control/include/control/specs/DeviceSpecifications.h
  modules/control/include/control/specs/Hcr12aSpecification.h
  modules/control/include/control/specs/Robotiq2F85Specification.h
  modules/control/include/control/simulation/SimRobotController.h
)

option(GRASPLINK_BUILD_GRAPHICS "Build the OpenGL viewer" ON)

if(GRASPLINK_BUILD_GRAPHICS)
  include(cmake/Dependencies.cmake)
  grasplink_add_graphics_dependencies()
  grasplink_add_graphics_library()

  set(GRASPLINK_VIEWER_SOURCES
    modules/viewer/src/Entity.cpp
    modules/viewer/src/systems/RenderSystemModule.cpp
    modules/viewer/src/systems/TransformSystemModule.cpp
    modules/viewer/src/scene/Scene.cpp
    modules/viewer/src/scene/SceneManager.cpp
    modules/viewer/src/assets/GltfLoader.cpp
    modules/viewer/src/assets/AssetManager.cpp
    modules/viewer/src/assets/PrefabFactory.cpp
    modules/viewer/src/robot/RobotTransformAdapter.cpp
  )
  set(GRASPLINK_VIEWER_HEADERS
    modules/viewer/include/Entity.h
    modules/viewer/include/components/Components.h
    modules/viewer/include/components/GripperComponents.h
    modules/viewer/include/components/RenderComponents.h
    modules/viewer/include/components/RobotComponents.h
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
    modules/viewer/include/robot/RobotTransformAdapter.h
  )

  add_library(grasplink_viewer STATIC
    ${GRASPLINK_VIEWER_SOURCES}
    ${GRASPLINK_VIEWER_HEADERS}
  )
  target_include_directories(grasplink_viewer PUBLIC
    modules/common/include
    modules/viewer/include
    modules/viewer/include/components
    modules/viewer/include/graphics
    modules/viewer/include/systems
  )
  target_link_libraries(grasplink_viewer PUBLIC
    grasplink_control
    grasplink_graphics
    flecs::flecs_static

    PRIVATE
      tinygltf
  )
  target_compile_definitions(grasplink_viewer PRIVATE
    GLM_ENABLE_EXPERIMENTAL
  )

  set(GRASPLINK_APP_SOURCES
    apps/viewer/main.cpp
    apps/viewer/ViewerApp.cpp
  )
  set(GRASPLINK_APP_HEADERS
    apps/viewer/ViewerApp.h
  )

  add_executable(grasplink_simulator
    ${GRASPLINK_APP_SOURCES}
    ${GRASPLINK_APP_HEADERS}
  )
  target_include_directories(grasplink_simulator PRIVATE
    apps/viewer
  )
  target_link_libraries(grasplink_simulator PRIVATE
    grasplink_viewer
  )

  add_custom_command(TARGET grasplink_simulator POST_BUILD
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
