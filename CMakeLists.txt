cmake_minimum_required(VERSION 3.20)

project(
  PoseLink
  VERSION 0.1.0
  LANGUAGES C CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/utf-8)
endif()

option(
  POSELINK_BUILD_GRAPHICS
  "Build PoseLink OpenGL viewer"
  ON
)

add_library(
  poselink_vision
  STATIC

  modules/vision/include/IPoseSource.h
  modules/vision/include/SyntheticPoseSource.h
  modules/vision/src/SyntheticPoseSource.cpp
)

target_include_directories(
  poselink_vision
  PUBLIC
    modules/common/include
    modules/vision/include
)

if(POSELINK_BUILD_GRAPHICS)

  include(cmake/Dependencies.cmake)

  poselink_add_graphics_library(
    modules/viewer/include/RenderContext.h)


  # --------------------------------------------------
  # Viewer Application
  # --------------------------------------------------

  add_executable(
    poselink_viewer

    apps/viewer/main.cpp
    apps/viewer/ViewerApp.cpp
    apps/viewer/ViewerApp.h

    # ECS wrapper
    modules/viewer/include/Entity.h

    # Viewer ECS System
    modules/viewer/include/systems/RenderSystem.h
    modules/viewer/src/systems/RenderSystem.cpp
  )


  target_include_directories(
    poselink_viewer
    PRIVATE
      apps/viewer
      modules/viewer/include
      modules/viewer/include/components
      modules/viewer/include/graphics
      modules/viewer/include/systems
  )


  target_link_libraries(
    poselink_viewer
    PRIVATE
      poselink_graphics
      poselink_vision
      flecs::flecs_static
  )


  # --------------------------------------------------
  # Runtime Assets
  # --------------------------------------------------

  add_custom_command(
    TARGET poselink_viewer
    POST_BUILD

    COMMAND ${CMAKE_COMMAND} -E copy_directory
      ${CMAKE_SOURCE_DIR}/assets/shaders
      $<TARGET_FILE_DIR:poselink_viewer>/shaders
  )

endif()