cmake_minimum_required(VERSION 3.20)
project(PoseLink VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/utf-8)
endif()

option(POSELINK_BUILD_GRAPHICS "Build the existing OpenGL graphics module" ON)

if(POSELINK_BUILD_GRAPHICS)
  include(cmake/Dependencies.cmake)
  poselink_add_graphics_library()

  add_executable(poselink_viewer
    apps/viewer/main.cpp
    apps/viewer/ViewerApp.cpp
    modules/viewer/include/IndexBuffer.h
    modules/viewer/src/IndexBuffer.cpp)

  target_include_directories(poselink_viewer PRIVATE
    apps/viewer)

  target_link_libraries(poselink_viewer PRIVATE
    poselink_graphics)

  add_custom_command(TARGET poselink_viewer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
      ${CMAKE_SOURCE_DIR}/assets/shaders
      $<TARGET_FILE_DIR:poselink_viewer>/shaders)
endif()

# Add poselink_vision_node and the networking modules only after the local
# Synthetic Pose -> Viewer vertical slice is complete.
