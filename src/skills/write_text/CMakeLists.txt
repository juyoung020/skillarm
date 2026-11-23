# write_text skill: pick up a pencil and write text on paper (SO-101). Standalone build next to the main project:
#   cmake -S src/skills/write_text -B build/write_text -DCMAKE_BUILD_TYPE=Release && cmake --build build/write_text -j
# Targets: skillarm_write (static core), libskillarm_write.so (C ABI), skillarm-write (CLI), skillarm-write-server.
cmake_minimum_required(VERSION 3.20)
project(skillarm_write LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_options(-Wall -Wextra -O2)

get_filename_component(SKILLARM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../.. ABSOLUTE)
set(MUJOCO_DIR ${SKILLARM_ROOT}/third_party/mujoco)
add_library(wt_mujoco SHARED IMPORTED)
set_target_properties(wt_mujoco PROPERTIES
  IMPORTED_LOCATION ${MUJOCO_DIR}/lib/libmujoco.so.4.0.1
  INTERFACE_INCLUDE_DIRECTORIES ${MUJOCO_DIR}/include)
set(CMAKE_BUILD_RPATH ${MUJOCO_DIR}/lib)
find_package(Threads REQUIRED)
find_package(OpenGL COMPONENTS OpenGL EGL)

add_library(skillarm_write STATIC
  font.cpp scene.cpp arm.cpp planner.cpp twin.cpp executor.cpp report.cpp api.cpp stb_write.cpp
  ${SKILLARM_ROOT}/servo/rt_client.cpp real_io.cpp)
target_include_directories(skillarm_write PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${SKILLARM_ROOT}/include
  ${SKILLARM_ROOT}/third_party ${SKILLARM_ROOT}/servo)
target_compile_definitions(skillarm_write PUBLIC SKILLARM_ROOT="${SKILLARM_ROOT}")
target_link_libraries(skillarm_write PUBLIC wt_mujoco Threads::Threads)

if(OpenGL_EGL_FOUND)
  target_sources(skillarm_write PRIVATE render.cpp)
  target_compile_definitions(skillarm_write PUBLIC WT_RENDER=1)
  target_link_libraries(skillarm_write PUBLIC OpenGL::OpenGL OpenGL::EGL)
endif()

add_library(skillarm_write_c SHARED capi.cpp)
set_target_properties(skillarm_write_c PROPERTIES OUTPUT_NAME skillarm_write)
target_link_libraries(skillarm_write_c PRIVATE skillarm_write)

add_executable(skillarm-write cli.cpp)
target_link_libraries(skillarm-write PRIVATE skillarm_write)

add_executable(skillarm-write-server server.cpp)
target_link_libraries(skillarm-write-server PRIVATE skillarm_write)
