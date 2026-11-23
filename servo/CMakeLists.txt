# Robot-side servers (Linux; the PC next to the arm or the Raspberry Pi) and the push-stream client:
#   cmake -S servo -B servo/build -DCMAKE_BUILD_TYPE=Release && cmake --build servo/build -j4
cmake_minimum_required(VERSION 3.20)
project(skillarm_servo LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-O2 -Wall -Wextra)
find_package(Threads REQUIRED)

# SKW1 push-stream client (rt::ArmStream / rt::CameraStream); also used by skillarm_real
add_library(skillarm_rt STATIC rt_client.cpp)
target_include_directories(skillarm_rt PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../third_party)
target_link_libraries(skillarm_rt PUBLIC Threads::Threads)

# Servo server (Feetech bus) on :7755
add_executable(skillarm-servo server.cpp feetech.cpp)
target_link_libraries(skillarm-servo PRIVATE skillarm_rt)

# Camera server (wrist C270, V4L2) on :7756; pairs frames with the servo stream
add_executable(skillarm-camera camera.cpp)
target_link_libraries(skillarm-camera PRIVATE skillarm_rt)

# Read-only I/O timing (legacy JSON vs push streams, camera lag)
add_executable(skillarm-rtbench rt_bench.cpp)
target_link_libraries(skillarm-rtbench PRIVATE skillarm_rt)

# Simulated STS3215 bus on a pty, for testing the server without the arm
add_executable(skillarm-fake-arm fake_arm.cpp)
