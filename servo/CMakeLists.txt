# Servo server for the Raspberry Pi (Linux). Build on the Pi:
#   cmake -S servo -B servo/build -DCMAKE_BUILD_TYPE=Release && cmake --build servo/build -j4
cmake_minimum_required(VERSION 3.20)
project(skillarm_servo LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O2 -Wall -Wextra)
add_executable(skillarm-servo server.cpp feetech.cpp)
target_include_directories(skillarm-servo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../third_party)
