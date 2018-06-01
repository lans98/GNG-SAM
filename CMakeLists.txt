cmake_minimum_required(VERSION 3.9)
project(GNG_SAM)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(src/include)

# Source
set(SOURCES src/main.cpp)

# Executables
add_executable(gng_sam ${SOURCES})

# Executable test
add_executable(point_test test/point.cpp)
