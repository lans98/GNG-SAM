cmake_minimum_required(VERSION 3.9)
project(GNG_SAM)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(src/include)

# Source
set(SOURCES src/main.cpp)

# Executables
add_executable(gng_sam ${SOURCES})

# Executable test
add_executable(point_test test/point.cpp)
target_link_libraries (gng_sam ${PCL_LIBRARIES})


