cmake_minimum_required(VERSION 3.9)
project(GNG_SAM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(src/include)

# Source
set(
  SOURCES 
  main.cpp
)

add_executable(gng_sam ${SOURCES})

# Tests 
add_executable(dummy test/dummy_test.cpp)
add_test(dummy_test dummy)
