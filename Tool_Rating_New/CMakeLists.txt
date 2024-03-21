cmake_minimum_required(VERSION 3.27)
project(Tool_Rating_New)

set(CMAKE_CXX_STANDARD 17)
set(COMMON_DIR "../Common")
include_directories(${COMMON_DIR})

add_executable(Tool_Rating_New main.cpp ${COMMON_DIR}/reads.cpp)
