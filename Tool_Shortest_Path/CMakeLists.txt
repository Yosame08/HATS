cmake_minimum_required(VERSION 3.22)
project(Tool_Shortest_Path)

set(CMAKE_CXX_STANDARD 17)
set(COMMON_DIR "../Common")
include_directories(${COMMON_DIR})

add_executable(Tool_Shortest_Path main.cpp ${COMMON_DIR}/reads.cpp ${COMMON_DIR}/structs.cpp
                                           ${COMMON_DIR}/maths.cpp ${COMMON_DIR}/definitions.cpp)
