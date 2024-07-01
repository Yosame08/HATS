cmake_minimum_required(VERSION 3.22)
project(Tool_Rating)

set(CMAKE_CXX_STANDARD 20)
set(COMMON_DIR "../Common")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os")
include_directories(${COMMON_DIR})

add_executable(Tool_Rating main.cpp ${COMMON_DIR}/graph.cpp ${COMMON_DIR}/structs.cpp
        ${COMMON_DIR}/safeIO.cpp ${COMMON_DIR}/geography.cpp
        ${COMMON_DIR}/graph.cpp ${COMMON_DIR}/road.cpp)
