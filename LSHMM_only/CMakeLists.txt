cmake_minimum_required(VERSION 3.27)
project(LSHMM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread")

set(COMMON_DIR "../Common")
include_directories(${COMMON_DIR})

add_executable(LSHMM main.cpp
        ${COMMON_DIR}/maths.cpp ${COMMON_DIR}/reads.cpp
        ${COMMON_DIR}/structs.cpp ${COMMON_DIR}/definitions.cpp
)
set_property(TARGET LSHMM PROPERTY CXX_STANDARD 17)
