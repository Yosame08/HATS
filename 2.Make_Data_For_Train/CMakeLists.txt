cmake_minimum_required(VERSION 3.22)
project(Make_Data)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -pthread")

set(COMMON_DIR "../Common")
include_directories(${COMMON_DIR})

add_executable(Make_Data main.cpp
        ${COMMON_DIR}/traffic.cpp ${COMMON_DIR}/opCSV.cpp
        ${COMMON_DIR}/traffic.cpp ${COMMON_DIR}/FuncEst.cpp
        ${COMMON_DIR}/funcIO.cpp
)
