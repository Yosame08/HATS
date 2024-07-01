cmake_minimum_required(VERSION 3.28)
project(3_FeatureLearning)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -pthread")
set(COMMON_DIR "../Common")

include_directories(${COMMON_DIR})
add_executable(3_FeatureLearning main.cpp dataset.cpp
        ${COMMON_DIR}/csv.cpp ${COMMON_DIR}/safeIO.cpp
        ${COMMON_DIR}/light.cpp ${COMMON_DIR}/general.cpp
        ${COMMON_DIR}/traffic.cpp ${COMMON_DIR}/structs.cpp
        ${COMMON_DIR}/feature.cpp ${COMMON_DIR}/geography.cpp
)
