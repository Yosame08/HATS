cmake_minimum_required(VERSION 3.28)
project(1_Preprocess)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -pthread")
set(COMMON_DIR "../Common")

include_directories(${COMMON_DIR})
add_executable(4_Solve main.cpp
        ${COMMON_DIR}/solve.cpp ${COMMON_DIR}/graph.cpp
        ${COMMON_DIR}/geography.cpp ${COMMON_DIR}/structs.cpp
        ${COMMON_DIR}/road.cpp ${COMMON_DIR}/safeIO.cpp
        ${COMMON_DIR}/traffic.cpp ${COMMON_DIR}/predictor.cpp
        ${COMMON_DIR}/light.cpp ${COMMON_DIR}/progress.cpp
        ${COMMON_DIR}/general.cpp ${COMMON_DIR}/criterion.cpp
        ${COMMON_DIR}/csv.cpp ${COMMON_DIR}/feature.cpp
)

set(Torch_DIR /home/yosame/libtorch/share/cmake/Torch)
find_package(Torch REQUIRED)
target_link_libraries(4_Solve "${TORCH_LIBRARIES}")
