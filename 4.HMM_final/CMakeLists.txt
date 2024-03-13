cmake_minimum_required(VERSION 3.27)
project(HMM_final)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -pthread")

set(COMMON_DIR "../Common")
include_directories(${COMMON_DIR})

# set(CMAKE_PREFIX_PATH "/home/yosame/libtorch/share/cmake/Torch") #pytorch自动提示
find_package(Torch REQUIRED)
# include_directories("/home/yosame/libtorch/include")
# include_directories("/home/yosame/libtorch/include/torch/csrc/api/include")

add_executable(HMM_final main.cpp predict.cpp
        ${COMMON_DIR}/maths.cpp ${COMMON_DIR}/reads.cpp
        ${COMMON_DIR}/structs.cpp ${COMMON_DIR}/definitions.cpp
        ${COMMON_DIR}/traffic.cpp ${COMMON_DIR}/FuncEst.cpp
)
target_link_libraries(HMM_final "${TORCH_LIBRARIES}")
set_property(TARGET HMM_final PROPERTY CXX_STANDARD 17)
