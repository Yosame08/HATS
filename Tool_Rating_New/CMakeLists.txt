cmake_minimum_required(VERSION 3.22)
project(Tool_Rating_New)

set(CMAKE_CXX_STANDARD 17)
set(COMMON_DIR "../Common")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os")
include_directories(${COMMON_DIR})

add_executable(Tool_Rating_New main.cpp ${COMMON_DIR}/reads.cpp ${COMMON_DIR}/structs.cpp
                                        ${COMMON_DIR}/maths.cpp ${COMMON_DIR}/definitions.cpp)
