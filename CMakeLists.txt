cmake_minimum_required(VERSION 3.5)
project(orobotix_user_control)

set(CMAKE_CXX_STANDARD 11)
# source code directory
SET(SRC .)
SET(UDPUSER_INC_DIR ${SRC}/udpUser)

INCLUDE_DIRECTORIES(${UDPUSER_INC_DIR})

set(SOURCE_FILES main.cpp ${UDPUSER_INC_DIR}/CUDPCommUser.cpp)
add_executable(orobotix_user_control ${SOURCE_FILES})