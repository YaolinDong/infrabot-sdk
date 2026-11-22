cmake_minimum_required(VERSION 3.16)
project(infrabot_sdk LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(INFRABOT_IK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/infrabot_ik")
add_library(infrabot_ik_native STATIC
    "${INFRABOT_IK_DIR}/src/kinematics.cpp"
    "${INFRABOT_IK_DIR}/src/ik_solver.cpp")
target_include_directories(infrabot_ik_native PUBLIC "$<BUILD_INTERFACE:${INFRABOT_IK_DIR}/include>")
target_compile_options(infrabot_ik_native PRIVATE -O2)

