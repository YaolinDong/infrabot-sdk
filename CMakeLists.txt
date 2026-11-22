# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
cmake_minimum_required(VERSION 3.16)
project(infrabot_sdk VERSION 0.1.0 LANGUAGES C CXX)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Keep the old cache options usable for one release, including existing builds.
foreach(feature TESTS PYTHON GUI EXAMPLES)
    if(DEFINED UDP_BUILD_${feature})
        message(DEPRECATION "Use INFRABOT_BUILD_${feature} instead of UDP_BUILD_${feature}")
        set(INFRABOT_BUILD_${feature} "${UDP_BUILD_${feature}}" CACHE BOOL "Build ${feature}" FORCE)
        unset(UDP_BUILD_${feature} CACHE)
    endif()
endforeach()
option(INFRABOT_BUILD_TESTS "Build native and loopback tests" OFF)
option(INFRABOT_BUILD_PYTHON "Build pybind11 Python bindings" OFF)
option(INFRABOT_BUILD_GUI "Build the Qt control GUI when Qt is available" ON)
option(INFRABOT_BUILD_EXAMPLES "Build the C++ SDK examples" OFF)
set(INFRABOT_FIRMWARE_DIR "$ENV{INFRABOT_FIRMWARE_DIR}" CACHE PATH "Optional firmware checkout for cross-repository tests")
set(INFRABOT_DESCRIPTION_DIR "" CACHE PATH "Optional default infrabot-description package directory for the GUI")
set(INFRABOT_IK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/infrabot_ik")
find_package(Threads REQUIRED)
if(INFRABOT_BUILD_TESTS)
    enable_testing()
    add_executable(test_motion_session tests/test_motion_session.cpp)
    target_include_directories(test_motion_session PRIVATE src include)
    target_compile_options(test_motion_session PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
    add_test(NAME motion_session COMMAND test_motion_session)
    add_executable(test_joint_stream tests/test_joint_stream.cpp)
    target_include_directories(test_joint_stream PRIVATE src)
    target_link_libraries(test_joint_stream PRIVATE infrabot_sdk)
    target_compile_options(test_joint_stream PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
    add_test(NAME joint_stream COMMAND test_joint_stream)
endif()



# Forward kinematics for the SDK (NRT Cartesian hold, Robot::forward) and
# the GUI's Cartesian controls.
add_library(infrabot_ik_native STATIC
    "${INFRABOT_IK_DIR}/src/kinematics.cpp"
    "${INFRABOT_IK_DIR}/src/ik_solver.cpp")
target_include_directories(infrabot_ik_native PUBLIC "$<BUILD_INTERFACE:${INFRABOT_IK_DIR}/include>")
target_compile_options(infrabot_ik_native PRIVATE -O2)

# The SDK: the one implementation of the UDP protocol (include/infrabot/
# protocol.h, shared with the MCU firmware), used by the GUI and the tools.
find_package(OpenSSL REQUIRED COMPONENTS Crypto)
add_library(infrabot_sdk STATIC src/firmware_update.cpp src/robot.cpp src/recorder.cpp src/serial_log.cpp src/run_log.cpp src/diagnostics.cpp)
# Same public trust anchor for native callers that do not pass a public key file.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/firmware/release.pub")
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/firmware/release.pub" release_public LIMIT_COUNT 1)
string(LENGTH "${release_public}" pub_len)
if(NOT pub_len EQUAL 64 OR NOT release_public MATCHES "^[0-9A-Fa-f]+$")
  message(FATAL_ERROR "Invalid firmware/release.pub")
endif()
target_compile_definitions(infrabot_sdk PRIVATE INFRABOT_RELEASE_PUBLIC="${release_public}")
target_include_directories(infrabot_sdk PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
target_compile_options(infrabot_sdk PRIVATE -Wall -Wextra -Wpedantic -O2)
target_link_libraries(infrabot_sdk PRIVATE infrabot_ik_native Threads::Threads OpenSSL::Crypto)
add_library(infrabot_sdk::infrabot_sdk ALIAS infrabot_sdk)
set_target_properties(infrabot_sdk infrabot_ik_native PROPERTIES POSITION_INDEPENDENT_CODE ON)
install(TARGETS infrabot_sdk infrabot_ik_native EXPORT infrabot_sdkTargets
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}")
install(DIRECTORY include/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
set(config_dir "${CMAKE_INSTALL_LIBDIR}/cmake/infrabot_sdk")
install(EXPORT infrabot_sdkTargets NAMESPACE infrabot_sdk:: DESTINATION "${config_dir}")
configure_package_config_file(cmake/infrabot_sdkConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/infrabot_sdkConfig.cmake" INSTALL_DESTINATION "${config_dir}")
# Before 1.0 a minor release may break the API, so 0.x only matches the same minor.
if(PROJECT_VERSION_MAJOR EQUAL 0)
    set(infrabot_sdk_compatibility SameMinorVersion)
else()
    set(infrabot_sdk_compatibility SameMajorVersion)
endif()
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/infrabot_sdkConfigVersion.cmake"
    VERSION "${PROJECT_VERSION}" COMPATIBILITY ${infrabot_sdk_compatibility})
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/infrabot_sdkConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/infrabot_sdkConfigVersion.cmake" DESTINATION "${config_dir}")

install(FILES LICENSE THIRD_PARTY.md DESTINATION "${CMAKE_INSTALL_DATADIR}/infrabot_sdk")
install(DIRECTORY LICENSES/ DESTINATION "${CMAKE_INSTALL_DATADIR}/infrabot_sdk/LICENSES")
install(FILES third_party/infrabot_ik/README.md third_party/infrabot_ik/LICENSE
    DESTINATION "${CMAKE_INSTALL_DATADIR}/infrabot_sdk/third_party/infrabot_ik")

include(cmake/legacy.cmake OPTIONAL)
