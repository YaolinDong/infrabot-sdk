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
if(INFRABOT_BUILD_PYTHON)
    find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
    set(PYBIND11_FINDPYTHON ON)
    find_package(pybind11 3.0 CONFIG REQUIRED)
    pybind11_add_module(infrabot_native python/bindings.cpp)
    target_link_libraries(infrabot_native PRIVATE infrabot_sdk)
    set(python_root "${CMAKE_CURRENT_BINARY_DIR}/python")
    file(MAKE_DIRECTORY "${python_root}/infrabot")
    set_target_properties(infrabot_native PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${python_root}/infrabot")
    foreach(file __init__.py py.typed infrabot_native.pyi)
        configure_file(python/infrabot/${file} "${python_root}/infrabot/${file}" COPYONLY)
    endforeach()
    configure_file(python/infrabot_native.py "${python_root}/infrabot_native.py" COPYONLY)
    configure_file(firmware/release.pub "${python_root}/infrabot/release.pub" COPYONLY)
    set(python_site "${CMAKE_INSTALL_LIBDIR}/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
    install(TARGETS infrabot_native LIBRARY DESTINATION "${python_site}/infrabot" COMPONENT python)
    install(FILES python/infrabot/__init__.py python/infrabot/infrabot_native.pyi python/infrabot/py.typed
        firmware/release.pub DESTINATION "${python_site}/infrabot" COMPONENT python)
    install(FILES python/infrabot_native.py DESTINATION "${python_site}" COMPONENT python)
    if(INFRABOT_BUILD_TESTS)
        pybind11_add_module(_infrabot_test_mcu tests/bind_fake_mcu.cpp)
        target_include_directories(_infrabot_test_mcu PRIVATE src include)
        target_link_libraries(_infrabot_test_mcu PRIVATE pthread)
        target_compile_options(_infrabot_test_mcu PRIVATE -UNDEBUG)
        add_test(NAME robot_python_loopback COMMAND ${Python3_EXECUTABLE} -B
                 ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_robot_binding.py)
        add_test(NAME python_examples_loopback COMMAND ${Python3_EXECUTABLE} -B
                 ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_examples.py)
        set_tests_properties(robot_python_loopback python_examples_loopback PROPERTIES
            ENVIRONMENT "PYTHONPATH=${python_root}:$<TARGET_FILE_DIR:_infrabot_test_mcu>" TIMEOUT 30)
    endif()
endif()

# Optional Qt Widgets front-end.  The command-line tests remain buildable on
# systems without Qt; install qtbase5-dev or qt6-base-dev to enable this GUI.
if(INFRABOT_BUILD_GUI)
    find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets QUIET)
endif()
if(INFRABOT_BUILD_GUI AND QT_FOUND)
    find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)
    # OpenGL 2.1 viewer for external infrabot-description meshes.
    # Without Qt Xml (Qt 6: also OpenGL
    # and OpenGLWidgets) the GUI is built without it.
    set(UDP_ARM_VIEW_LIBS "")
    if(QT_VERSION_MAJOR EQUAL 6)
        find_package(Qt6 QUIET COMPONENTS Xml OpenGL OpenGLWidgets)
        if(Qt6Xml_FOUND AND Qt6OpenGL_FOUND AND Qt6OpenGLWidgets_FOUND)
            set(UDP_ARM_VIEW_LIBS Qt6::Xml Qt6::OpenGL Qt6::OpenGLWidgets)
        endif()
    else()
        find_package(Qt5 QUIET COMPONENTS Xml)
        if(Qt5Xml_FOUND)
            set(UDP_ARM_VIEW_LIBS Qt5::Xml)
        endif()
    endif()
    if(UDP_ARM_VIEW_LIBS)
        add_library(udp_arm_view STATIC tools/gui/urdf_viewer.cpp)
        target_include_directories(udp_arm_view PUBLIC tools/gui)
        target_compile_options(udp_arm_view PRIVATE -O2)
        target_link_libraries(udp_arm_view PUBLIC Qt${QT_VERSION_MAJOR}::Widgets ${UDP_ARM_VIEW_LIBS})
        target_compile_definitions(udp_arm_view PUBLIC
            INFRABOT_HAS_ARM_VIEW INFRABOT_DEFAULT_DESCRIPTION_DIR="${INFRABOT_DESCRIPTION_DIR}")
    else()
        message(WARNING "Qt Xml not found: udp_control_gui is built without the 3D arm view")
    endif()
    add_executable(udp_control_gui tools/gui/qt_control_gui.cpp)
    target_include_directories(udp_control_gui PRIVATE src)
    target_compile_options(udp_control_gui PRIVATE -Wall -Wextra -Wpedantic -O2)
    target_link_libraries(udp_control_gui PRIVATE Qt${QT_VERSION_MAJOR}::Widgets pthread infrabot_sdk infrabot_ik_native)
    if(TARGET udp_arm_view)
        target_link_libraries(udp_control_gui PRIVATE udp_arm_view)
    endif()
    if(INFRABOT_BUILD_TESTS)
        add_executable(test_qt_motion tests/test_qt_motion.cpp)
        target_include_directories(test_qt_motion PRIVATE src)
        target_compile_options(test_qt_motion PRIVATE -Wall -Wextra -Wpedantic -Wno-unused-function -UNDEBUG)
        target_link_libraries(test_qt_motion PRIVATE Qt${QT_VERSION_MAJOR}::Widgets pthread infrabot_sdk infrabot_ik_native)
        if(TARGET udp_arm_view)
            target_link_libraries(test_qt_motion PRIVATE udp_arm_view)
        endif()
        add_test(NAME qt_motion_loopback COMMAND test_qt_motion)
        set_tests_properties(qt_motion_loopback PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen" TIMEOUT 90)
        if(TARGET udp_arm_view)
            add_test(NAME qt_without_description COMMAND test_qt_motion)
            set_tests_properties(qt_without_description PROPERTIES TIMEOUT 90
                ENVIRONMENT "QT_QPA_PLATFORM=offscreen;INFRABOT_DESCRIPTION_DIR=${CMAKE_CURRENT_BINARY_DIR}/missing-model;INFRABOT_TEST_MISSING_MODEL=1")
        endif()
    endif()
    # Template-only changes must also run the post-build deployment step.
    set_property(TARGET udp_control_gui APPEND PROPERTY LINK_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/config/infrabot.tsv"
        "${CMAKE_CURRENT_SOURCE_DIR}/config/control_template.tsv")
    # Keep the project-provided default motor template beside the GUI binary.
    # The GUI resolves it relative to the executable; no user file selection or
    # machine-specific absolute path is required for the default action.
    add_custom_command(TARGET udp_control_gui POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/config/infrabot.tsv"
            "$<TARGET_FILE_DIR:udp_control_gui>/infrabot.tsv"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/config/control_template.tsv"
            "$<TARGET_FILE_DIR:udp_control_gui>/control_template.tsv")
elseif(INFRABOT_BUILD_GUI)
    message(WARNING "Qt Widgets not found: udp_control_gui is not built (install qtbase5-dev or qt6-base-dev)")
endif()

add_executable(infrabot_update tools/update/update_firmware.cpp)
target_link_libraries(infrabot_update PRIVATE infrabot_sdk)

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
