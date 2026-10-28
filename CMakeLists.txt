cmake_minimum_required(VERSION 3.16)

project(indi-starpoint C CXX)

include(GNUInstallDirs)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

find_package(INDI 2.0 COMPONENTS driver REQUIRED)

# Bluetooth goes through BlueZ on the system's D-Bus, with systemd's sd-bus
find_package(PkgConfig REQUIRED)
pkg_check_modules(SYSTEMD REQUIRED IMPORTED_TARGET libsystemd)

if(NOT INDI_JSONLIB)
    find_package(nlohmann_json 3 REQUIRED)
    add_definitions(-D_USE_SYSTEM_JSONLIB)
endif()

# Set the version number in config.h and the drivers' XML file
set(CDRIVER_VERSION_MAJOR 2)
set(CDRIVER_VERSION_MINOR 0)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/indi_starpoint.xml.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/indi_starpoint.xml
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${INDI_INCLUDE_DIR})

include(CMakeCommon)

# Code shared by all the Starpoint Australis drivers
add_library(
    starpoint_common STATIC
    starpoint_bluetooth.cpp
    starpoint_protocol.cpp
)

target_link_libraries(
    starpoint_common
    ${INDI_LIBRARIES}
    PkgConfig::SYSTEMD
)

# Focuser
add_executable(
    indi_starpoint_focuser
    starpoint_focuser.cpp
)

target_link_libraries(
    indi_starpoint_focuser
    starpoint_common
    ${INDI_LIBRARIES}
)

# Filter wheel
add_executable(
    indi_starpoint_filterwheel
    starpoint_filterwheel.cpp
)

target_link_libraries(
    indi_starpoint_filterwheel
    starpoint_common
    ${INDI_LIBRARIES}
)

install(TARGETS indi_starpoint_filterwheel RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(TARGETS indi_starpoint_focuser RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/indi_starpoint.xml
    DESTINATION
        ${INDI_DATA_DIR}
)
