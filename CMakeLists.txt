cmake_minimum_required(VERSION 3.16)

project(indi_starpoint_focuser C CXX)

include(GNUInstallDirs)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

find_package(INDI 2.0 COMPONENTS driver REQUIRED)
if(NOT INDI_JSONLIB)
    find_package(nlohmann_json 3 REQUIRED)
    add_definitions(-D_USE_SYSTEM_JSONLIB)
endif()

# Set the version number in config.h and the driver's XML file
set(CDRIVER_VERSION_MAJOR 2)
set(CDRIVER_VERSION_MINOR 0)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/indi_starpoint_focuser.xml.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/indi_starpoint_focuser.xml
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${INDI_INCLUDE_DIR})

include(CMakeCommon)

add_executable(
    indi_starpoint_focuser
    indi_starpoint_focuser.cpp
)

target_link_libraries(
    indi_starpoint_focuser
    ${INDI_LIBRARIES}
)

install(TARGETS indi_starpoint_focuser RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/indi_starpoint_focuser.xml
    DESTINATION
        ${INDI_DATA_DIR}
)
