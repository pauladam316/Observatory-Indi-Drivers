cmake_minimum_required(VERSION 3.5)
project(ObservatoryINDIDrivers VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Option to build all drivers or individual drivers
option(BUILD_ALL_DRIVERS "Build all drivers" ON)
option(BUILD_TELESCOPE_CONTROLLER "Build Telescope Controller driver" ON)
option(BUILD_ROOF_CONTROLLER "Build Roof Controller driver" ON)

# Find INDI include directory
find_path(INDI_INCLUDE_DIR
    NAMES indidriver.h
    PATHS
    /usr/include/libindi
    /usr/local/include/libindi
    /opt/local/include/libindi
    ${CMAKE_SOURCE_DIR}/../libindi
    ENV INDI_DIR
    PATH_SUFFIXES include/libindi libindi
)

# Find INDI library
find_library(INDI_LIBRARIES
    NAMES indidriver
    PATHS
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    /usr/lib/x86_64-linux-gnu
    ${CMAKE_SOURCE_DIR}/../libindi
    ENV INDI_DIR
    PATH_SUFFIXES lib
)

if(NOT INDI_INCLUDE_DIR)
    message(FATAL_ERROR "INDI include directory not found. Please install libindi-dev package or set INDI_DIR environment variable.")
endif()

if(NOT INDI_LIBRARIES)
    message(FATAL_ERROR "INDI library not found. Please install libindi-dev package or set INDI_DIR environment variable.")
endif()

message(STATUS "Found INDI include dir: ${INDI_INCLUDE_DIR}")
message(STATUS "Found INDI library: ${INDI_LIBRARIES}")

# Include directories
# INDI headers are typically in libindi subdirectory
set(INDI_INCLUDE_DIRS ${INDI_INCLUDE_DIR})
# Also include parent directory for connection plugins (connectionplugins/ is usually at parent level)
get_filename_component(INDI_PARENT_DIR ${INDI_INCLUDE_DIR} DIRECTORY)
if(INDI_PARENT_DIR)
    list(APPEND INDI_INCLUDE_DIRS ${INDI_PARENT_DIR})
endif()

# Build drivers
if(BUILD_ALL_DRIVERS OR BUILD_TELESCOPE_CONTROLLER)
    add_subdirectory(drivers/telescope_controller)
endif()

if(BUILD_ALL_DRIVERS OR BUILD_ROOF_CONTROLLER)
    add_subdirectory(drivers/roof_controller)
endif()

# Print build configuration
message(STATUS "")
message(STATUS "=== Build Configuration ===")
message(STATUS "Build all drivers: ${BUILD_ALL_DRIVERS}")
message(STATUS "Build Telescope Controller: ${BUILD_TELESCOPE_CONTROLLER}")
message(STATUS "Build Roof Controller: ${BUILD_ROOF_CONTROLLER}")
message(STATUS "")
