cmake_minimum_required(VERSION 3.5)
project(TelescopeControllerDriver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
include_directories(${INDI_INCLUDE_DIR})
# Also include parent directory for connection plugins (connectionplugins/ is usually at parent level)
get_filename_component(INDI_PARENT_DIR ${INDI_INCLUDE_DIR} DIRECTORY)
if(INDI_PARENT_DIR)
    include_directories(${INDI_PARENT_DIR})
endif()

# Source files
set(SOURCES
    telescope_controller.cpp
)

# Header files
set(HEADERS
    telescope_controller.h
)

# Create the driver
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Link libraries
target_link_libraries(${PROJECT_NAME} ${INDI_LIBRARIES})

# Install the driver
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)

