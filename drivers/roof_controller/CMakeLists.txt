# CMakeLists.txt for Roof Controller driver

set(DRIVER_NAME "RoofControllerDriver")
set(DRIVER_SOURCES
    roof_controller.cpp
)
set(DRIVER_HEADERS
    roof_controller.h
)

# Create the driver executable
add_executable(${DRIVER_NAME} ${DRIVER_SOURCES} ${DRIVER_HEADERS})

# Link INDI library (defined in parent)
target_link_libraries(${DRIVER_NAME} ${INDI_LIBRARIES})

# Include directories (defined in parent)
target_include_directories(${DRIVER_NAME} PRIVATE ${INDI_INCLUDE_DIRS})

# Install the driver
install(TARGETS ${DRIVER_NAME}
    RUNTIME DESTINATION bin
)

