# CMakeLists.txt for Telescope Controller driver

set(DRIVER_NAME "TelescopeControllerDriver")
set(DRIVER_SOURCES
    telescope_controller.cpp
)
set(DRIVER_HEADERS
    telescope_controller.h
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

