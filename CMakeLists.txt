cmake_minimum_required(VERSION 3.16)
project(Autopilot_EClub VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_TESTS "Build test suite" ON)
option(STM32_TARGET "Target STM32 microcontroller (ARM Cortex-M)" OFF)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/hal
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/fusion
    ${CMAKE_CURRENT_SOURCE_DIR}/math
    ${CMAKE_CURRENT_SOURCE_DIR}/control
    ${CMAKE_CURRENT_SOURCE_DIR}/vehicles
    ${CMAKE_CURRENT_SOURCE_DIR}/system
)

# Core sources (Architecture and algorithms)
set(CORE_SOURCES
    math/vector3.hpp
    math/quaternion.hpp
    math/matrix3x3.hpp
    math/low_pass_filter.hpp
    math/pid.hpp
    hal/posix/posix_hal.cpp
    hal/stm32/stm32_hal.cpp
    hal/stm32/stm32_pwm.cpp
    drivers/sensor_manager.cpp
    drivers/imu/mpu6050.cpp
    drivers/imu/icm20689.cpp
    drivers/baro/bmp280.cpp
    drivers/mag/qmc5883l.cpp
    drivers/gps/nmea.cpp
    fusion/madgwick.cpp
    fusion/complementary.cpp
    fusion/ahrs_interface.cpp
    control/arming_handler.cpp
    vehicles/vehicle_base.cpp
    vehicles/uav/uav_vehicle.cpp
    vehicles/ugv/ugv_vehicle.cpp
    vehicles/rov/rov_vehicle.cpp
    system/scheduler.cpp
    system/telemetry.cpp
)

add_library(autopilot_core STATIC ${CORE_SOURCES})

# Target Executable
add_executable(autopilot_app system/main.cpp)
target_link_libraries(autopilot_app PRIVATE autopilot_core pthread)

if(BUILD_TESTS)
    enable_testing()

    add_executable(test_pnp_sensors tests/test_pnp_sensors.cpp)
    target_link_libraries(test_pnp_sensors PRIVATE autopilot_core pthread)
    add_test(NAME test_pnp_sensors COMMAND test_pnp_sensors)

    add_executable(test_sensor_fusion tests/test_sensor_fusion.cpp)
    target_link_libraries(test_sensor_fusion PRIVATE autopilot_core pthread)
    add_test(NAME test_sensor_fusion COMMAND test_sensor_fusion)

    add_executable(test_vehicle_controls tests/test_vehicle_controls.cpp)
    target_link_libraries(test_vehicle_controls PRIVATE autopilot_core pthread)
    add_test(NAME test_vehicle_controls COMMAND test_vehicle_controls)
endif()
