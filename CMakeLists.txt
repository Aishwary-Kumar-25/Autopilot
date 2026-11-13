cmake_minimum_required(VERSION 3.16)
project(Autopilot_EClub VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_TESTS "Build test suite" ON)
option(STM32_TARGET "Target STM32 microcontroller (ARM Cortex-M)" OFF)

# Modular Driver Compilation Flags (avoids bloated compile times)
option(ENABLE_DRIVER_MPU6050  "Compile MPU6050 IMU Driver" ON)
option(ENABLE_DRIVER_ICM20689 "Compile ICM-20689 IMU Driver" ON)
option(ENABLE_DRIVER_BMP280   "Compile BMP280 Barometer Driver" ON)
option(ENABLE_DRIVER_QMC5883L "Compile QMC5883L Magnetometer Driver" ON)
option(ENABLE_DRIVER_NMEA_GPS "Compile NMEA GPS Driver" ON)
option(ENABLE_CAN_DRONECAN    "Compile DroneCAN Sensor Drivers" ON)

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
    hal/stm32/stm32_flash.cpp
    hal/flash_storage.cpp
    drivers/sensor_manager.cpp
    drivers/mag/compass_calibrator.cpp
    fusion/madgwick.cpp
    fusion/complementary.cpp
    fusion/ahrs_interface.cpp
    control/arming_handler.cpp
    control/failsafe.cpp
    control/actuator_matrix_mixer.cpp
    control/motor_failure_detector.cpp
    vehicles/vehicle_base.cpp
    vehicles/uav/uav_vehicle.cpp
    vehicles/ugv/ugv_vehicle.cpp
    vehicles/rov/rov_vehicle.cpp
    vehicles/vehicle_manager.cpp
    system/scheduler.cpp
    system/telemetry.cpp
    system/pubsub.hpp
    comms/mavlink/mavlink_protocol.cpp
    comms/mavlink/sha256.cpp
    comms/mavlink/mavlink_signing.cpp
    comms/settings.cpp
    comms/params.cpp
    comms/vehicle_params.cpp
    comms/gcs_link.cpp
)

# MAVLink over UDP for SITL / desktop builds (POSIX sockets)
if(NOT STM32_TARGET)
    list(APPEND CORE_SOURCES hal/posix/udp_link.cpp hal/posix/file_storage.cpp)
endif()

if(ENABLE_DRIVER_MPU6050)
    list(APPEND CORE_SOURCES drivers/imu/mpu6050.cpp)
    add_compile_definitions(ENABLE_DRIVER_MPU6050=1)
endif()

if(ENABLE_DRIVER_ICM20689)
    list(APPEND CORE_SOURCES drivers/imu/icm20689.cpp)
    add_compile_definitions(ENABLE_DRIVER_ICM20689=1)
endif()

if(ENABLE_DRIVER_BMP280)
    list(APPEND CORE_SOURCES drivers/baro/bmp280.cpp)
    add_compile_definitions(ENABLE_DRIVER_BMP280=1)
endif()

if(ENABLE_DRIVER_QMC5883L)
    list(APPEND CORE_SOURCES drivers/mag/qmc5883l.cpp)
    add_compile_definitions(ENABLE_DRIVER_QMC5883L=1)
endif()

if(ENABLE_DRIVER_NMEA_GPS)
    list(APPEND CORE_SOURCES drivers/gps/nmea.cpp)
    add_compile_definitions(ENABLE_DRIVER_NMEA_GPS=1)
endif()

if(ENABLE_CAN_DRONECAN)
    list(APPEND CORE_SOURCES
        drivers/can/dronecan_sensors.cpp
        drivers/can/dronecan_node.cpp
        drivers/can/dronecan_codec.cpp
    )
    add_compile_definitions(ENABLE_CAN_DRONECAN=1)
endif()

add_library(autopilot_core STATIC ${CORE_SOURCES})

# Target Executable
add_executable(autopilot_app system/main.cpp)
target_link_libraries(autopilot_app PRIVATE autopilot_core pthread)

# Real-time SITL: simulated quad + the full flight stack, talking MAVLink over
# UDP to QGroundControl (127.0.0.1:14550). See docs/MAVLINK_QGC.md.
if(NOT STM32_TARGET)
    add_executable(autopilot_sitl sim/sitl_main.cpp)
    target_link_libraries(autopilot_sitl PRIVATE autopilot_core pthread)
endif()

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

    add_executable(test_pubsub tests/test_pubsub.cpp)
    target_link_libraries(test_pubsub PRIVATE autopilot_core pthread)
    add_test(NAME test_pubsub COMMAND test_pubsub)

    add_executable(test_dronecan tests/test_dronecan.cpp)
    target_link_libraries(test_dronecan PRIVATE autopilot_core pthread)
    add_test(NAME test_dronecan COMMAND test_dronecan)

    add_executable(test_failsafe tests/test_failsafe.cpp)
    target_link_libraries(test_failsafe PRIVATE autopilot_core pthread)
    add_test(NAME test_failsafe COMMAND test_failsafe)

    # Review fixes (darsh branch): vehicle safety, mixer, sensors/drivers, filters
    foreach(t test_vehicle_safety test_mixer_saturation test_drivers test_filters_and_cal)
        add_executable(${t} tests/${t}.cpp)
        target_link_libraries(${t} PRIVATE autopilot_core pthread)
        add_test(NAME ${t} COMMAND ${t})
    endforeach()

    # Closed-loop software-in-the-loop flight (sim/quad_sim.hpp)
    add_executable(test_sitl_flight tests/test_sitl_flight.cpp)
    target_link_libraries(test_sitl_flight PRIVATE autopilot_core pthread)
    add_test(NAME test_sitl_flight COMMAND test_sitl_flight)

    # Rover, boat and ROV on the same firmware, switched at runtime (sim/sim_world.hpp)
    add_executable(test_sitl_vehicles tests/test_sitl_vehicles.cpp)
    target_link_libraries(test_sitl_vehicles PRIVATE autopilot_core pthread)
    add_test(NAME test_sitl_vehicles COMMAND test_sitl_vehicles)

    # Motor failure: re-allocation, detection, hexa/quad SITL survival
    add_executable(test_motor_failure tests/test_motor_failure.cpp)
    target_link_libraries(test_motor_failure PRIVATE autopilot_core pthread)
    add_test(NAME test_motor_failure COMMAND test_motor_failure)

    add_executable(test_sitl_gps_attitude tests/test_sitl_gps_attitude.cpp)
    target_link_libraries(test_sitl_gps_attitude PRIVATE autopilot_core pthread)
    add_test(NAME test_sitl_gps_attitude COMMAND test_sitl_gps_attitude)

    # Attitude without GPS: rotor-drag aiding and online gyro bias
    add_executable(test_sitl_drag_bias tests/test_sitl_drag_bias.cpp)
    target_link_libraries(test_sitl_drag_bias PRIVATE autopilot_core pthread)
    add_test(NAME test_sitl_drag_bias COMMAND test_sitl_drag_bias)

    # Flight-code bugs found by the ROS 2 simulator scenarios (ros2/autopilot_sim)
    add_executable(test_sim_found_bugs tests/test_sim_found_bugs.cpp)
    target_link_libraries(test_sim_found_bugs PRIVATE autopilot_core pthread)
    add_test(NAME test_sim_found_bugs COMMAND test_sim_found_bugs)

    # MAVLink codec (golden frames from pymavlink) + GCS link behaviour
    add_executable(test_mavlink tests/test_mavlink.cpp)
    target_link_libraries(test_mavlink PRIVATE autopilot_core pthread)
    add_test(NAME test_mavlink COMMAND test_mavlink)

    # Saved settings (file + A/B flash storage, parameters) and MAVLink signing
    foreach(t test_storage test_signing)
        add_executable(${t} tests/${t}.cpp)
        target_link_libraries(${t} PRIVATE autopilot_core pthread)
        add_test(NAME ${t} COMMAND ${t})
    endforeach()
endif()
