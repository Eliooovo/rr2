cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME rr2)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C ASM)
add_executable(${CMAKE_PROJECT_NAME}
    App/chassis_app.c
    App/chassis_app.h
    App/comm_app.c
    App/comm_app.h
    App/gripper_app.c
    App/gripper_app.h
    App/kfs_lift_app.c
    App/kfs_lift_app.h
    App/kfs_rotate_app.c
    App/kfs_rotate_app.h
    App/kfs_grip_app.c
    App/kfs_grip_app.h
    App/weapon_rotate_app.c
    App/weapon_rotate_app.h
    App/weapon_grip_app.c
    App/weapon_grip_app.h
    App/lift_app.c
    App/lift_app.h
    Device/dji_motor.c
    Device/dji_motor.h
    Device/dji_motor_group.c
    Device/dji_motor_group.h
    Device/rs_motor/rs_motor.c
    Device/rs_motor/rs_motor.h
    Device/sts_servo.c
    Device/sts_servo.h
    robotic_arm/SCSLib/SCS.c
    robotic_arm/SCSLib/SCS.h
    robotic_arm/SCSLib/SMS_STS.c
    robotic_arm/SCSLib/SMS_STS.h
    robotic_arm/SCSLib/INST.h
    Modules/comm_protocol.h
    Device/tof200c/tof200c.c
    Device/tof200c/tof200c.h
    Device/tof200c/vl53l0x_platform.c
    ThirdParty/ST/VL53L0X/core/src/vl53l0x_api.c
    ThirdParty/ST/VL53L0X/core/src/vl53l0x_api_calibration.c
    ThirdParty/ST/VL53L0X/core/src/vl53l0x_api_core.c
    ThirdParty/ST/VL53L0X/core/src/vl53l0x_api_ranging.c
    ThirdParty/ST/VL53L0X/core/src/vl53l0x_api_strings.c)

add_subdirectory(cmake/stm32cubemx)

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Bsp/bsp_fdcan.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
    App
    Bsp
    Device
    Device/rs_motor
    robotic_arm/SCSLib
    Modules
    Device/tof200c
    ThirdParty/ST/VL53L0X/core/inc
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)
