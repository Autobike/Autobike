option(local_build "local_build=OFF builds for target = NI myRIO, local_build=ON builds for target = host." OFF) 

cmake_minimum_required(VERSION 3.7)

if (local_build)
    message("NOTE: local_build=ON. Building for host system ${CMAKE_SYSTEM}.")
    if (CMAKE_HOST_UNIX)
        set(CMAKE_C_COMPILER /usr/bin/gcc)
        set(CMAKE_C_STANDARD_INCLUDE_DIRECTORIES /usr/local/include)
    elseif(CMAKE_HOST_WIN32)
        message("CMake rules for Windows target not implemented.")
        return()
    else()
        message("Unknown C toolchain path for system ${CMAKE_SYSTEM}.")
        return()
    endif()
    set(CMAKE_C_FLAGS "-Wall -O3")
else()
    message("local_build=OFF. Building for NI myRIO.")
    set(CMAKE_SYSTEM_NAME Linux)
    set(CMAKE_SYSTEM_PROCESSOR armv71)

    if (CMAKE_HOST_UNIX)
        set(toolchainpath /usr/local/oecore-x86_64/sysroots)
        set(CMAKE_C_COMPILER ${toolchainpath}/x86_64-nilrtsdk-linux/usr/bin/arm-nilrt-linux-gnueabi/arm-nilrt-linux-gnueabi-gcc)
    elseif(CMAKE_HOST_WIN32)
        set(toolchainpath C:/build/18.0/arm/sysroots)
        set(CMAKE_C_COMPILER ${toolchainpath}/i686-nilrtsdk-mingw32/usr/bin/arm-nilrt-linux-gnueabi/arm-nilrt-linux-gnueabi-gcc.exe)
    else()
        message("Unknown NI toolchain path for host system ${CMAKE_SYSTEM}. Check myrio/c/CMakeLists.txt.")
        return()
    endif()

    set(CMAKE_SYSROOT ${toolchainpath}/cortexa9-vfpv3-nilrt-linux-gnueabi)
    set(CMAKE_C_FLAGS "-Wall -O3 -fmessage-length=0 -mfpu=vfpv3 -mfloat-abi=softfp -mcpu=cortex-a9 -march=armv7-a")
    set(CMAKE_C_STANDARD_INCLUDE_DIRECTORIES ${toolchainpath}/cortexa9-vfpv3-nilrt-linuxgnueabi/usr/include/c++/6.3.0 ${toolchainpath}/cortexa9-vfpv3-nilrt-linuxgnueabi/usr/include/c++/6.3.0/arm-nilrt-linux-gnueabi)
endif()

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

project(autobike)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY bin)
set(EXECUTABLE_OUTPUT_PATH debug_bin)

add_library(balancing_controller SHARED src/balancing_controller.c)
#add_library(genAbortTraj SHARED src/generateAbortTrajectory.c)
add_library(trajectory_controller SHARED src/trajectory_controller.c
    src/trajectory_selector.c
    src/generateAbortTrajectory.c)
add_library(trajectory_selector SHARED src/trajectory_selector.c
    src/generateAbortTrajectory.c)
add_library(state_estimator SHARED src/state_estimator.c)
add_library(vesc_communication SHARED src/vesc_communication.c
    submodules/bldc_uart_comm_stm32f4_discovery/bldc_interface_uart.c
    submodules/bldc_uart_comm_stm32f4_discovery/packet.c
    submodules/bldc_uart_comm_stm32f4_discovery/crc.c
    submodules/bldc_uart_comm_stm32f4_discovery/bldc_interface.c
    submodules/bldc_uart_comm_stm32f4_discovery/buffer.c)
add_library(kalman_filter SHARED src/kalman_filter.c)
add_library(pid_speed SHARED src/pid_speed_controller.c)
target_include_directories(vesc_communication PRIVATE src/include/ChibiOS)
add_subdirectory(submodules/qpSWIFT)
add_library(mpc_trajectory SHARED src/pre_mpc_trajectory.c src/mpc_trajectory.c)
target_link_libraries(mpc_trajectory qpSWIFT-static)
add_executable(mpc_debug src/mpc_debug.c)
target_link_libraries(mpc_debug mpc_trajectory trajectory_selector)
