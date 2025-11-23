cmake_minimum_required(VERSION 3.16)
project(RynnMotion)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

if(CMAKE_TOOLCHAIN_FILE OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build (Debug or Release)")
    set(USING_GDB OFF)
    set(BUILD_MUJOCO OFF)
    set(BUILD_TESTS OFF)
    set(BUILD_ROS_MUJOCO OFF)
    set(BUILD_STATIC ON)
else()
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build (Debug or Release)")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(APPLE)
    execute_process(
        COMMAND xcrun --sdk macosx --show-sdk-path
        OUTPUT_VARIABLE CMAKE_OSX_SYSROOT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )

    if(CMAKE_OSX_SYSROOT)
        message(STATUS "Using macOS SDK: ${CMAKE_OSX_SYSROOT}")
        set(CMAKE_OSX_SYSROOT "${CMAKE_OSX_SYSROOT}" CACHE PATH "macOS SDK path" FORCE)
    endif()

    set(CMAKE_C_IMPLICIT_LINK_DIRECTORIES "" CACHE STRING "C implicit link directories" FORCE)
    set(CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES "" CACHE STRING "C++ implicit link directories" FORCE)
    set(CMAKE_C_IMPLICIT_LINK_LIBRARIES "" CACHE STRING "C implicit link libraries" FORCE)
    set(CMAKE_CXX_IMPLICIT_LINK_LIBRARIES "" CACHE STRING "C++ implicit link libraries" FORCE)

    set(CMAKE_C_STANDARD_LIBRARIES "" CACHE STRING "Standard C Libraries" FORCE)
    set(CMAKE_CXX_STANDARD_LIBRARIES "" CACHE STRING "Standard C++ Libraries" FORCE)

    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L${CMAKE_OSX_SYSROOT}/usr/lib" CACHE STRING "Linker flags" FORCE)
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -L${CMAKE_OSX_SYSROOT}/usr/lib" CACHE STRING "Shared linker flags" FORCE)
endif()

if(NOT CMAKE_TOOLCHAIN_FILE)
    option(USING_GDB "Using GDB to debug" ON)
    option(BUILD_MUJOCO "Build mujoco simulation" ON)
    option(BUILD_TESTS "Build tests in motion/tests" OFF)
    option(BUILD_ROS_MUJOCO "Build ROS-enabled MuJoCo executable" OFF)
    option(BUILD_STATIC "Build static libraries for deployment" OFF)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

include(FindMuJoCo)
find_mujoco()

if(NOT MUJOCO_FOUND)
    message(FATAL_ERROR "MuJoCo not found. Please install MuJoCo or set MUJOCO_DIR.")
endif()

if(APPLE)
    set(NATIVE_HINTS /opt/homebrew/lib)
else()
    set(NATIVE_HINTS /usr/local/lib)
endif()

set(MODEL_DIR "${CMAKE_SOURCE_DIR}/models" CACHE PATH "Path to robot model directory")
set(MOTION_DIR "${CMAKE_SOURCE_DIR}/motion" CACHE PATH "Path to motion directory")
set(COMMON_DIR "${CMAKE_SOURCE_DIR}/common" CACHE PATH "Path to common directory")
add_definitions(-DMODEL_DIR="${MODEL_DIR}")
add_definitions(-DMOTION_DIR="${MOTION_DIR}")

include(cmake/GenerateRobotTypes.cmake)
generate_robot_types(
    MODELS_DIR "${MODEL_DIR}"
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/robotDiscovery/robot_types.hpp"
)
include_directories("${CMAKE_BINARY_DIR}/robotDiscovery")

if(USING_GDB)
    set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g3 -ggdb -fPIC")
    set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O0 -Wall -g3 -ggdb -fPIC")
    add_compile_options(-g -fPIC)
else()
    set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O3 -Wall -fPIC")
    set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -fPIC")
    add_compile_options(-O3 -fPIC)
endif()

if(APPLE)
    set(MUJOCOLIB_DIR ${MUJOCO_LIB_DIR})
    set(MOTION_LIBRARY ${CMAKE_BINARY_DIR}/motion/libmotion.dylib)
    set(UTILITY_LIBRARY ${CMAKE_BINARY_DIR}/motion/utils/libutility.dylib)
elseif(UNIX)
    set(MUJOCOLIB_DIR ${MUJOCO_LIB_DIR})
    set(MOTION_LIBRARY ${CMAKE_BINARY_DIR}/motion/libmotion.so)
    set(UTILITY_LIBRARY ${CMAKE_BINARY_DIR}/motion/utils/libutility.so)
    set(MOTION_STATIC_LIBRARY ${CMAKE_BINARY_DIR}/motion/libmotion.a)
    set(MOUTILS_STATIC_LIBRARY ${CMAKE_BINARY_DIR}/motion/utils/libutility.a)
endif()

add_subdirectory(motion)

if(BUILD_MUJOCO)
    add_subdirectory(mujoco)
endif()

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(unittest)
endif()

if(BUILD_STATIC)
    file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/robots/damiao/lib)
    file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/robots/piper/src/xiaoda/lib)
    file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/robots/franka/src/libmotion/lib)
    file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/robots/realman/src/telemotion/lib)
endif()

add_custom_target(clean-all
    COMMAND ${CMAKE_BUILD_TOOL} clean
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/CMakeFiles
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/CMakeCache.txt
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/cmake_install.cmake
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/Makefile
)
