cmake_minimum_required(VERSION 3.16)
project(RynnMotion)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

option(ENABLE_DEBUG_SYMBOLS "Enable debug symbols for debugging" OFF)
option(BUILD_MUJOCO "Build MuJoCo simulation" ON)
option(BUILD_TESTS "Build tests" OFF)
option(BUILD_ROS_MUJOCO "Build ROS-enabled MuJoCo executable" OFF)
option(BUILD_DEPLOY "Build static libraries for deployment" OFF)

option(CROSS_COMPILE "Enable cross-compilation" OFF)
set(CROSS_TARGET "orin" CACHE STRING "Cross-compile target")
set_property(CACHE CROSS_TARGET PROPERTY STRINGS "orin")

if(CROSS_COMPILE)
    message(STATUS "Cross-compiling for target: ${CROSS_TARGET}")

    if(CROSS_TARGET STREQUAL "orin")
        include(${CMAKE_SOURCE_DIR}/cmake/toolchains/toolchain_orin.cmake)
    else()
        message(FATAL_ERROR "Unknown cross-compile target: ${CROSS_TARGET}")
    endif()

    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
    set(ENABLE_DEBUG_SYMBOLS OFF CACHE BOOL "" FORCE)
    set(BUILD_MUJOCO OFF CACHE BOOL "" FORCE)
    set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(BUILD_ROS_MUJOCO OFF CACHE BOOL "" FORCE)
    set(BUILD_DEPLOY ON CACHE BOOL "" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(ENABLE_DEBUG_SYMBOLS)
    set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g3 -ggdb -fPIC")
    set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O0 -Wall -g3 -ggdb -fPIC")
    add_compile_options(-g -fPIC)
else()
    set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O3 -Wall -fPIC")
    set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -fPIC")
    add_compile_options(-O3 -fPIC)
endif()

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

    set(CMAKE_C_IMPLICIT_LINK_DIRECTORIES "" CACHE STRING "" FORCE)
    set(CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES "" CACHE STRING "" FORCE)
    set(CMAKE_C_IMPLICIT_LINK_LIBRARIES "" CACHE STRING "" FORCE)
    set(CMAKE_CXX_IMPLICIT_LINK_LIBRARIES "" CACHE STRING "" FORCE)
    set(CMAKE_C_STANDARD_LIBRARIES "" CACHE STRING "" FORCE)
    set(CMAKE_CXX_STANDARD_LIBRARIES "" CACHE STRING "" FORCE)

    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L${CMAKE_OSX_SYSROOT}/usr/lib" CACHE STRING "" FORCE)
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -L${CMAKE_OSX_SYSROOT}/usr/lib" CACHE STRING "" FORCE)

    set(NATIVE_HINTS /opt/homebrew/lib)
else()
    set(NATIVE_HINTS /usr/local/lib)
endif()

set(MODEL_DIR "${CMAKE_SOURCE_DIR}/models" CACHE PATH "Path to robot model directory")
set(MOTION_DIR "${CMAKE_SOURCE_DIR}/motion" CACHE PATH "Path to motion directory")
set(COMMON_DIR "${CMAKE_SOURCE_DIR}/common" CACHE PATH "Path to common directory")
add_definitions(-DMODEL_DIR="${MODEL_DIR}")
add_definitions(-DMOTION_DIR="${MOTION_DIR}")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

include(FindMuJoCo)
find_mujoco()

if(NOT MUJOCO_FOUND)
    message(FATAL_ERROR "MuJoCo not found. Please install MuJoCo or set MUJOCO_DIR.")
endif()

include(cmake/GenerateRobotTypes.cmake)
generate_robot_types(
    MODELS_DIR "${MODEL_DIR}"
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/robotDiscovery/robot_types.hpp"
)
include_directories("${CMAKE_BINARY_DIR}/robotDiscovery")

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

if(BUILD_DEPLOY)
    set(RYNN_INSTALL_DIR ${CMAKE_BINARY_DIR}/rynn_install)
    set(RYNN_ROOT ${CMAKE_SOURCE_DIR})
    set(RYNN_LIB_DIR ${RYNN_INSTALL_DIR}/lib)

    file(MAKE_DIRECTORY ${RYNN_INSTALL_DIR}/lib)
    file(MAKE_DIRECTORY ${RYNN_INSTALL_DIR}/cmake)
    file(MAKE_DIRECTORY ${RYNN_INSTALL_DIR}/include)

    configure_file(
        ${CMAKE_SOURCE_DIR}/cmake/RynnMotionConfig.cmake.in
        ${RYNN_INSTALL_DIR}/cmake/RynnMotionConfig.cmake
        @ONLY
    )

    file(COPY ${CMAKE_BINARY_DIR}/robotDiscovery/
        DESTINATION ${RYNN_INSTALL_DIR}/include/robotDiscovery
    )

    add_custom_target(rynn_install ALL
        COMMAND ${CMAKE_COMMAND} -E copy
            $<TARGET_FILE:motion_static> ${RYNN_INSTALL_DIR}/lib/libmotion.a
        COMMAND ${CMAKE_COMMAND} -E copy
            $<TARGET_FILE:utility_static> ${RYNN_INSTALL_DIR}/lib/libutility.a
        COMMAND ${CMAKE_COMMAND} -E copy
            $<TARGET_FILE:manager_static> ${RYNN_INSTALL_DIR}/lib/libmanager.a
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_BINARY_DIR}/robotDiscovery ${RYNN_INSTALL_DIR}/include/robotDiscovery
        DEPENDS motion_static utility_static manager_static
        COMMENT "Installing RynnMotion package to ${RYNN_INSTALL_DIR}"
    )
endif()

add_custom_target(clean-all
    COMMAND ${CMAKE_BUILD_TOOL} clean
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/CMakeFiles
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/CMakeCache.txt
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/cmake_install.cmake
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/Makefile
)
