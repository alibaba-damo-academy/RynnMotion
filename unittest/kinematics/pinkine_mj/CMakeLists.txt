cmake_minimum_required(VERSION 3.16)

project(PinocchioMuJoCoKinematicsTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)

# Setup library paths for Pinocchio
string(REPLACE "-L" "" PINOCCHIO_LIB_PATHS "${PINOCCHIO_LDFLAGS}")
separate_arguments(PINOCCHIO_LIB_PATHS)
list(FILTER PINOCCHIO_LIB_PATHS INCLUDE REGEX "^/")
link_directories(${PINOCCHIO_LIB_PATHS})

# Setup yaml-cpp
if(APPLE)
    set(YAML_LIB yaml-cpp::yaml-cpp)
    include_directories(/opt/homebrew/opt/yaml-cpp/include)
else()
    find_library(YAML_CPP_LIBRARY NAMES yaml-cpp HINTS /usr/local/lib)
    set(YAML_LIB ${YAML_CPP_LIBRARY})
endif()

# Setup MODEL_DIR (works both standalone and as part of larger build)
if(NOT DEFINED MODEL_DIR)
    # Get absolute path for standalone builds
    get_filename_component(MODEL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../models" ABSOLUTE CACHE)
endif()

message(STATUS "    MODEL_DIR: ${MODEL_DIR}")
add_definitions(-DMODEL_DIR="${MODEL_DIR}")

# Setup UNITTEST_OUTPUT_DIR for standalone builds
if(NOT DEFINED UNITTEST_OUTPUT_DIR)
    set(UNITTEST_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}")
endif()

# MuJoCo configuration: Use centralized FindMuJoCo module
# Works both as standalone test and as part of main build
if(NOT MUJOCO_FOUND)
    # Standalone build: need to find MuJoCo ourselves
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake")
    include(FindMuJoCo)
    find_mujoco()

    if(NOT MUJOCO_FOUND)
        message(WARNING "      MuJoCo not found. Hybrid test will not be built.")
        return()
    endif()
endif()

# Set platform-specific base library directories
if(APPLE)
    set(BASE_LIB_DIR /opt/homebrew/lib)
    set(BASE_INCLUDE_DIR /opt/homebrew/include)
else()
    set(BASE_LIB_DIR /usr/local/lib)
    set(BASE_INCLUDE_DIR /usr/local/include)
endif()

message(STATUS "      Using MuJoCo ${MUJOCO_VERSION}: ${MUJOCO_LIBRARY}")

# Find GLFW
if(APPLE)
    set(GLFW_INCLUDE_DIR "${BASE_INCLUDE_DIR}/GLFW")
    set(GLFW_LIB "${BASE_LIB_DIR}/libglfw.dylib")

    if(EXISTS "${GLFW_INCLUDE_DIR}/glfw3.h" AND EXISTS "${GLFW_LIB}")
        message(STATUS "      GLFW library found: ${GLFW_LIB}")
        include_directories(${GLFW_INCLUDE_DIR})
        set(GLFW_INCLUDE_DIRS ${GLFW_INCLUDE_DIR})
    else()
        message(FATAL_ERROR "      GLFW library not found at expected location.")
    endif()
else()
    find_package(glfw3 REQUIRED)
    set(GLFW_INCLUDE_DIRS /usr/include/GLFW)
    set(GLFW_LIB glfw)
endif()

# Build Pinocchio+MuJoCo test
add_executable(pinMjTest
    pinmj_main.cpp
    pin_mj.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../motion/utils/kinematics/kine_pin.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../motion/manager/robot_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../motion/manager/mjcf_parser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../motion/manager/robot_discovery.cpp
)

set_target_properties(pinMjTest PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${UNITTEST_OUTPUT_DIR}/
    COMPILE_FLAGS "-g"
)

target_include_directories(pinMjTest PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    ${MUJOCO_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../motion/manager/
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../motion/utils/tools/
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../motion/utils/kinematics/
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/data/
)

# Add Pinocchio includes and definitions to target
target_include_directories(pinMjTest PRIVATE ${PINOCCHIO_INCLUDE_DIRS})
target_compile_definitions(pinMjTest PRIVATE
    PINOCCHIO_WITH_URDFDOM
    PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
)

# Suppress deprecation warnings from Pinocchio headers
target_compile_options(pinMjTest PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-declarations>
)

target_link_libraries(pinMjTest
    ${YAML_LIB}
    pinocchio_default
    pinocchio_parsers
    Boost::filesystem
    urdfdom_sensor
    urdfdom_model
    urdfdom_world
    ${MUJOCO_LIBRARY}
    ${GLFW_LIB}
)

if(APPLE)
    set_target_properties(pinMjTest PROPERTIES
        INSTALL_RPATH "${MUJOCO_LIB_DIR};${BASE_LIB_DIR}"
        BUILD_WITH_INSTALL_RPATH TRUE
    )

    add_custom_command(TARGET pinMjTest POST_BUILD
        COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/fix_mujoco_mac.sh $<TARGET_FILE:pinMjTest> ${MUJOCO_LIBRARY}
        COMMENT "Fixing MuJoCo library install name"
    )
endif()

# Register with CTest
add_test(NAME pinMjTest COMMAND pinMjTest)

# Build Pinocchio+MuJoCo multi-site test
add_executable(pinMjMulti
    pinmj_multimain.cpp
    pin_mj.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../motion/utils/kinematics/kine_pin.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../motion/manager/robot_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../motion/manager/mjcf_parser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../motion/manager/robot_discovery.cpp
)

set_target_properties(pinMjMulti PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${UNITTEST_OUTPUT_DIR}/
    COMPILE_FLAGS "-g"
)

target_include_directories(pinMjMulti PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    ${MUJOCO_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../motion/manager/
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../motion/utils/tools/
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../motion/utils/kinematics/
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/data/
)

# Add Pinocchio includes and definitions to target
target_include_directories(pinMjMulti PRIVATE ${PINOCCHIO_INCLUDE_DIRS})
target_compile_definitions(pinMjMulti PRIVATE
    PINOCCHIO_WITH_URDFDOM
    PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
)

# Suppress deprecation warnings from Pinocchio headers
target_compile_options(pinMjMulti PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-declarations>
)

target_link_libraries(pinMjMulti
    ${YAML_LIB}
    pinocchio_default
    pinocchio_parsers
    Boost::filesystem
    urdfdom_sensor
    urdfdom_model
    urdfdom_world
    ${MUJOCO_LIBRARY}
    ${GLFW_LIB}
)

if(APPLE)
    set_target_properties(pinMjMulti PROPERTIES
        INSTALL_RPATH "${MUJOCO_LIB_DIR};${BASE_LIB_DIR}"
        BUILD_WITH_INSTALL_RPATH TRUE
    )

    add_custom_command(TARGET pinMjMulti POST_BUILD
        COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/fix_mujoco_mac.sh $<TARGET_FILE:pinMjMulti> ${MUJOCO_LIBRARY}
        COMMENT "Fixing MuJoCo library install name for multi-site test"
    )
endif()

# Register with CTest
add_test(NAME pinMjMulti COMMAND pinMjMulti)

message(STATUS "    Configured pinMjTest (Pinocchio+MuJoCo kinematics test)")
message(STATUS "    Configured pinMjMulti (Pinocchio+MuJoCo multi-site test)")
