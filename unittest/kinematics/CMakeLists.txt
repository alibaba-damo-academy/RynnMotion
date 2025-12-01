cmake_minimum_required(VERSION 3.16)

project(PinocchioMuJoCoKinematicsTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)

string(REPLACE "-L" "" PINOCCHIO_LIB_PATHS "${PINOCCHIO_LDFLAGS}")
separate_arguments(PINOCCHIO_LIB_PATHS)
list(FILTER PINOCCHIO_LIB_PATHS INCLUDE REGEX "^/")
link_directories(${PINOCCHIO_LIB_PATHS})

if(APPLE)
    set(YAML_LIB yaml-cpp::yaml-cpp)
    set(BASE_LIB_DIR /opt/homebrew/lib)
    set(BASE_INCLUDE_DIR /opt/homebrew/include)
    include_directories(/opt/homebrew/opt/yaml-cpp/include)
else()
    find_library(YAML_CPP_LIBRARY NAMES yaml-cpp HINTS /usr/local/lib)
    set(YAML_LIB ${YAML_CPP_LIBRARY})
    set(BASE_LIB_DIR /usr/local/lib)
    set(BASE_INCLUDE_DIR /usr/local/include)
endif()

if(NOT DEFINED MODEL_DIR)
    get_filename_component(MODEL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../models" ABSOLUTE CACHE)
endif()

message(STATUS "  Building Kinematics Tests")
message(STATUS "    MODEL_DIR: ${MODEL_DIR}")
add_definitions(-DMODEL_DIR="${MODEL_DIR}")

set(ROBOT_TYPES_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/robotDiscovery")
set(ROBOT_TYPES_FILE "${ROBOT_TYPES_OUTPUT_DIR}/robot_types.hpp")

if(NOT EXISTS "${ROBOT_TYPES_FILE}")
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake")
    include(GenerateRobotTypes)
    generate_robot_types(
        MODELS_DIR "${MODEL_DIR}"
        OUTPUT_FILE "${ROBOT_TYPES_FILE}"
    )
endif()

include_directories(${ROBOT_TYPES_OUTPUT_DIR})

if(NOT DEFINED UNITTEST_OUTPUT_DIR)
    set(UNITTEST_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}")
endif()

if(NOT MUJOCO_FOUND)
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake")
    include(FindMuJoCo)
    find_mujoco()

    if(NOT MUJOCO_FOUND)
        message(WARNING "    MuJoCo not found. Kinematics tests will not be built.")
        return()
    endif()
endif()

message(STATUS "    Using MuJoCo ${MUJOCO_VERSION}: ${MUJOCO_LIBRARY}")

if(APPLE)
    set(GLFW_INCLUDE_DIR "${BASE_INCLUDE_DIR}/GLFW")
    set(GLFW_LIB "${BASE_LIB_DIR}/libglfw.dylib")

    if(EXISTS "${GLFW_INCLUDE_DIR}/glfw3.h" AND EXISTS "${GLFW_LIB}")
        include_directories(${GLFW_INCLUDE_DIR})
        set(GLFW_INCLUDE_DIRS ${GLFW_INCLUDE_DIR})
    else()
        message(FATAL_ERROR "    GLFW library not found at expected location.")
    endif()
else()
    find_package(glfw3 REQUIRED)
    set(GLFW_INCLUDE_DIRS /usr/include/GLFW)
    set(GLFW_LIB glfw)
endif()

set(MOTION_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../motion)
set(COMMON_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../common)
set(MODELS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../models)

add_executable(pinMjTest
    pin_mj_main.cpp
    pin_mj.cpp
    ${MOTION_SRC_DIR}/utils/kinematics/kine_pin.cpp
    ${MOTION_SRC_DIR}/manager/robot_manager.cpp
    ${MOTION_SRC_DIR}/manager/mjcf_parser.cpp
    ${MOTION_SRC_DIR}/manager/debug_config.cpp
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
    ${MOTION_SRC_DIR}/manager/
    ${MOTION_SRC_DIR}/utils/tools/
    ${MOTION_SRC_DIR}/utils/kinematics/
    ${COMMON_SRC_DIR}/data/
    ${MODELS_SRC_DIR}
)

target_include_directories(pinMjTest PRIVATE ${PINOCCHIO_INCLUDE_DIRS})
target_compile_definitions(pinMjTest PRIVATE
    PINOCCHIO_WITH_URDFDOM
    PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
)

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

add_test(NAME pinMjTest COMMAND pinMjTest)
add_test(NAME pinMjTestMulti COMMAND pinMjTest --multi)

message(STATUS "    Configured pinMjTest (Pinocchio+MuJoCo kinematics test)")
