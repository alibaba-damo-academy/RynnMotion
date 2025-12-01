cmake_minimum_required(VERSION 3.16)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

project(MotionUnitTests)

message(STATUS "Building Motion Unit Tests")

include_directories(
    ${COMMON_DIR}/data/
    ${CMAKE_CURRENT_SOURCE_DIR}/../motion/fsm/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/../motion/manager/
    ${CMAKE_CURRENT_SOURCE_DIR}/../motion/utils/tools/
    ${CMAKE_CURRENT_SOURCE_DIR}/../motion/utils/curve/
    ${CMAKE_CURRENT_SOURCE_DIR}/../motion/utils/kinematics/
    ${CMAKE_CURRENT_SOURCE_DIR}/../motion/utils/trajectory/
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

if(APPLE)
    set(YAML_LIB yaml-cpp::yaml-cpp)
    set(NATIVE_HINTS /opt/homebrew/lib)
    include_directories(/opt/homebrew/opt/yaml-cpp/include)
else()
    set(NATIVE_HINTS /usr/local/lib)
    find_library(YAML_CPP_LIBRARY NAMES yaml-cpp HINTS ${NATIVE_HINTS})
    set(YAML_LIB ${YAML_CPP_LIBRARY})
endif()

find_library(LCM_LIBRARY NAMES lcm HINTS ${NATIVE_HINTS})
find_library(QPOASES_LIBRARY NAMES qpOASES HINTS ${NATIVE_HINTS})

find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)
string(REPLACE "-L" "" PINOCCHIO_LIB_PATHS "${PINOCCHIO_LDFLAGS}")
separate_arguments(PINOCCHIO_LIB_PATHS)
list(FILTER PINOCCHIO_LIB_PATHS INCLUDE REGEX "^/")
link_directories(${PINOCCHIO_LIB_PATHS})

set(UNITTEST_OUTPUT_DIR ${CMAKE_BINARY_DIR}/unittest)

add_subdirectory(manager)
add_subdirectory(kinematics)
add_subdirectory(trajectory)
add_subdirectory(utility)
add_subdirectory(fsm)

message(STATUS "Motion Unit Tests configured successfully")
