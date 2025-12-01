cmake_minimum_required(VERSION 3.16)
project(manager)

set(LIBRARY_NAME manager)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

if(APPLE)
  set(YAML_LIB yaml-cpp::yaml-cpp)
  include_directories(/opt/homebrew/opt/yaml-cpp/include)
  set(BASE_LIB_DIR /opt/homebrew/lib)
else()
  find_library(YAML_CPP_LIBRARY NAMES yaml-cpp)
  set(YAML_LIB ${YAML_CPP_LIBRARY})
  set(BASE_LIB_DIR /usr/local/lib)
endif()

message(STATUS "libmanager: Using MuJoCo library: ${MUJOCO_LIBRARY}")

set(SOURCE_FILES
  robot_manager.cpp
  mjcf_parser.cpp
  debug_config.cpp
)

add_library(${LIBRARY_NAME} SHARED ${SOURCE_FILES})

if(BUILD_STATIC)
  add_library(${LIBRARY_NAME}_static STATIC ${SOURCE_FILES})
  set_target_properties(${LIBRARY_NAME}_static PROPERTIES OUTPUT_NAME ${LIBRARY_NAME})
endif()

target_include_directories(${LIBRARY_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/models
  ${CMAKE_SOURCE_DIR}/common/data/
  ${CMAKE_SOURCE_DIR}/common/lcm/lcmMotion/
  ${CMAKE_SOURCE_DIR}/common/lcm/piperlcm/
  ${CMAKE_SOURCE_DIR}/motion/fsm/include/
  ${CMAKE_SOURCE_DIR}/motion/module/
  ${CMAKE_SOURCE_DIR}/motion/module/planner/
  ${CMAKE_SOURCE_DIR}/motion/module/osc/
  ${CMAKE_SOURCE_DIR}/motion/module/estimation/
  ${CMAKE_SOURCE_DIR}/motion/module/joint/
  ${CMAKE_SOURCE_DIR}/motion/module/teleopfollow/
  ${CMAKE_SOURCE_DIR}/motion/interface/
  ${CMAKE_SOURCE_DIR}/motion/utils/tools/
  ${CMAKE_SOURCE_DIR}/motion/utils/kinematics/
  ${CMAKE_SOURCE_DIR}/motion/utils/dynamics/
  ${CMAKE_SOURCE_DIR}/motion/utils/curve/
  ${CMAKE_SOURCE_DIR}/motion/utils/generator/
  ${CMAKE_SOURCE_DIR}/motion/utils/trajectory/
  ${CMAKE_SOURCE_DIR}/motion/utils/logger/
  ${CMAKE_SOURCE_DIR}/motion/utils/sketch/include/
  ${MUJOCO_INCLUDE_DIRS}
)

target_compile_definitions(${LIBRARY_NAME} PUBLIC
  MODEL_DIR="${MODEL_DIR}"
  MOTION_DIR="${MOTION_DIR}"
)

target_link_libraries(${LIBRARY_NAME} PUBLIC
  Eigen3::Eigen
  ${YAML_LIB}
  ${MUJOCO_LIBRARY}
)

set_target_properties(${LIBRARY_NAME} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/motion/manager
)

if(BUILD_STATIC)
  target_include_directories(${LIBRARY_NAME}_static PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/models
    ${CMAKE_SOURCE_DIR}/common/data/
    ${CMAKE_SOURCE_DIR}/common/lcm/lcmMotion/
    ${CMAKE_SOURCE_DIR}/common/lcm/piperlcm/
    ${CMAKE_SOURCE_DIR}/motion/fsm/include/
    ${CMAKE_SOURCE_DIR}/motion/module/
    ${CMAKE_SOURCE_DIR}/motion/module/planner/
    ${CMAKE_SOURCE_DIR}/motion/module/osc/
    ${CMAKE_SOURCE_DIR}/motion/module/estimation/
    ${CMAKE_SOURCE_DIR}/motion/module/joint/
    ${CMAKE_SOURCE_DIR}/motion/module/teleopfollow/
    ${CMAKE_SOURCE_DIR}/motion/interface/
    ${CMAKE_SOURCE_DIR}/motion/utils/tools/
    ${CMAKE_SOURCE_DIR}/motion/utils/kinematics/
    ${CMAKE_SOURCE_DIR}/motion/utils/dynamics/
    ${CMAKE_SOURCE_DIR}/motion/utils/curve/
    ${CMAKE_SOURCE_DIR}/motion/utils/generator/
    ${CMAKE_SOURCE_DIR}/motion/utils/trajectory/
    ${CMAKE_SOURCE_DIR}/motion/utils/logger/
    ${CMAKE_SOURCE_DIR}/motion/utils/sketch/include/
    ${MUJOCO_INCLUDE_DIRS}
  )

  target_compile_definitions(${LIBRARY_NAME}_static PUBLIC
    MODEL_DIR="${MODEL_DIR}"
    MOTION_DIR="${MOTION_DIR}"
  )

  target_link_libraries(${LIBRARY_NAME}_static PUBLIC
    Eigen3::Eigen
    ${YAML_LIB}
    ${MUJOCO_LIBRARY}
  )

  set_target_properties(${LIBRARY_NAME}_static PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/motion/manager
  )
endif()

message(STATUS "libmanager configured successfully")
