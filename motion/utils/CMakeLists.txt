cmake_minimum_required(VERSION 3.16)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

project(motion_utils)

set(LIBRARY_NAME utility)

set(SOURCE_FILES
  kinematics/diff_ik.cpp
  kinematics/kine_pin.cpp
  logger/lcm_logger.cpp
  tools/filter.cpp
  tools/math_tools.cpp
  generator/signal_generator.cpp
  trajectory/trajgen.cpp
  ../fsm/src/pickplace_fsm.cpp
  dynamics/dyn_pin.cpp
  curve/bspline.cpp
  curve/fifthSpline.cpp
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)

string(REPLACE "-L" "" PINOCCHIO_LIB_PATHS "${PINOCCHIO_LDFLAGS}")
separate_arguments(PINOCCHIO_LIB_PATHS)
list(FILTER PINOCCHIO_LIB_PATHS INCLUDE REGEX "^/")
link_directories(${PINOCCHIO_LIB_PATHS})

if(CROSS_COMPILE)
  message(STATUS "Cross-compiling for ${CROSS_TARGET} - Using static linking")
  set(BUILD_SHARED_LIBS OFF)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static")
  add_library(${LIBRARY_NAME} STATIC ${SOURCE_FILES})
else()
  message(STATUS "libutility, Native compilation")
  add_library(${LIBRARY_NAME} SHARED ${SOURCE_FILES})
  if(APPLE)
    mujoco_fix_rpath(${LIBRARY_NAME})
  endif()

  if(BUILD_DEPLOY)
    add_library(${LIBRARY_NAME}_static STATIC ${SOURCE_FILES})
    set_target_properties(${LIBRARY_NAME}_static PROPERTIES OUTPUT_NAME ${LIBRARY_NAME})
  endif()
endif()

if(APPLE)
  set(YAML_LIB yaml-cpp::yaml-cpp)
  set(NATIVE_HINTS /opt/homebrew/lib)
  include_directories(/opt/homebrew/opt/yaml-cpp/include)
else()
  set(NATIVE_HINTS /usr/local/lib)
  find_library(YAML_CPP_LIBRARY NAMES yaml-cpp HINTS ${NATIVE_HINTS})
  set(YAML_LIB ${YAML_CPP_LIBRARY})
endif()

if(CROSS_COMPILE)
  find_library(FCL_LIBRARIES NAMES libfcl.so HINTS /usr/aarch64-linux-gnu/lib)
  find_library(LCM_LIBRARY NAMES liblcm.a HINTS /usr/aarch64-linux-gnu/lib)
  find_library(QPOASES_LIBRARY NAMES libqpOASES.a HINTS /usr/aarch64-linux-gnu/lib)
  find_library(RUCKIG_LIBRARY NAMES libruckig.a HINTS /usr/aarch64-linux-gnu/lib)
else()
  find_library(FCL_LIBRARIES NAMES fcl HINTS ${NATIVE_HINTS})
  find_library(LCM_LIBRARY NAMES lcm HINTS ${NATIVE_HINTS})
  find_library(QPOASES_LIBRARY NAMES qpOASES HINTS ${NATIVE_HINTS})
  find_library(RUCKIG_LIBRARY NAMES libruckig.a HINTS ${NATIVE_HINTS})
endif()

set(COMMON_INCLUDE_DIRS
  ${COMMON_DIR}/data/
  ${COMMON_DIR}/lcm/piperlcm/
  ${COMMON_DIR}/lcm/lcmMotion/
  ${CMAKE_CURRENT_SOURCE_DIR}/../fsm/include/
  ${CMAKE_CURRENT_SOURCE_DIR}/../manager/
  ${CMAKE_CURRENT_SOURCE_DIR}/tools
  ${CMAKE_CURRENT_SOURCE_DIR}/collision
  ${CMAKE_CURRENT_SOURCE_DIR}/kinematics
  ${CMAKE_CURRENT_SOURCE_DIR}/logger
  ${CMAKE_CURRENT_SOURCE_DIR}/generator
  ${CMAKE_CURRENT_SOURCE_DIR}/trajectory
  ${CMAKE_CURRENT_SOURCE_DIR}/curve
)

if(APPLE)
  list(APPEND COMMON_INCLUDE_DIRS
    /opt/homebrew/opt/yaml-cpp/include
    /opt/homebrew/include
    /opt/homebrew/opt/qpoases/include
  )
endif()

set(COMMON_LIBRARIES
  ${FCL_LIBRARIES}
  ${LCM_LIBRARY}
  ${QPOASES_LIBRARY}
  ${RUCKIG_LIBRARY}
  pinocchio_default
  pinocchio_parsers
  Boost::filesystem
  urdfdom_sensor
  urdfdom_model
  urdfdom_world
  manager
)

target_include_directories(${LIBRARY_NAME} PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_libraries(${LIBRARY_NAME} PUBLIC Eigen3::Eigen ${YAML_LIB} ${COMMON_LIBRARIES})

target_include_directories(${LIBRARY_NAME} SYSTEM BEFORE PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${PINOCCHIO_INCLUDE_DIRS}
)
target_compile_definitions(${LIBRARY_NAME} PUBLIC
  PINOCCHIO_WITH_URDFDOM
  PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
)
target_compile_options(${LIBRARY_NAME} PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-declarations>
)

if(NOT CROSS_COMPILE AND BUILD_DEPLOY)
  target_include_directories(${LIBRARY_NAME}_static PUBLIC ${COMMON_INCLUDE_DIRS})
  target_link_libraries(${LIBRARY_NAME}_static PUBLIC Eigen3::Eigen ${YAML_LIB} ${COMMON_LIBRARIES})

  target_include_directories(${LIBRARY_NAME}_static SYSTEM BEFORE PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    ${PINOCCHIO_INCLUDE_DIRS}
  )
  target_compile_definitions(${LIBRARY_NAME}_static PUBLIC
    PINOCCHIO_WITH_URDFDOM
    PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
  )
  target_compile_options(${LIBRARY_NAME}_static PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-declarations>
  )
endif()
