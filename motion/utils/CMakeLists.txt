cmake_minimum_required(VERSION 3.16)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

project(motion_utils)

set(LIBRARY_NAME utility)

set(SOURCE_FILES

  # Kinematics
  kinematics/diff_ik.cpp
  kinematics/kine_pin.cpp

  # Logger
  logger/lcm_logger.cpp

  # Tools
  tools/filter.cpp
  tools/math_tools.cpp

  # Generator
  generator/signal_generator.cpp

  # Trajectory
  trajectory/trajgen.cpp

  # FSM (required for pickplace_trajgen)
  ../fsm/src/pickplace_fsm.cpp

  # Dynamics
  dynamics/dyn_pin.cpp

  # curve
  curve/bspline.cpp
  curve/fifthSpline.cpp
)

# Find packages before creating targets
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Find Boost with required components for Pinocchio
find_package(Boost REQUIRED COMPONENTS filesystem)

# Find Pinocchio using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)

# Extract library paths for linker (must be called before add_library)
string(REPLACE "-L" "" PINOCCHIO_LIB_PATHS "${PINOCCHIO_LDFLAGS}")
separate_arguments(PINOCCHIO_LIB_PATHS)
list(FILTER PINOCCHIO_LIB_PATHS INCLUDE REGEX "^/")
link_directories(${PINOCCHIO_LIB_PATHS})

# Handle cross-compilation settings
if(CMAKE_TOOLCHAIN_FILE)
  message(STATUS "Cross-compiling for aarch64 - Using static linking")
  set(BUILD_SHARED_LIBS OFF)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static")

  # For cross-compilation, only build static library
  add_library(${LIBRARY_NAME} STATIC ${SOURCE_FILES})
else()
  message(STATUS "libutility, Native compilation")
  add_library(${LIBRARY_NAME} SHARED ${SOURCE_FILES})

  if(BUILD_STATIC)
    add_library(${LIBRARY_NAME}_static STATIC ${SOURCE_FILES})
    set_target_properties(${LIBRARY_NAME}_static PROPERTIES OUTPUT_NAME ${LIBRARY_NAME})
  endif()
endif()

# Set YAML library based on platform
if(APPLE)
  set(YAML_LIB yaml-cpp::yaml-cpp)
  set(NATIVE_HINTS /opt/homebrew/lib)
  include_directories(/opt/homebrew/opt/yaml-cpp/include)
else()
  set(NATIVE_HINTS /usr/local/lib)
  find_library(YAML_CPP_LIBRARY NAMES yaml-cpp HINTS ${NATIVE_HINTS})
  set(YAML_LIB ${YAML_CPP_LIBRARY})
endif()

# Find libraries
if(CMAKE_TOOLCHAIN_FILE)
  # Find static libraries for cross-compilation
  find_library(
    FCL_LIBRARIES
    NAMES libfcl.so
    HINTS /usr/aarch64-linux-gnu/lib)
  find_library(
    LCM_LIBRARY
    NAMES liblcm.a
    HINTS /usr/aarch64-linux-gnu/lib)
  find_library(
    QPOASES_LIBRARY
    NAMES libqpOASES.a
    HINTS /usr/aarch64-linux-gnu/lib)
  find_library(RUCKIG_LIBRARY
    NAMES libruckig.a
    HINTS /usr/aarch64-linux-gnu/lib)
else()
  # Find dynamic libraries for native build
  find_library(
    FCL_LIBRARIES
    NAMES fcl
    HINTS ${NATIVE_HINTS})
  find_library(
    LCM_LIBRARY
    NAMES lcm
    HINTS ${NATIVE_HINTS})
  find_library(
    QPOASES_LIBRARY
    NAMES qpOASES
    HINTS ${NATIVE_HINTS})
  find_library(RUCKIG_LIBRARY
    NAMES libruckig.a
    HINTS ${NATIVE_HINTS})
endif()

# Common include directories
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

# Common libraries to link against
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
)

# Apply include directories and link libraries
target_include_directories(${LIBRARY_NAME} PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_libraries(${LIBRARY_NAME} PUBLIC Eigen3::Eigen ${YAML_LIB} ${COMMON_LIBRARIES})

# Add Pinocchio includes and definitions to target
# Add Eigen3 include dir before Pinocchio to ensure correct header resolution
target_include_directories(${LIBRARY_NAME} SYSTEM BEFORE PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${PINOCCHIO_INCLUDE_DIRS}
)
target_compile_definitions(${LIBRARY_NAME} PUBLIC
  PINOCCHIO_WITH_URDFDOM
  PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
)

# Suppress deprecation warnings from Pinocchio headers
target_compile_options(${LIBRARY_NAME} PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-declarations>
)

# For native build, also set up the static library if needed
if(NOT CMAKE_TOOLCHAIN_FILE AND BUILD_STATIC)
  target_include_directories(${LIBRARY_NAME}_static PUBLIC ${COMMON_INCLUDE_DIRS})
  target_link_libraries(${LIBRARY_NAME}_static PUBLIC Eigen3::Eigen ${YAML_LIB} ${COMMON_LIBRARIES})

  # Add Pinocchio includes and definitions to static target
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

# Set up include directories
set(BUILD_INCLUDE_DIR ${CMAKE_BINARY_DIR}/${LIBRARY_NAME}/include)
file(MAKE_DIRECTORY ${BUILD_INCLUDE_DIR})

# Copy all header files from the new structure
file(GLOB_RECURSE HEADER_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/kinematics/*.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/collision/*.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/curve/*.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/generator/*.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/logger/*.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/tools/*.h*"
)

foreach(HEADER_FILE ${HEADER_FILES})
  file(RELATIVE_PATH REL_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${HEADER_FILE})
  get_filename_component(DIR_PATH ${REL_PATH} DIRECTORY)
  file(MAKE_DIRECTORY "${BUILD_INCLUDE_DIR}/${DIR_PATH}")
  file(COPY ${HEADER_FILE} DESTINATION "${BUILD_INCLUDE_DIR}/${DIR_PATH}")
endforeach()
