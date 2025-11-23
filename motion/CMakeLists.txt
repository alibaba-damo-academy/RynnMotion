cmake_minimum_required(VERSION 3.16)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

project(motion)

set(LIBRARY_NAME motion)

add_subdirectory(manager)
add_subdirectory(utils)

set(SOURCE_FILES
    module/module_base.cpp
    module/joint/joint_move.cpp
    module/osc/osc.cpp
    module/planner/planner.cpp
    module/planner/pickplace_controller.cpp
    module/estimation/estimator.cpp
    module/teleopfollow/teleop_follow.cpp
    module/teleopfollow/fr3_teleopfollow.cpp
    module/joint/fr3_jointmove.cpp
    interface/interface_base.cpp
    fsm/src/motion_fsm.cpp
    runtime/scene_manager.cpp
    runtime/module_manager.cpp
    runtime/fsm_manager.cpp
    runtime/timing_manager.cpp
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
    message(STATUS "libmotion, Native compilation")

    # For native compilation, build dynamic library
    add_library(${LIBRARY_NAME} SHARED ${SOURCE_FILES})

    # Only build static library if needed for deployment
    if(BUILD_STATIC)
        # Static library
        add_library(${LIBRARY_NAME}_static STATIC ${SOURCE_FILES})
        set_target_properties(${LIBRARY_NAME}_static PROPERTIES OUTPUT_NAME ${LIBRARY_NAME})
    endif()
endif()

set(COMMON_INCLUDE_DIRS
    ${COMMON_DIR}/data/
    ${COMMON_DIR}/lcm/lcmMotion/
    ${COMMON_DIR}/lcm/piperlcm/
    ${CMAKE_CURRENT_SOURCE_DIR}/fsm/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/manager/
    ${CMAKE_CURRENT_SOURCE_DIR}/runtime/
    ${CMAKE_CURRENT_SOURCE_DIR}/module/
    ${CMAKE_CURRENT_SOURCE_DIR}/module/planner/
    ${CMAKE_CURRENT_SOURCE_DIR}/module/osc/
    ${CMAKE_CURRENT_SOURCE_DIR}/module/estimation/
    ${CMAKE_CURRENT_SOURCE_DIR}/module/joint/
    ${CMAKE_CURRENT_SOURCE_DIR}/interface/
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/tools/
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/kinematics/
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/dynamics/
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/collision/
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/curve/
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/generator/
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/trajectory/
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/logger/
    ${CMAKE_CURRENT_SOURCE_DIR}/module/teleopfollow/
    ${CMAKE_SOURCE_DIR}/mujoco/include/ # For robot_manager.hpp
)

if(APPLE)
    list(APPEND COMMON_INCLUDE_DIRS /opt/homebrew/opt/yaml-cpp/include)
endif()

# Add compile definitions (include MuJoCo lib for mjcf_parser)
target_compile_definitions(${LIBRARY_NAME} PUBLIC
    MODEL_DIR="${MODEL_DIR}"
    MOTION_DIR="${MOTION_DIR}"
)

# operation
option(READ_HDF5_FILE "Enable hdf5 library to replay trajectory" OFF)

if(READ_HDF5_FILE)
    find_package(HDF5 REQUIRED COMPONENTS C CXX)
    include_directories(${HDF5_INCLUDE_DIRS})
    target_link_libraries(${LIBRARY_NAME} PRIVATE ${HDF5_C_LIBRARIES} ${HDF5_CXX_LIBRARIES})
    target_compile_definitions(${LIBRARY_NAME} PRIVATE READ_HDF5_FILE)
endif()

# For native build, also set up the static library if needed
if(NOT CMAKE_TOOLCHAIN_FILE AND BUILD_STATIC)
    target_compile_definitions(${LIBRARY_NAME}_static PUBLIC
        MODEL_DIR="${MODEL_DIR}"
        MOTION_DIR="${MOTION_DIR}"
    )
    target_include_directories(${LIBRARY_NAME}_static PUBLIC ${COMMON_INCLUDE_DIRS})
    target_link_libraries(${LIBRARY_NAME}_static PUBLIC
        Eigen3::Eigen ${YAML_LIB}
        ${LCM_LIBRARY}
        ${QPOASES_LIBRARY}
        utility_static
    )

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

target_include_directories(${LIBRARY_NAME} PUBLIC ${COMMON_INCLUDE_DIRS})

# Find libraries
if(CMAKE_TOOLCHAIN_FILE)
    # Find static libraries for cross-compilation
    find_library(LCM_LIBRARY NAMES liblcm.a HINTS /usr/aarch64-linux-gnu/lib)
    find_library(QPOASES_LIBRARY NAMES libqpOASES.a HINTS /usr/aarch64-linux-gnu/lib)
else()
    # Find dynamic libraries for native build
    find_library(LCM_LIBRARY NAMES lcm HINTS ${NATIVE_HINTS})
    find_library(QPOASES_LIBRARY NAMES qpOASES HINTS ${NATIVE_HINTS})
endif()

# Common libraries to link against
# Note: Pinocchio libraries are linked via utility, no need to link them here again
# Note: NO MuJoCo library - libmotion is MuJoCo-independent!
if(CMAKE_TOOLCHAIN_FILE)
    # For cross-compilation
    set(COMMON_LIBRARIES
        ${LCM_LIBRARY}
        ${QPOASES_LIBRARY}
        utility
    )
else()
    # For native build
    set(COMMON_LIBRARIES
        ${LCM_LIBRARY}
        ${QPOASES_LIBRARY}
        utility
    )
endif()

target_link_libraries(${LIBRARY_NAME} PUBLIC Eigen3::Eigen ${YAML_LIB} ${COMMON_LIBRARIES})

# Link against manager library
target_link_libraries(${LIBRARY_NAME} PUBLIC manager)

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

# Create directories for library copies
file(MAKE_DIRECTORY
    ${CMAKE_SOURCE_DIR}/robots/realman/src/telemotion/lib/
    ${CMAKE_SOURCE_DIR}/robots/piper/src/xiaoda/lib/
    ${CMAKE_SOURCE_DIR}/robots/franka/src/libmotion/lib/
)

# Create directory for damiao if needed
if(CMAKE_TOOLCHAIN_FILE OR BUILD_STATIC)
    file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/robots/damiao/lib/)
endif()

# Copy libraries to appropriate locations
if(CMAKE_TOOLCHAIN_FILE)
    add_custom_command(TARGET ${LIBRARY_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${LIBRARY_NAME}> ${CMAKE_SOURCE_DIR}/robots/realman/src/telemotion/lib/lib${LIBRARY_NAME}.a
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${LIBRARY_NAME}> ${CMAKE_SOURCE_DIR}/robots/piper/src/xiaoda/lib/lib${LIBRARY_NAME}.a
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${LIBRARY_NAME}> ${CMAKE_SOURCE_DIR}/robots/damiao/lib/lib${LIBRARY_NAME}.a
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${LIBRARY_NAME}> ${CMAKE_SOURCE_DIR}/robots/franka/src/libmotion/lib/lib${LIBRARY_NAME}.a
    )
else()
    add_custom_command(TARGET ${LIBRARY_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${LIBRARY_NAME}> ${CMAKE_SOURCE_DIR}/robots/realman/src/telemotion/lib/
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${LIBRARY_NAME}> ${CMAKE_SOURCE_DIR}/robots/franka/src/libmotion/lib/
    )

    if(BUILD_STATIC)
        add_custom_command(TARGET ${LIBRARY_NAME}_static POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${LIBRARY_NAME}_static> ${CMAKE_SOURCE_DIR}/robots/damiao/lib/lib${LIBRARY_NAME}.a
            COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${LIBRARY_NAME}_static> ${CMAKE_SOURCE_DIR}/robots/piper/src/xiaoda/lib/lib${LIBRARY_NAME}.a
            COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${LIBRARY_NAME}_static> ${CMAKE_SOURCE_DIR}/robots/franka/src/libmotion/lib/lib${LIBRARY_NAME}.a
            COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${LIBRARY_NAME}_static> ${CMAKE_SOURCE_DIR}/robots/realman/src/telemotion/lib/lib${LIBRARY_NAME}.a
            COMMENT "Copying static libmotion.a to deployment dirs"
        )
    endif()
endif()

set(INCLUDE_FILES
    ${CMAKE_SOURCE_DIR}/${LIBRARY_NAME}/utils/tools/math_tools.hpp
    ${CMAKE_SOURCE_DIR}/${LIBRARY_NAME}/utils/tools/orient_tools.hpp
    ${CMAKE_SOURCE_DIR}/${LIBRARY_NAME}/runtime/module_manager.hpp
    ${CMAKE_SOURCE_DIR}/${LIBRARY_NAME}/runtime/scene_manager.hpp
    ${CMAKE_SOURCE_DIR}/${LIBRARY_NAME}/interface/interface_base.hpp
    ${CMAKE_SOURCE_DIR}/${LIBRARY_NAME}/utils/curve/fifthSpline.hpp
)

file(MAKE_DIRECTORY
    ${CMAKE_SOURCE_DIR}/robots/piper/src/xiaoda/lib/include
)

file(COPY
    ${INCLUDE_FILES}
    DESTINATION
    ${CMAKE_SOURCE_DIR}/robots/piper/src/xiaoda/lib/include
)

# Create assets directory for gcode files
file(MAKE_DIRECTORY
    ${CMAKE_SOURCE_DIR}/robots/piper/assets/gcode
)

file(GLOB_RECURSE HEADER_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/messages/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/manager/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/runtime/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/module/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/module/estimation/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/module/teleopfollow/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/interface/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/fsm/include/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/utils/kinematics/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/utils/collision/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/utils/curve/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/utils/generator/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/utils/logger/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/utils/tools/*.h*"
    "${CMAKE_CURRENT_SOURCE_DIR}/utils/dynamics/*.h*"
)

file(MAKE_DIRECTORY
    ${CMAKE_SOURCE_DIR}/robots/franka/src/libmotion/include
)

foreach(HEADER_FILE ${HEADER_FILES})
    file(COPY ${HEADER_FILE} DESTINATION ${CMAKE_SOURCE_DIR}/robots/franka/src/libmotion/include)
endforeach()
