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
    message(STATUS "libmotion, Native compilation")
    add_library(${LIBRARY_NAME} SHARED ${SOURCE_FILES})

    if(BUILD_DEPLOY)
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
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/curve/
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/generator/
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/trajectory/
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/logger/
    ${CMAKE_CURRENT_SOURCE_DIR}/module/teleopfollow/
    ${CMAKE_SOURCE_DIR}/mujoco/include/
)

if(APPLE)
    list(APPEND COMMON_INCLUDE_DIRS /opt/homebrew/opt/yaml-cpp/include)
endif()

target_compile_definitions(${LIBRARY_NAME} PUBLIC
    MODEL_DIR="${MODEL_DIR}"
    MOTION_DIR="${MOTION_DIR}"
)

option(READ_HDF5_FILE "Enable hdf5 library to replay trajectory" OFF)

if(READ_HDF5_FILE)
    find_package(HDF5 REQUIRED COMPONENTS C CXX)
    include_directories(${HDF5_INCLUDE_DIRS})
    target_link_libraries(${LIBRARY_NAME} PRIVATE ${HDF5_C_LIBRARIES} ${HDF5_CXX_LIBRARIES})
    target_compile_definitions(${LIBRARY_NAME} PRIVATE READ_HDF5_FILE)
endif()

if(NOT CROSS_COMPILE AND BUILD_DEPLOY)
    target_compile_definitions(${LIBRARY_NAME}_static PUBLIC
        MODEL_DIR="${MODEL_DIR}"
        MOTION_DIR="${MOTION_DIR}"
    )
    target_include_directories(${LIBRARY_NAME}_static PUBLIC
        ${COMMON_INCLUDE_DIRS}
        ${CMAKE_BINARY_DIR}/robotDiscovery
        ${CMAKE_SOURCE_DIR}/models
    )
    target_link_libraries(${LIBRARY_NAME}_static PUBLIC
        Eigen3::Eigen ${YAML_LIB}
        ${LCM_LIBRARY}
        ${QPOASES_LIBRARY}
        utility_static
    )
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

if(CROSS_COMPILE)
    find_library(LCM_LIBRARY NAMES liblcm.a HINTS /usr/aarch64-linux-gnu/lib)
    find_library(QPOASES_LIBRARY NAMES libqpOASES.a HINTS /usr/aarch64-linux-gnu/lib)
else()
    find_library(LCM_LIBRARY NAMES lcm HINTS ${NATIVE_HINTS})
    find_library(QPOASES_LIBRARY NAMES qpOASES HINTS ${NATIVE_HINTS})
endif()

set(COMMON_LIBRARIES
    ${LCM_LIBRARY}
    ${QPOASES_LIBRARY}
    utility
)

target_link_libraries(${LIBRARY_NAME} PUBLIC Eigen3::Eigen ${YAML_LIB} ${COMMON_LIBRARIES})
target_link_libraries(${LIBRARY_NAME} PUBLIC manager)

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

