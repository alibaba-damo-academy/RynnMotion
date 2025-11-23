cmake_minimum_required(VERSION 3.16)

project(FsmTests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(STANDALONE_BUILD ON)
    message(STATUS "Building FSM tests in STANDALONE mode")
else()
    set(STANDALONE_BUILD OFF)
endif()

if(STANDALONE_BUILD)
    set(UNITTEST_OUTPUT_DIR ${CMAKE_BINARY_DIR})

    find_package(Eigen3 REQUIRED)
    find_package(yaml-cpp REQUIRED)

    if(APPLE)
        set(YAML_LIB yaml-cpp::yaml-cpp)
    else()
        find_library(YAML_CPP_LIBRARY NAMES yaml-cpp HINTS /usr/local/lib)
        set(YAML_LIB ${YAML_CPP_LIBRARY})
    endif()

    set(COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../common)
else()
    if(NOT DEFINED UNITTEST_OUTPUT_DIR)
        set(UNITTEST_OUTPUT_DIR ${CMAKE_BINARY_DIR}/unittest)
    endif()
endif()

add_executable(simpleTick
    simpletick_test.cpp
    src/test_fsm.cpp
)

target_include_directories(simpleTick PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../motion/fsm/include
)

if(STANDALONE_BUILD)
    set_target_properties(simpleTick PROPERTIES COMPILE_FLAGS "-g")
else()
    set_target_properties(simpleTick PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${UNITTEST_OUTPUT_DIR}/fsm
        COMPILE_FLAGS "-g"
    )
endif()

add_test(NAME simpleTick COMMAND simpleTick)

add_executable(motionTick
    motiontick_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../motion/fsm/src/motion_fsm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../motion/runtime/fsm_manager.cpp
)

target_include_directories(motionTick PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../motion/fsm/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../motion/runtime
    ${COMMON_DIR}/data
)

target_link_libraries(motionTick PRIVATE Eigen3::Eigen)

if(STANDALONE_BUILD)
    set_target_properties(motionTick PROPERTIES COMPILE_FLAGS "-g")
else()
    set_target_properties(motionTick PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${UNITTEST_OUTPUT_DIR}/fsm
        COMPILE_FLAGS "-g"
    )
endif()

add_test(NAME motionTick COMMAND motionTick)

find_package(ruckig QUIET)
find_package(nlohmann_json QUIET)

if(ruckig_FOUND)
    add_executable(pickplaceTest
        pickplace_test.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../../motion/fsm/src/pickplace_fsm.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../../motion/utils/trajectory/trajgen.cpp
    )

    target_include_directories(pickplaceTest PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../motion/fsm/include
        ${COMMON_DIR}/data
        ${CMAKE_CURRENT_SOURCE_DIR}/../../motion/utils/trajectory
        ${CMAKE_CURRENT_SOURCE_DIR}/../../motion/utils/kinematics
        ${CMAKE_CURRENT_SOURCE_DIR}/../../motion/utils/tools
    )

    target_link_libraries(pickplaceTest PRIVATE
        Eigen3::Eigen
        ${YAML_LIB}
        ruckig::ruckig
    )

    if(nlohmann_json_FOUND)
        target_link_libraries(pickplaceTest PRIVATE nlohmann_json::nlohmann_json)
    endif()

    if(STANDALONE_BUILD)
        set_target_properties(pickplaceTest PROPERTIES COMPILE_FLAGS "-g")
        message(STATUS "Configured FSM tests (3 tests) - STANDALONE")
    else()
        set_target_properties(pickplaceTest PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${UNITTEST_OUTPUT_DIR}/fsm
            COMPILE_FLAGS "-g"
        )
        message(STATUS "  Configured FSM tests (3 tests)")
    endif()

    add_test(NAME pickplaceTest COMMAND pickplaceTest)
else()
    if(STANDALONE_BUILD)
        message(WARNING "ruckig library not found. pickplaceTest will not be built.")
        message(STATUS "Configured FSM tests (2 tests) - STANDALONE")
    else()
        message(WARNING "  ruckig library not found. pickplaceTest will not be built.")
        message(STATUS "  Configured FSM tests (2 tests)")
    endif()
endif()
