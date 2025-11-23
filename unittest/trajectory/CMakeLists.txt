cmake_minimum_required(VERSION 3.16)

project(TrajectoryTests)

set(TRAJ_TEST_TARGETS
    test_ee
    test_joint
)

# Find nlohmann/json for ruckig cloud client
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
    if(APPLE)
        find_path(NLOHMANN_JSON_INCLUDE_DIR
            NAMES nlohmann/json.hpp
            PATHS /opt/homebrew/include /usr/local/include
            NO_DEFAULT_PATH
        )
    else()
        find_path(NLOHMANN_JSON_INCLUDE_DIR
            NAMES nlohmann/json.hpp
            PATHS /usr/include /usr/local/include
            NO_DEFAULT_PATH
        )
    endif()

    if(NLOHMANN_JSON_INCLUDE_DIR)
        set(nlohmann_json_FOUND TRUE)
        add_library(nlohmann_json::nlohmann_json INTERFACE IMPORTED)
        set_target_properties(nlohmann_json::nlohmann_json PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${NLOHMANN_JSON_INCLUDE_DIR}"
        )
    endif()
endif()

# Find ruckig library
find_package(ruckig QUIET)
if(NOT ruckig_FOUND)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(RUCKIG ruckig)
    endif()

    if(NOT RUCKIG_FOUND)
        if(APPLE)
            find_path(RUCKIG_INCLUDE_DIR NAMES ruckig/ruckig.hpp
                PATHS /opt/homebrew/include /usr/local/include NO_DEFAULT_PATH)
            find_library(RUCKIG_LIBRARY NAMES ruckig
                PATHS /opt/homebrew/lib /usr/local/lib NO_DEFAULT_PATH)
        else()
            find_path(RUCKIG_INCLUDE_DIR NAMES ruckig/ruckig.hpp
                PATHS /usr/include /usr/local/include NO_DEFAULT_PATH)
            find_library(RUCKIG_LIBRARY NAMES ruckig
                PATHS /usr/lib /usr/local/lib NO_DEFAULT_PATH)
        endif()

        if(RUCKIG_INCLUDE_DIR AND RUCKIG_LIBRARY)
            set(ruckig_FOUND TRUE)
            add_library(ruckig::ruckig SHARED IMPORTED)
            set_target_properties(ruckig::ruckig PROPERTIES
                IMPORTED_LOCATION "${RUCKIG_LIBRARY}"
                INTERFACE_INCLUDE_DIRECTORIES "${RUCKIG_INCLUDE_DIR}"
            )
            message(STATUS "    Found ruckig: ${RUCKIG_LIBRARY}")
        endif()
    endif()
endif()

# Build trajectory tests if ruckig is available
if(ruckig_FOUND OR RUCKIG_FOUND)
    foreach(TARGET ${TRAJ_TEST_TARGETS})
        add_executable(${TARGET}
            ${TARGET}.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/../../motion/utils/trajectory/trajgen.cpp
        )

        set_target_properties(${TARGET} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${UNITTEST_OUTPUT_DIR}/trajectory
            COMPILE_FLAGS "-g"
        )

        target_include_directories(${TARGET} PUBLIC
            ${EIGEN3_INCLUDE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/../../motion/utils/trajectory/
        )

        if(TARGET ruckig::ruckig)
            target_link_libraries(${TARGET} Eigen3::Eigen ruckig::ruckig)
            if(TARGET nlohmann_json::nlohmann_json)
                target_link_libraries(${TARGET} nlohmann_json::nlohmann_json)
            endif()
        elseif(RUCKIG_FOUND)
            target_link_libraries(${TARGET} Eigen3::Eigen ${RUCKIG_LIBRARIES})
            target_include_directories(${TARGET} PUBLIC ${RUCKIG_INCLUDE_DIRS})
            if(TARGET nlohmann_json::nlohmann_json)
                target_link_libraries(${TARGET} nlohmann_json::nlohmann_json)
            endif()
        endif()

        # Register with CTest
        add_test(NAME ${TARGET} COMMAND ${TARGET})
    endforeach()

    message(STATUS "  Configured ${CMAKE_CURRENT_LIST_LENGTH} trajectory tests")
else()
    message(WARNING "  ruckig library not found. Trajectory tests will not be built.")
endif()
