cmake_minimum_required(VERSION 3.16)

project(UtilityTests)

set(UTILITY_TEST_TARGETS
    bezierTest
)

foreach(TARGET ${UTILITY_TEST_TARGETS})
    add_executable(${TARGET}
        ${TARGET}.cpp
        bezierPlanner.cpp
    )

    set_target_properties(${TARGET} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${UNITTEST_OUTPUT_DIR}/utility
        COMPILE_FLAGS "-g"
    )

    target_link_libraries(${TARGET}
        Eigen3::Eigen
        utility
        ${YAML_LIB}
    )

    # Register with CTest
    add_test(NAME ${TARGET} COMMAND ${TARGET})
endforeach()

message(STATUS "  Configured ${CMAKE_CURRENT_LIST_LENGTH} utility tests")
