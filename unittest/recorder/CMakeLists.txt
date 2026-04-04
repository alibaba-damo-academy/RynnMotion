cmake_minimum_required(VERSION 3.16)

add_executable(test_robot_context test_robot_context.cpp)

target_link_libraries(test_robot_context
    manager
    Eigen3::Eigen
    ${YAML_LIB}
)

target_compile_definitions(test_robot_context PRIVATE
    MODEL_DIR="${MODEL_DIR}"
)

set_target_properties(test_robot_context PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${UNITTEST_OUTPUT_DIR}/recorder
)

message(STATUS "Recorder unit tests configured")
