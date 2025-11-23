cmake_minimum_required(VERSION 3.16)

# Test for robot discovery system
add_executable(test_discovery test_discovery.cpp)

target_link_libraries(test_discovery
    manager
    Eigen3::Eigen
    ${YAML_LIB}
)

target_compile_definitions(test_discovery PRIVATE
    MODEL_DIR="${MODEL_DIR}"
)

set_target_properties(test_discovery PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${UNITTEST_OUTPUT_DIR}/manager
)

# Test for module defaults (smart module chain selection)
add_executable(test_module_defaults test_module_defaults.cpp)

target_link_libraries(test_module_defaults
    manager
    motion
    Eigen3::Eigen
    ${YAML_LIB}
)

target_compile_definitions(test_module_defaults PRIVATE
    MODEL_DIR="${MODEL_DIR}"
)

set_target_properties(test_module_defaults PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${UNITTEST_OUTPUT_DIR}/manager
)

message(STATUS "Manager unit tests configured")
