find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

find_package(rm_ros_interfaces QUIET)
if(rm_ros_interfaces_FOUND)
    add_compile_definitions(USE_RM_INTERFACES)
    message(STATUS "Found rm_ros_interfaces - Building with RM robot support")
else()
    message(STATUS "rm_ros_interfaces not found - Building without RM robot support")
endif()

find_library(CCD_LIBRARY NAMES ccd REQUIRED)
if(NOT CCD_LIBRARY)
    message(FATAL_ERROR "libccd library not found")
else()
    message(STATUS "Found libccd: ${CCD_LIBRARY}")
endif()

add_executable(rosMujocoExe
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ros_main_mj.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ros_mj_interface.cpp
    ${MUJOCO_SOURCES}
)

ament_target_dependencies(rosMujocoExe
    rclcpp
    sensor_msgs
    nav_msgs
    rosgraph_msgs
    yaml-cpp
)

if(rm_ros_interfaces_FOUND)
    ament_target_dependencies(rosMujocoExe rm_ros_interfaces)
endif()

target_include_directories(rosMujocoExe PUBLIC ${MUJOCO_INCLUDE_DIRS})
target_link_directories(rosMujocoExe PRIVATE ${MUJOCO_LINK_DIRS})

if(APPLE)
    set_target_properties(rosMujocoExe PROPERTIES
        BUILD_RPATH "${CMAKE_BINARY_DIR}/motion;${CMAKE_BINARY_DIR}/motion/utils;${PLATFORM_LIB_DIR}"
        INSTALL_RPATH "@executable_path/../lib;@loader_path/../lib;${PLATFORM_LIB_DIR}"
    )
endif()

target_link_libraries(rosMujocoExe
    manager
    motion
    utility
    ${MUJOCO_COMMON_LIBS}
    ${FCL_LIBRARIES}
    ${CCD_LIBRARY}
    ${OpenCV_LIBS}
    PkgConfig::LIBAV
)

if(HDF5_AVAILABLE)
    target_link_libraries(rosMujocoExe ${HDF5_CXX_LIBRARIES})
endif()

if(PARQUET_AVAILABLE)
    target_link_libraries(rosMujocoExe Arrow::arrow_shared Parquet::parquet_shared)
endif()

add_dependencies(rosMujocoExe utility motion)

add_custom_command(TARGET rosMujocoExe POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:rosMujocoExe> ${CMAKE_SOURCE_DIR}/build/rosMujocoExe
)

install(TARGETS rosMujocoExe
    DESTINATION lib/${PROJECT_NAME}
)
