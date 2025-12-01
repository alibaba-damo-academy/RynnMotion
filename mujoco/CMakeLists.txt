cmake_minimum_required(VERSION 3.16)
project(mujocoProject_cpp)

find_package(yaml-cpp REQUIRED)
find_package(fcl REQUIRED)
find_package(OpenCV REQUIRED)

if(APPLE)
    function(clean_target_interface_libs target_name)
        if(TARGET ${target_name})
            get_target_property(INTERFACE_LIBS ${target_name} INTERFACE_LINK_LIBRARIES)

            if(INTERFACE_LIBS)
                set(CLEANED_LIBS "")

                foreach(lib ${INTERFACE_LIBS})
                    if(NOT lib MATCHES ".*MacOSX[0-9]+\\.sdk.*" AND NOT lib MATCHES ".*/libm\\..*")
                        list(APPEND CLEANED_LIBS ${lib})
                    else()
                        message(STATUS "Filtering out old SDK reference: ${lib}")
                    endif()
                endforeach()

                set_target_properties(${target_name} PROPERTIES INTERFACE_LINK_LIBRARIES "${CLEANED_LIBS}")
            endif()
        endif()
    endfunction()

    clean_target_interface_libs(fcl)

    foreach(opencv_target opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs opencv_videoio opencv_calib3d opencv_features2d opencv_objdetect opencv_dnn opencv_ml opencv_flann opencv_photo opencv_stitching opencv_video)
        clean_target_interface_libs(${opencv_target})
    endforeach()

    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-search_paths_first")
endif()

if(APPLE)
    set(BASE_INCLUDE_DIR /opt/homebrew/include)
    set(BASE_LIB_DIR /opt/homebrew/lib)

    set(YAML_LIB yaml-cpp::yaml-cpp)
    set(YAML_CPP_INCLUDE_DIR /opt/homebrew/opt/yaml-cpp/include)
    include_directories(/opt/homebrew/opt/yaml-cpp/include)

    set(GLFW_INCLUDE_DIR "${BASE_INCLUDE_DIR}/GLFW")
    set(GLFW_LIB "${BASE_LIB_DIR}/libglfw.dylib")

    set(LCM_INCLUDE_DIRS "${BASE_INCLUDE_DIR}")
    find_library(LCM_LIBRARY NAMES lcm HINTS ${BASE_LIB_DIR})

    if(EXISTS "${GLFW_INCLUDE_DIR}/glfw3.h" AND EXISTS "${GLFW_LIB}")
        message(STATUS "GLFW library found: ${GLFW_LIB}")
        include_directories(${GLFW_INCLUDE_DIR})
    else()
        message(FATAL_ERROR "GLFW library not found at expected location.")
    endif()
else()
    set(BASE_INCLUDE_DIR /usr/local/include)
    set(BASE_LIB_DIR /usr/local/lib)

    find_library(YAML_CPP_LIBRARY NAMES yaml-cpp HINTS ${BASE_LIB_DIR})
    set(YAML_LIB ${YAML_CPP_LIBRARY})
    set(YAML_CPP_INCLUDE_DIR /usr/local/include)

    include_directories(/usr/local/include)

    find_package(glfw3 REQUIRED)
    set(GLFW_INCLUDE_DIRS /usr/include/GLFW)

    set(LCM_INCLUDE_DIRS "${BASE_INCLUDE_DIR}")
    find_library(LCM_LIBRARY NAMES lcm HINTS ${BASE_LIB_DIR})
endif()

message(STATUS "Using MuJoCo library: ${MUJOCO_LIBRARY}")

set(COMMON_INCLUDE_DIRS
    include/
    ${COMMON_DIR}/data/
    ${COMMON_DIR}/lcm/piperlcm/
    ${COMMON_DIR}/lcm/lcmMotion/
    ${MOTION_DIR}/fsm/include
    ${MOTION_DIR}/manager/
    ${MOTION_DIR}/runtime/
    ${MOTION_DIR}/interface/
    ${MOTION_DIR}/module/
    ${MOTION_DIR}/module/estimation/
    ${MOTION_DIR}/module/joint/
    ${MOTION_DIR}/module/planner/
    ${MOTION_DIR}/module/osc/
    ${MOTION_DIR}/module/teleopfollow/
    ${MOTION_DIR}/utils/tools/
    ${MOTION_DIR}/utils/kinematics/
    ${MOTION_DIR}/utils/dynamics/
    ${MOTION_DIR}/utils/curve/
    ${MOTION_DIR}/utils/generator/
    ${MOTION_DIR}/utils/trajectory/
    ${MOTION_DIR}/utils/logger/
    ${MUJOCO_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIRS}
    ${UTILS_BUILD_INCLUDE_DIR}
    ${YAML_CPP_INCLUDE_DIR}
    ${LCM_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

if(APPLE)
    set(COMMON_LIBRARIES
        ${MUJOCO_LIBRARY}
        ${GLFW_LIB}
        ${YAML_LIB}
        ${LCM_LIBRARY}
    )
else()
    set(COMMON_LIBRARIES
        ${MUJOCO_LIBRARY}
        glfw
        ${YAML_LIB}
        ${LCM_LIBRARY}
    )
endif()

set(COMMON_LINK_DIRS
    ${MUJOCO_LIB_DIR}
    ${CMAKE_BINARY_DIR}/motion
    ${CMAKE_BINARY_DIR}/motion/utils
)

set(UTILS_BUILD_INCLUDE_DIR ${CMAKE_BINARY_DIR}/utility/include/)

add_executable(mujocoExe
    src/main_mj.cpp
    src/mj_interface.cpp
    src/mj_actuator.cpp
    src/mj_sensor.cpp
    src/mj_ui.cpp
    src/mj_scene.cpp
)

target_include_directories(mujocoExe PUBLIC ${COMMON_INCLUDE_DIRS})
target_link_directories(mujocoExe PRIVATE ${COMMON_LINK_DIRS})

if(APPLE)
    set_target_properties(mujocoExe PROPERTIES
        BUILD_RPATH "${CMAKE_BINARY_DIR}/motion;${CMAKE_BINARY_DIR}/motion/utils;${BASE_LIB_DIR}"
        INSTALL_RPATH "@executable_path/../lib;@loader_path/../lib;${BASE_LIB_DIR}"
    )
endif()

target_link_libraries(mujocoExe
    manager
    ${MOTION_LIBRARY}
    ${UTILITY_LIBRARY}
    ${COMMON_LIBRARIES}
    ${FCL_LIBRARIES}
    ${OpenCV_LIBS}
)

if(APPLE)
    set_target_properties(mujocoExe PROPERTIES
        LINK_DEPENDS ""
    )

    target_link_options(mujocoExe PRIVATE
        "LINKER:-w"
    )
endif()

add_dependencies(mujocoExe motion utility)

add_custom_command(TARGET mujocoExe POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:mujocoExe> ${CMAKE_SOURCE_DIR}/build/mujocoExe
)

if(APPLE)
    get_filename_component(MUJOCO_LIB_NAME ${MUJOCO_LIBRARY} NAME)

    add_custom_command(TARGET mujocoExe POST_BUILD
        COMMAND install_name_tool -change @rpath/mujoco.framework/Versions/A/${MUJOCO_LIB_NAME} ${MUJOCO_LIBRARY} $<TARGET_FILE:mujocoExe> || true
        COMMAND install_name_tool -change @rpath/mujoco.framework/Versions/A/${MUJOCO_LIB_NAME} ${MUJOCO_LIBRARY} ${CMAKE_SOURCE_DIR}/build/mujocoExe || true
        COMMAND install_name_tool -change liblcm.1.dylib ${BASE_LIB_DIR}/liblcm.1.dylib $<TARGET_FILE:mujocoExe> || true
        COMMAND install_name_tool -change liblcm.1.dylib ${BASE_LIB_DIR}/liblcm.1.dylib ${CMAKE_SOURCE_DIR}/build/mujocoExe || true
    )
endif()

if(BUILD_ROS_MUJOCO)
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
        message(STATUS "rm_ros_interfaces not found - Building without RM robot support (simulation only)")
    endif()

    find_library(CCD_LIBRARY NAMES ccd REQUIRED)

    if(NOT CCD_LIBRARY)
        message(FATAL_ERROR "libccd library not found")
    else()
        message(STATUS "Found libccd: ${CCD_LIBRARY}")
    endif()

    add_executable(rosMujocoExe
        src/ros_main_mj.cpp
        src/ros_mj_interface.cpp
        src/mj_interface.cpp
        src/mj_actuator.cpp
        src/mj_sensor.cpp
        src/mj_ui.cpp
        src/mj_scene.cpp
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

    target_include_directories(rosMujocoExe PUBLIC ${COMMON_INCLUDE_DIRS})
    target_link_directories(rosMujocoExe PRIVATE ${COMMON_LINK_DIRS})

    if(APPLE)
        set_target_properties(rosMujocoExe PROPERTIES
            BUILD_RPATH "${CMAKE_BINARY_DIR}/motion;${CMAKE_BINARY_DIR}/motion/utils;${BASE_LIB_DIR}"
            INSTALL_RPATH "@executable_path/../lib;@loader_path/../lib;${BASE_LIB_DIR}"
        )
    endif()

    target_link_libraries(rosMujocoExe
        manager
        motion
        utility
        ${COMMON_LIBRARIES}
        ${FCL_LIBRARIES}
        ${CCD_LIBRARY}
        ${OpenCV_LIBS}
    )

    add_dependencies(rosMujocoExe utility motion)

    add_custom_command(TARGET rosMujocoExe POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:rosMujocoExe> ${CMAKE_SOURCE_DIR}/build/rosMujocoExe
    )

    install(TARGETS rosMujocoExe
        DESTINATION lib/${PROJECT_NAME}
    )
endif()
