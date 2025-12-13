cmake_minimum_required(VERSION 3.16)
project(mujoco_simulation)

option(RYNN_USE_PARQUET "Enable Parquet recording support" ON)
option(RYNN_USE_HDF5 "Enable HDF5 recording support" OFF)

if(APPLE)
    set(PLATFORM_LIB_DIR /opt/homebrew/lib)
    set(PLATFORM_INCLUDE_DIR /opt/homebrew/include)
    set(YAML_LIB yaml-cpp::yaml-cpp)
    set(GLFW_LIB "${PLATFORM_LIB_DIR}/libglfw.dylib")
    include_directories(/opt/homebrew/opt/yaml-cpp/include)
else()
    set(PLATFORM_LIB_DIR /usr/local/lib)
    set(PLATFORM_INCLUDE_DIR /usr/local/include)
    find_library(YAML_CPP_LIBRARY NAMES yaml-cpp HINTS ${PLATFORM_LIB_DIR})
    set(YAML_LIB ${YAML_CPP_LIBRARY})
    find_package(glfw3 REQUIRED)
    set(GLFW_LIB glfw)
    include_directories(/usr/local/include)
endif()

find_package(yaml-cpp REQUIRED)
find_package(fcl REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
    libavcodec
    libavformat
    libavutil
    libswscale
)

find_library(LCM_LIBRARY NAMES lcm HINTS ${PLATFORM_LIB_DIR})

if(APPLE)
    if(NOT EXISTS "${PLATFORM_LIB_DIR}/libglfw.dylib")
        message(FATAL_ERROR "GLFW library not found at ${PLATFORM_LIB_DIR}")
    endif()
    include_directories(${PLATFORM_INCLUDE_DIR}/GLFW)
else()
    include_directories(/usr/include/GLFW)
endif()

message(STATUS "Using MuJoCo library: ${MUJOCO_LIBRARY}")

set(PARQUET_AVAILABLE FALSE)
if(RYNN_USE_PARQUET)
    set(_OLD_LOG_LEVEL ${CMAKE_MESSAGE_LOG_LEVEL})
    set(CMAKE_MESSAGE_LOG_LEVEL WARNING)
    find_package(Arrow QUIET)
    find_package(Parquet QUIET)
    set(CMAKE_MESSAGE_LOG_LEVEL ${_OLD_LOG_LEVEL})

    if(Arrow_FOUND AND Parquet_FOUND)
        add_compile_definitions(RYNN_USE_PARQUET)
        set(PARQUET_AVAILABLE TRUE)
        message(STATUS "Parquet recording: ENABLED")
    else()
        message(STATUS "Parquet recording: DISABLED (Arrow/Parquet not found)")
    endif()
endif()

set(HDF5_AVAILABLE FALSE)
if(RYNN_USE_HDF5)
    find_package(HDF5 QUIET COMPONENTS CXX)
    if(HDF5_FOUND)
        add_compile_definitions(RYNN_USE_HDF5)
        set(HDF5_AVAILABLE TRUE)
        message(STATUS "HDF5 recording: ENABLED")
    else()
        message(STATUS "HDF5 recording: DISABLED (HDF5 not found)")
    endif()
endif()

if(APPLE)
    function(clean_target_interface_libs target_name)
        if(TARGET ${target_name})
            get_target_property(INTERFACE_LIBS ${target_name} INTERFACE_LINK_LIBRARIES)
            if(INTERFACE_LIBS)
                set(CLEANED_LIBS "")
                foreach(lib ${INTERFACE_LIBS})
                    if(NOT lib MATCHES ".*MacOSX[0-9]+\\.sdk.*" AND NOT lib MATCHES ".*/libm\\..*")
                        list(APPEND CLEANED_LIBS ${lib})
                    endif()
                endforeach()
                set_target_properties(${target_name} PROPERTIES INTERFACE_LINK_LIBRARIES "${CLEANED_LIBS}")
            endif()
        endif()
    endfunction()

    clean_target_interface_libs(fcl)
    foreach(opencv_target opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs
            opencv_videoio opencv_calib3d opencv_features2d opencv_objdetect
            opencv_dnn opencv_ml opencv_flann opencv_photo opencv_stitching opencv_video)
        clean_target_interface_libs(${opencv_target})
    endforeach()

    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-search_paths_first")
endif()

set(MUJOCO_SOURCES
    src/mj_interface.cpp
    src/mj_actuator.cpp
    src/mj_sensor.cpp
    src/mj_ui.cpp
    src/mj_scene.cpp
    src/mj_data_writer.cpp
    src/mj_hdf5_writer.cpp
    src/mj_parquet_writer.cpp
    src/mj_video_encoder.cpp
    src/mj_recorder.cpp
)

set(MUJOCO_INCLUDE_DIRS
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
    ${PLATFORM_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${LIBAV_INCLUDE_DIRS}
)

if(HDF5_AVAILABLE)
    list(APPEND MUJOCO_INCLUDE_DIRS ${HDF5_INCLUDE_DIRS})
endif()

if(PARQUET_AVAILABLE)
    list(APPEND MUJOCO_INCLUDE_DIRS ${Arrow_INCLUDE_DIRS} ${Parquet_INCLUDE_DIRS})
endif()

set(MUJOCO_LINK_DIRS
    ${MUJOCO_LIB_DIR}
    ${CMAKE_BINARY_DIR}/motion
    ${CMAKE_BINARY_DIR}/motion/utils
)

set(MUJOCO_COMMON_LIBS
    ${MUJOCO_LIBRARY}
    ${GLFW_LIB}
    ${YAML_LIB}
    ${LCM_LIBRARY}
)

add_executable(mujocoExe
    src/main_mj.cpp
    ${MUJOCO_SOURCES}
)

target_include_directories(mujocoExe PUBLIC ${MUJOCO_INCLUDE_DIRS})
target_link_directories(mujocoExe PRIVATE ${MUJOCO_LINK_DIRS})

if(APPLE)
    set_target_properties(mujocoExe PROPERTIES
        BUILD_RPATH "${CMAKE_BINARY_DIR}/motion;${CMAKE_BINARY_DIR}/motion/utils;${PLATFORM_LIB_DIR}"
        INSTALL_RPATH "@executable_path/../lib;@loader_path/../lib;${PLATFORM_LIB_DIR}"
        LINK_DEPENDS ""
    )
    target_link_options(mujocoExe PRIVATE "LINKER:-w")
endif()

target_link_libraries(mujocoExe
    manager
    ${MOTION_LIBRARY}
    ${UTILITY_LIBRARY}
    ${MUJOCO_COMMON_LIBS}
    ${FCL_LIBRARIES}
    ${OpenCV_LIBS}
    PkgConfig::LIBAV
)

if(HDF5_AVAILABLE)
    target_link_libraries(mujocoExe ${HDF5_CXX_LIBRARIES})
endif()

if(PARQUET_AVAILABLE)
    target_link_libraries(mujocoExe Arrow::arrow_shared Parquet::parquet_shared)
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
        COMMAND install_name_tool -change liblcm.1.dylib ${PLATFORM_LIB_DIR}/liblcm.1.dylib $<TARGET_FILE:mujocoExe> || true
        COMMAND install_name_tool -change liblcm.1.dylib ${PLATFORM_LIB_DIR}/liblcm.1.dylib ${CMAKE_SOURCE_DIR}/build/mujocoExe || true
    )
endif()

if(BUILD_ROS_MUJOCO)
    include(${CMAKE_SOURCE_DIR}/cmake/RosMujoco.cmake)
endif()
