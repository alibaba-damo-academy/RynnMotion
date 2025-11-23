function(find_motion_library OUT_VAR LIB_NAME)
  find_package(${LIB_NAME} QUIET)
  if(${LIB_NAME}_FOUND)
    message(STATUS "Found ${LIB_NAME} via find_package: ${${LIB_NAME}_LIBRARIES}")
    set(${OUT_VAR} ${${LIB_NAME}_LIBRARIES} PARENT_SCOPE)
    return()
  endif()

  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" AND CMAKE_SYSTEM_NAME MATCHES "Linux")
    find_library(${OUT_VAR}
      NAMES lib${LIB_NAME}.a lib${LIB_NAME}.so ${LIB_NAME}
      HINTS
        /usr/aarch64-linux-gnu/lib
        /usr/aarch64-linux-gnu/lib64
    )
  else()
    find_library(${OUT_VAR}
      NAMES ${LIB_NAME} lib${LIB_NAME}.so lib${LIB_NAME}.a
      HINTS
        /usr/local/lib
        /usr/lib
        /usr/lib/x86_64-linux-gnu
        /usr/lib64
        /usr/local/lib64
    )
  endif()

  if(${OUT_VAR})
    message(STATUS "Found ${LIB_NAME}: ${${OUT_VAR}}")
  else()
    message(WARNING "Could not find library: ${LIB_NAME}")
  endif()

  set(${OUT_VAR} ${${OUT_VAR}} PARENT_SCOPE)
endfunction()

function(configure_motion_target TARGET_NAME)
  target_include_directories(${TARGET_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  target_link_libraries(${TARGET_NAME} PUBLIC
    Eigen3::Eigen
  )

  target_compile_definitions(${TARGET_NAME} PUBLIC
    MODEL_DIR="${MODEL_DIR}"
    MOTION_DIR="${MOTION_DIR}"
  )

  set_target_properties(${TARGET_NAME} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
  )
endfunction()

function(copy_to_deployment_dirs TARGET_NAME)
  if(NOT BUILD_STATIC)
    return()
  endif()

  set(DEPLOYMENT_DIRS
    ${CMAKE_SOURCE_DIR}/robots/damiao/lib
    ${CMAKE_SOURCE_DIR}/robots/piper/src/xiaoda/lib
    ${CMAKE_SOURCE_DIR}/robots/franka/src/libmotion/lib
    ${CMAKE_SOURCE_DIR}/robots/realman/src/telemotion/lib
  )

  foreach(DIR ${DEPLOYMENT_DIRS})
    add_custom_command(
      TARGET ${TARGET_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${TARGET_NAME}> ${DIR}/
      COMMENT "Copying ${TARGET_NAME} to ${DIR}"
    )
  endforeach()
endfunction()

set(COMMON_INCLUDE_DIRS
  ${CMAKE_SOURCE_DIR}/common/lcm/lcmMotion/
  ${CMAKE_SOURCE_DIR}/motion/manager
  ${CMAKE_SOURCE_DIR}/motion/messages
  ${CMAKE_SOURCE_DIR}/motion/interface
  ${CMAKE_SOURCE_DIR}/motion/module
  ${CMAKE_SOURCE_DIR}/motion/utils/tools
  ${CMAKE_SOURCE_DIR}/motion/utils/kinematics
  ${CMAKE_SOURCE_DIR}/motion/utils/dynamics
  ${CMAKE_SOURCE_DIR}/motion/utils/collision
  ${CMAKE_SOURCE_DIR}/motion/utils/curve
  ${CMAKE_SOURCE_DIR}/motion/utils/generator
  ${CMAKE_SOURCE_DIR}/motion/utils/trajectory
  ${CMAKE_SOURCE_DIR}/motion/utils/logger
  ${CMAKE_SOURCE_DIR}/motion/utils/sketch/include
)
