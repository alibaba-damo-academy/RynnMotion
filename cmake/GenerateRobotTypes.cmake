function(generate_robot_types)
  cmake_parse_arguments(
    ARG
    ""
    "MODELS_DIR;OUTPUT_FILE"
    ""
    ${ARGN}
  )

  if(NOT ARG_MODELS_DIR)
    message(FATAL_ERROR "MODELS_DIR argument required")
  endif()

  if(NOT ARG_OUTPUT_FILE)
    message(FATAL_ERROR "OUTPUT_FILE argument required")
  endif()

  get_filename_component(OUTPUT_DIR "${ARG_OUTPUT_FILE}" DIRECTORY)
  file(MAKE_DIRECTORY "${OUTPUT_DIR}")

  set(HEADER_CONTENT "// Auto-generated file - DO NOT EDIT MANUALLY
// Generated from models/ directory structure at build time
// To regenerate: Run CMake or rebuild project
//
// This file contains RobotType enum for all discovered robots.
// Format: models/{category}/{NUMBER}.{robot_name}/ → RobotType::{robot_name} = {NUMBER}

#pragma once

namespace rynn {

/**
 * @brief Auto-discovered robot type enumeration
 *
 * This enum is automatically generated from the models/ directory structure.
 * Each robot directory XX.robot_name creates enum value RobotType::robot_name = XX
 */
enum class RobotType {
  Undefined = 0,
")

  file(GLOB CATEGORY_DIRS "${ARG_MODELS_DIR}/*")

  set(ROBOT_LIST "")

  foreach(CATEGORY_DIR ${CATEGORY_DIRS})
    if(IS_DIRECTORY "${CATEGORY_DIR}")
      file(GLOB ROBOT_DIRS "${CATEGORY_DIR}/*")

      foreach(ROBOT_DIR ${ROBOT_DIRS})
        if(IS_DIRECTORY "${ROBOT_DIR}")
          get_filename_component(DIR_NAME "${ROBOT_DIR}" NAME)

          if(DIR_NAME MATCHES "^([0-9]+)\\.(.+)$")
            set(ROBOT_NUMBER "${CMAKE_MATCH_1}")
            set(ROBOT_NAME "${CMAKE_MATCH_2}")

            list(APPEND ROBOT_LIST "${ROBOT_NUMBER}:${ROBOT_NAME}")
          endif()
        endif()
      endforeach()
    endif()
  endforeach()

  list(SORT ROBOT_LIST)

  set(PREV_CATEGORY "")
  set(FIRST_ENTRY TRUE)
  foreach(ROBOT_ENTRY ${ROBOT_LIST})
    string(REPLACE ":" ";" ENTRY_PARTS "${ROBOT_ENTRY}")
    list(GET ENTRY_PARTS 0 NUMBER)
    list(GET ENTRY_PARTS 1 NAME)

    set(CATEGORY "")
    if(NUMBER GREATER_EQUAL 1 AND NUMBER LESS 10)
      set(CATEGORY "Low DOF test robots")
    elseif(NUMBER GREATER_EQUAL 10 AND NUMBER LESS 20)
      set(CATEGORY "End effectors")
    elseif(NUMBER GREATER_EQUAL 20 AND NUMBER LESS 30)
      set(CATEGORY "Single arm manipulators")
    elseif(NUMBER GREATER_EQUAL 30 AND NUMBER LESS 40)
      set(CATEGORY "LeRobot/SO-ARM platforms")
    elseif(NUMBER GREATER_EQUAL 40 AND NUMBER LESS 50)
      set(CATEGORY "Dual arm systems")
    elseif(NUMBER GREATER_EQUAL 50 AND NUMBER LESS 60)
      set(CATEGORY "Mobile bases")
    elseif(NUMBER GREATER_EQUAL 60 AND NUMBER LESS 70)
      set(CATEGORY "Mobile manipulators")
    endif()

    if(NOT "${CATEGORY}" STREQUAL "${PREV_CATEGORY}")
      if(NOT "${PREV_CATEGORY}" STREQUAL "")
        string(APPEND HEADER_CONTENT "\n")
      endif()
      string(APPEND HEADER_CONTENT "  // ${CATEGORY} (${NUMBER}-...)\n")
      set(PREV_CATEGORY "${CATEGORY}")
    endif()

    if(NOT FIRST_ENTRY)
      string(APPEND HEADER_CONTENT ",\n")
    endif()
    string(APPEND HEADER_CONTENT "  ${NAME} = ${NUMBER}")
    set(FIRST_ENTRY FALSE)
  endforeach()

  string(APPEND HEADER_CONTENT "
}; // enum class RobotType

// Utility to convert enum to int for range comparisons
constexpr int toInt(RobotType type) {
  return static_cast<int>(type);
}

} // namespace rynn
")

  file(WRITE "${ARG_OUTPUT_FILE}" "${HEADER_CONTENT}")

  list(LENGTH ROBOT_LIST ROBOT_COUNT)

  message(STATUS "Generated robot types: ${ARG_OUTPUT_FILE} (${CMAKE_CURRENT_LIST_FILE})")
  message(STATUS "  Found ${ROBOT_COUNT} robots in ${ARG_MODELS_DIR}")

endfunction()
