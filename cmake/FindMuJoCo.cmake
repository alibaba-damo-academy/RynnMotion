set(MUJOCO_VERSION_CANDIDATES "3.3.7" "3.3.5" "3.3.4" "3.3.3" "3.3.2" "3.3.1" "3.3.0")

function(find_mujoco)
    set(MUJOCO_DIR "" CACHE PATH "Path to MuJoCo installation")

    if(NOT MUJOCO_DIR OR NOT EXISTS "${MUJOCO_DIR}")
        if(APPLE)
            if(EXISTS "/opt/homebrew/include/mujoco" AND EXISTS "/opt/homebrew/lib")
                file(GLOB HOMEBREW_MUJOCO_LIBS "/opt/homebrew/lib/libmujoco*.dylib")

                if(HOMEBREW_MUJOCO_LIBS)
                    set(MUJOCO_DIR "/opt/homebrew" CACHE PATH "Path to MuJoCo installation" FORCE)
                    message(STATUS "Found MuJoCo in Homebrew: ${MUJOCO_DIR}")
                endif()
            endif()

            if(NOT MUJOCO_DIR)
                foreach(VERSION ${MUJOCO_VERSION_CANDIDATES})
                    set(CANDIDATE_DIR "/usr/local/mujoco-${VERSION}")

                    if(EXISTS "${CANDIDATE_DIR}")
                        set(MUJOCO_DIR "${CANDIDATE_DIR}" CACHE PATH "Path to MuJoCo installation" FORCE)
                        message(STATUS "Found MuJoCo installation at: ${MUJOCO_DIR}")
                        break()
                    endif()
                endforeach()
            endif()

            if(NOT MUJOCO_DIR OR NOT EXISTS "${MUJOCO_DIR}")
                set(MUJOCO_DIR "/usr/local/mujoco-3.3.7" CACHE PATH "Path to MuJoCo installation" FORCE)
                message(WARNING "MuJoCo not found, using default path: ${MUJOCO_DIR}")
            endif()
        else()
            foreach(VERSION ${MUJOCO_VERSION_CANDIDATES})
                set(CANDIDATE_DIR "/usr/local/mujoco-${VERSION}")

                if(EXISTS "${CANDIDATE_DIR}")
                    set(MUJOCO_DIR "${CANDIDATE_DIR}" CACHE PATH "Path to MuJoCo installation" FORCE)
                    message(STATUS "Found MuJoCo installation at: ${MUJOCO_DIR}")
                    break()
                endif()
            endforeach()

            if(NOT MUJOCO_DIR OR NOT EXISTS "${MUJOCO_DIR}")
                set(MUJOCO_DIR "/usr/local/mujoco-3.3.7" CACHE PATH "Path to MuJoCo installation" FORCE)
                message(WARNING "MuJoCo not found, using default path: ${MUJOCO_DIR}")
            endif()
        endif()
    endif()

    set(MUJOCO_LIB_DIR "${MUJOCO_DIR}/lib" CACHE PATH "Path to MuJoCo libraries" FORCE)
    set(MUJOCO_INCLUDE_DIR "${MUJOCO_DIR}/include" CACHE PATH "Path to MuJoCo headers" FORCE)

    if(MUJOCO_DIR MATCHES "mujoco-([0-9]+\\.[0-9]+\\.[0-9]+)")
        set(MUJOCO_VERSION "${CMAKE_MATCH_1}" CACHE STRING "MuJoCo version" FORCE)
    else()
        foreach(VERSION ${MUJOCO_VERSION_CANDIDATES})
            if(APPLE)
                if(EXISTS "${MUJOCO_LIB_DIR}/libmujoco.${VERSION}.dylib")
                    set(MUJOCO_VERSION "${VERSION}" CACHE STRING "MuJoCo version" FORCE)
                    break()
                endif()
            else()
                if(EXISTS "${MUJOCO_LIB_DIR}/libmujoco.${VERSION}.so")
                    set(MUJOCO_VERSION "${VERSION}" CACHE STRING "MuJoCo version" FORCE)
                    break()
                endif()
            endif()
        endforeach()

        if(NOT MUJOCO_VERSION)
            set(MUJOCO_VERSION "3.3.7" CACHE STRING "MuJoCo version" FORCE)
        endif()
    endif()

    if(APPLE)
        set(LIB_EXTENSIONS "dylib")
    else()
        set(LIB_EXTENSIONS "so")
    endif()

    set(LIB_NAMES "")
    foreach(VERSION ${MUJOCO_VERSION_CANDIDATES})
        foreach(EXT ${LIB_EXTENSIONS})
            list(APPEND LIB_NAMES "libmujoco.${VERSION}.${EXT}")
        endforeach()
    endforeach()
    list(APPEND LIB_NAMES "libmujoco.${LIB_EXTENSIONS}" "libmujoco" "mujoco")

    find_library(
        MUJOCO_LIBRARY
        NAMES ${LIB_NAMES}
        HINTS ${MUJOCO_LIB_DIR}
        PATHS ${MUJOCO_LIB_DIR}
        NO_DEFAULT_PATH
    )

    if(NOT MUJOCO_LIBRARY)
        message(WARNING "MuJoCo library not found in ${MUJOCO_LIB_DIR}")
        set(MUJOCO_FOUND FALSE PARENT_SCOPE)
        return()
    endif()

    message(STATUS "Found MuJoCo library: ${MUJOCO_LIBRARY}")
    message(STATUS "MuJoCo version: ${MUJOCO_VERSION}")
    message(STATUS "MuJoCo include dir: ${MUJOCO_INCLUDE_DIR}")
    message(STATUS "MuJoCo lib dir: ${MUJOCO_LIB_DIR}")

    if(APPLE AND EXISTS "${MUJOCO_INCLUDE_DIR}")
        set(MUJOCO_BUILD_INCLUDE "${CMAKE_CURRENT_BINARY_DIR}/mujoco_include")
        set(MUJOCO_SUBDIR "${MUJOCO_BUILD_INCLUDE}/mujoco")

        file(MAKE_DIRECTORY "${MUJOCO_BUILD_INCLUDE}")
        file(MAKE_DIRECTORY "${MUJOCO_SUBDIR}")

        file(GLOB MUJOCO_HEADERS "${MUJOCO_INCLUDE_DIR}/*.h")

        foreach(HEADER ${MUJOCO_HEADERS})
            get_filename_component(HEADER_NAME ${HEADER} NAME)
            set(SYMLINK_PATH "${MUJOCO_SUBDIR}/${HEADER_NAME}")

            if(NOT EXISTS "${SYMLINK_PATH}")
                execute_process(
                    COMMAND ${CMAKE_COMMAND} -E create_symlink "${HEADER}" "${SYMLINK_PATH}"
                    RESULT_VARIABLE SYMLINK_RESULT
                )
            endif()
        endforeach()

        set(MUJOCO_INCLUDE_DIRS "${MUJOCO_INCLUDE_DIR};${MUJOCO_BUILD_INCLUDE}" CACHE STRING "MuJoCo include directories" FORCE)
    else()
        set(MUJOCO_INCLUDE_DIRS "${MUJOCO_INCLUDE_DIR}" CACHE STRING "MuJoCo include directories" FORCE)
    endif()

    set(MUJOCO_DIR "${MUJOCO_DIR}" PARENT_SCOPE)
    set(MUJOCO_LIB_DIR "${MUJOCO_LIB_DIR}" PARENT_SCOPE)
    set(MUJOCO_INCLUDE_DIR "${MUJOCO_INCLUDE_DIR}" PARENT_SCOPE)
    set(MUJOCO_INCLUDE_DIRS "${MUJOCO_INCLUDE_DIRS}" PARENT_SCOPE)
    set(MUJOCO_LIBRARY "${MUJOCO_LIBRARY}" PARENT_SCOPE)
    set(MUJOCO_VERSION "${MUJOCO_VERSION}" PARENT_SCOPE)
    set(MUJOCO_FOUND TRUE PARENT_SCOPE)
endfunction()
