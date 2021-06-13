### Module to find VCPKG root and setup the following variables
## VCPKG_ROOT searched in ${CMAKE_(SOURCE|BINARY)_DIR}(/..|/../..)/vcpkg, variable VCPKG_HINTS and environment VCPKG_ROOT
## VCPKG_TARGET_TRIPLET if not set by the user

function(DEDUCE_VCPKG_CRT_LINKAGE)
    # This is a function to not leak variables from the triplet into the project
    if(EXISTS "${VCPKG_ROOT}/triplets/${VCPKG_TARGET_TRIPLET}.cmake")
        include("${VCPKG_ROOT}/triplets/${VCPKG_TARGET_TRIPLET}.cmake")
        if(VCPKG_CRT_LINKAGE STREQUAL static)
            set(WITH_STATIC_CRT ON CACHE BOOL "" FORCE)
        else()
            set(WITH_STATIC_CRT OFF CACHE BOOL "" FORCE)
        endif()
        return()
    endif()
    foreach(_file_location IN_LISTS VCPKG_OVERLAY_TRIPLETS)
        if(EXISTS "${_file_location}")
            include("${_file_location}")
            if(VCPKG_CRT_LINKAGE STREQUAL static)
                set(WITH_STATIC_CRT ON CACHE BOOL "" FORCE)
            else()
                set(WITH_STATIC_CRT OFF CACHE BOOL "" FORCE)
            endif()
        return()
        endif()
    endforeach()
endfunction()

include(FeatureSummary)
include(CMakePrintHelpers)
option(USE_VCPKG_TOOLCHAIN "Use VCPKG toolchain; Switching this option requires a clean reconfigure" ON) 

#TODO: Use fetch_content to get vcpkg

list(APPEND VCPKG_HINTS "${CMAKE_SOURCE_DIR}/../vcpkg/;${CMAKE_SOURCE_DIR}/vcpkg/;${CMAKE_BINARY_DIR}/../../vcpkg/;${CMAKE_BINARY_DIR}/../vcpkg/;${CMAKE_BINARY_DIR}/vcpkg/")
if(CMAKE_TOOLCHAIN_FILE AND EXISTS "${CMAKE_TOOLCHAIN_FILE}")
    get_filename_component(VCPKG_TOOLCHAIN_PATH "${CMAKE_TOOLCHAIN_FILE}" DIRECTORY)
    list(APPEND VCPKG_HINTS "${VCPKG_TOOLCHAIN_PATH}/../../")
endif()
find_path(VCPKG_ROOT NAMES .vcpkg-root HINTS ${VCPKG_HINTS} PATHS "./vcpkg" "./../vcpkg" "./../../vcpkg" $ENV{VCPKG_ROOT})
if(EXISTS "${VCPKG_ROOT}")
    if(NOT CMAKE_TOOLCHAIN_FILE AND USE_VCPKG_TOOLCHAIN)
        set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
    endif()
elseif(VCPKG_FIND_REQUIRED)
    message(WARNING "Could not find VCPKG! ${VCPKG_ROOT} with hints: ${VCPKG_HINTS}")
    message(STATUS "Downloading vcpkg.")
    include(FetchContent)
    FetchContent_Declare(
        vcpkg
        GIT_REPOSITORY https://github.com/microsoft/vcpkg
        GIT_TAG        master
        SOURCE_DIR ${CMAKE_SOURCE_DIR}/vcpkg
        )
    FetchContent_GetProperties(vcpkg)
    if(NOT vcpkg_POPULATED)
        FetchContent_Populate(vcpkg)
    endif()
    find_path(VCPKG_ROOT NAMES .vcpkg-root HINTS ${VCPKG_HINTS} PATHS "./vcpkg")
endif()

if(CMAKE_HOST_WIN32 AND EXISTS "${VCPKG_ROOT}/triplets/x64-windows-llvm-static.cmake")
    set(VCPKG_TARGET_TRIPLET "x64-windows-llvm-static" CACHE STRING "")
endif()
if(NOT VCPKG_TARGET_TRIPLET)
    set(VCPKG_DEFAULT_ARCH x64)
    if(DEFINED ENV{VCPKG_DEFAULT_TRIPLET})
        set(VCPKG_TARGET_TRIPLET "$ENV{VCPKG_DEFAULT_TRIPLET}" CACHE STRING "" FORCE)
        message(STATUS "Default triplet found in environment: $ENV{VCPKG_DEFAULT_TRIPLET}")
    elseif(WINDOWS_STORE)
        set(VCPKG_TARGET_TRIPLET "${VCPKG_DEFAULT_ARCH}-uwp" CACHE STRING "" FORCE)
    elseif(WIN32)
        set(VCPKG_TARGET_TRIPLET "${VCPKG_DEFAULT_ARCH}-windows" CACHE STRING "" FORCE)
    elseif(MINGW)
        set(VCPKG_TARGET_TRIPLET "${VCPKG_DEFAULT_ARCH}-mingw" CACHE STRING "" FORCE)
    elseif(IOS)
        set(VCPKG_TARGET_TRIPLET "${VCPKG_DEFAULT_ARCH}-ios" CACHE STRING "" FORCE)
    elseif(APPLE)
        set(VCPKG_TARGET_TRIPLET "${VCPKG_DEFAULT_ARCH}-osx" CACHE STRING "" FORCE)
    elseif(ANDROID)
        set(VCPKG_TARGET_TRIPLET "${VCPKG_DEFAULT_ARCH}-android" CACHE STRING "" FORCE)
    elseif(UNIX)
        set(VCPKG_TARGET_TRIPLET "${VCPKG_DEFAULT_ARCH}-linux" CACHE STRING "" FORCE)
    else()
        message(FATAL_ERROR "Unknown system or architecture. Cannot deduce VCPKG_TARGET_TRIPLET!")
    endif()
endif()


cmake_print_variables(VCPKG_ROOT CMAKE_TOOLCHAIN_FILE)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(VCPKG REQUIRED_VARS "VCPKG_ROOT;VCPKG_TARGET_TRIPLET")
set("VCPKG_MANIFEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vcpkg_manifest")
DEDUCE_VCPKG_CRT_LINKAGE()

set_package_properties(VCPKG PROPERTIES
                             DESCRIPTION "C++ Library Manager for Windows, Linux, and MacOS."
                             URL "https://github.com/microsoft/vcpkg")