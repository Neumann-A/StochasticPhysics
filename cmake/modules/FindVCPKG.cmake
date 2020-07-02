### Module to find VCPKG root and setup the following variables
## VCPKG_ROOT searched in ${CMAKE_(SOURCE|BINARY)_DIR}(/..|/../..)/vcpkg, variable VCPKG_HINTS and environment VCPKG_ROOT
## VCPKG_TARGET_TRIPLET if not set by the user

include(FeatureSummary)
include(CMakePrintHelpers)
option(USE_VCPKG_TOOLCHAIN "Use VCPKG toolchain; Switching this option requires a clean reconfigure" ON) 


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
list(APPEND VCPKG_HINTS "${CMAKE_SOURCE_DIR}/../vcpkg/;${CMAKE_SOURCE_DIR}/vcpkg/;${CMAKE_BINARY_DIR}/../../vcpkg/;${CMAKE_BINARY_DIR}/../vcpkg/;${CMAKE_BINARY_DIR}/vcpkg/")
if(CMAKE_TOOLCHAIN_FILE AND EXISTS "${CMAKE_TOOLCHAIN_FILE}")
    get_filename_component(VCPKG_TOOLCHAIN_PATH "${CMAKE_TOOLCHAIN_FILE}" DIRECTORY)
    list(APPEND VCPKG_HINTS "${VCPKG_TOOLCHAIN_PATH}/../../")
endif()
find_path(VCPKG_ROOT NAMES .vcpkg-root HINTS ${VCPKG_HINTS} PATHS "./vcpkg" "./../vcpkg" "./vcpkg" "./../../vcpkg" $ENV{VCPKG_ROOT})
if(EXISTS "${VCPKG_ROOT}")
    if(NOT CMAKE_TOOLCHAIN_FILE AND USE_VCPKG_TOOLCHAIN)      
        set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
    endif()
    set(VCPKG_INSTALLED_DIR "${VCPKG_ROOT}/installed/${VCPKG_TARGET_TRIPLET}" CACHE PATH "")
    if(NOT EXISTS "${VCPKG_INSTALLED_DIR}")
        message(STATUS "VCPKG with triplet ${VCPKG_TARGET_TRIPLET} seems to be empty!")
    endif()
    cmake_print_variables(VCPKG_ROOT VCPKG_TARGET_TRIPLET CMAKE_TOOLCHAIN_FILE)
elseif(VCPKG_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find VCPKG! ${VCPKG_ROOT} with hints: ${VCPKG_HINTS}")
endif()


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(VCPKG REQUIRED_VARS "VCPKG_INSTALLED_DIR;VCPKG_ROOT;VCPKG_TARGET_TRIPLET")

set_package_properties(VCPKG PROPERTIES
                             DESCRIPTION "C++ Library Manager for Windows, Linux, and MacOS."
                             URL "https://github.com/microsoft/vcpkg")