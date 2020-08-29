cmake_minimum_required (VERSION 3.17 FATAL_ERROR)
set(CMAKE_VERBOSE_MAKEFILE ON) 
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")
set(CMAKE_MESSAGE_LOG_LEVEL VERBOSE)
set(PATHSEP ":")
if(CMAKE_HOST_WIN32)
    set(PATHSEP ";")
endif()

set(ENV{VCPKG_PORT_OVERLAYS} "$ENV{VCPKG_PORT_OVERLAYS}${PATHSEP}${CMAKE_CURRENT_SOURCE_DIR}/../my-vcpkg-ports")

set(VCPKG_TARGET_TRIPLET "x64-windows-llvm-static" CACHE STRING "")
find_package(VCPKG)
if(NOT CMakeCS_FOUND)
    find_package(CMakeCS REQUIRED)
endif()
include(cmake/GeneralOptions.cmake)
include(cmake/load_vcpkg_toolchain.cmake)
include(cmake/Watcher.cmake)
include(cmake/DefaultProjectSettings.cmake)
include(cmake/CompilerOptions.cmake)
include(cmake/StaticAnalyzers.cmake)
include(cmake/Sanitizers.cmake)


# about this project
project("StoPhysics.cmakecs")


#set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT Particle_Simulation)
#set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Particle_Simulation PROPERTY VS_STARTUP_PROJECT Particle_Simulation)
