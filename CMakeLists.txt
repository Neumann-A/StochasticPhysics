cmake_minimum_required (VERSION 3.17 FATAL_ERROR)
set(CMAKE_VERBOSE_MAKEFILE ON) 
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")
set(CMAKE_MESSAGE_LOG_LEVEL VERBOSE)
find_package(VCPKG)
find_package(CMakeCS)
#include(cmake/load_vcpkg_toolchain.cmake)
include(cmake/Watcher.cmake)
include(cmake/DefaultProjectSettings.cmake)
include(cmake/CompilerOptions.cmake)
include(cmake/StaticAnalyzers.cmake)
include(cmake/Sanitizers.cmake)
include(cmake/GeneralOptions.cmake)

# about this project
project("StoPhysics.cmakecs")

#set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT Particle_Simulation)
#set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Particle_Simulation PROPERTY VS_STARTUP_PROJECT Particle_Simulation)
