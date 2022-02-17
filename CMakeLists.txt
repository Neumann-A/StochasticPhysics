cmake_minimum_required (VERSION 3.21 FATAL_ERROR)
set(CMAKE_VERBOSE_MAKEFILE ON) 
include(cmake/modules.cmake)
set(CMAKE_MESSAGE_LOG_LEVEL VERBOSE)

include(cmake/vcpkg_settings.cmake)
include(cmake/platform.cmake)
find_package(VCPKG)
if(NOT CMakeJSON_FOUND)
    find_package(CMakeJSON REQUIRED)
endif()
include(cmake/GeneralOptions.cmake)
include(cmake/toolchains.cmake)
include(cmake/Watcher.cmake)
include(cmake/DefaultProjectSettings.cmake)
include(cmake/CompilerOptions.cmake)
include(cmake/StaticAnalyzers.cmake)
include(cmake/Sanitizers.cmake)

# Some GitHub runners have not enough resources to compile this project multithreaded.
if(StoPhys_LIMITED_RESOURCE)
    set_property(GLOBAL APPEND PROPERTY JOB_POOLS one_job=1)
    set(CMAKE_JOB_POOL_COMPILE one_job)
    set(CMAKE_JOB_POOL_LINK one_job)
endif()

project("StoPhysics.json")



