cmake_minimum_required (VERSION 3.16 FATAL_ERROR)
set(CMAKE_VERBOSE_MAKEFILE ON) 
#set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(CMakeToolsHelpers OPTIONAL)
include(GNUInstallDirs)
include(cmake/load_vcpkg_toolchain.cmake)
include(cmake/Watcher.cmake)
include(cmake/DefaultProjectSettings.cmake)
include(cmake/CompilerOptions.cmake)
include(cmake/StaticAnalyzers.cmake)
include(cmake/Sanitizers.cmake)
include(cmake/GeneralOptions.cmake)


# about this project
project(Everything VERSION 1.0.0 LANGUAGES CXX)

# Set the output folder where your program will be created
set(CMAKE_BINARY_DIR ${CMAKE_BINARY_DIR})
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

message(STATUS "Binary output path: " ${CMAKE_BINARY_DIR})
message(STATUS "EXECUTABLE output path: " ${EXECUTABLE_OUTPUT_PATH})
message(STATUS "Library output path: " ${LIBRARY_OUTPUT_PATH})
message(STATUS "CMAKE Toolchain file: " ${CMAKE_TOOLCHAIN_FILE})

#Basic First
add_subdirectory (Basic_Library)

#Archive Second
add_subdirectory (Archive) 
#add_subdirectory (Archives_Test)

#Rest third
add_subdirectory (Particle_Simulation) 

if(BUILD_TESTING)
    add_subdirectory (Tests)
endif()
if(BUILD_BENCHMARKS)
    add_subdirectory (Benchmarks)
endif()

#add_subdirectory (Performance_ParticleSimulation) 

add_subdirectory(Playground)

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT Particle_Simulation)
#set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Particle_Simulation PROPERTY VS_STARTUP_PROJECT Particle_Simulation)
