cmake_minimum_required (VERSION 3.8 FATAL_ERROR)
include(CMakeToolsHelpers OPTIONAL)
set(CMAKE_VERBOSE_MAKEFILE ON) 
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(Watcher)
include(PackageToolchain)
include(ArchiveOptions)
include(SimulationOptions)

# about this project
project (Everything)
include(CompilerOptions)

set(VERSION_MAJOR "1")
set(VERSION_MINOR "0")
set(VERSION_PATCH "0")
set(VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")

set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

# Set the output folder where your program will be created
set(CMAKE_BINARY_DIR ${CMAKE_BINARY_DIR})
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

message(STATUS "Binary output path: " ${CMAKE_BINARY_DIR})
message(STATUS "EXECUTABLE output path: " ${EXECUTABLE_OUTPUT_PATH})
message(STATUS "Library output path: " ${LIBRARY_OUTPUT_PATH})

if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif ()

#Basic First
add_subdirectory (Basic_Library)

#Archive Second
add_subdirectory (Archive) 
#add_subdirectory (Archives_Test)

#Rest third
add_subdirectory (Particle_Simulation) 
#add_executable(${PROJECT_NAME} ALIAS Particle_Simulation)

add_subdirectory (Tests)
add_subdirectory (Benchmarks)

#add_subdirectory (Performance_ParticleSimulation) 
SET(CMAKE_DEFAULT_STARTUP_PROJECT Particle_Simulation)
#set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Particle_Simulation PROPERTY VS_STARTUP_PROJECT Particle_Simulation)
