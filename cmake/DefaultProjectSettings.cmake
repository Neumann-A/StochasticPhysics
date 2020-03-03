#Default Options
option(BUILD_SHARED_LIBS "Enable compilation of shared libraries" OFF)
option(BUILD_TESTING  "Enable build of tests" OFF)
option(BUILD_BENCHMARKS "Enable build of benchmarks" OFF)
option(ENABLE_IPO "Enable Iterprocedural Optimization, aka Link Time Optimization (LTO)" OFF)
option(ENABLE_PCH "Enable Precompiled Headers" OFF)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui, ccmake
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Use ccache if available
find_program(CCACHE ccache)
if(CCACHE)
  message("using ccache")
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE})
else()
  message("ccache not available")
endif()

# Generate compile_commands.json to make it easier to work with clang based tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE STRING "Generate compile_commands.json to make it easier to work with clang based tools")

if(ENABLE_IPO)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT result OUTPUT output LANGUAGES C CXX)
  if(result)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_DEBUG FALSE)
  else()
    message(SEND_ERROR "IPO is not supported: ${output}")
  endif()
endif()

set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES ON)

if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif ()

# Very basic PCH example
add_library(global_pch INTERFACE)
if(TARGET global_options)
    target_link_libraries(global_pch INTERFACE global_options)
endif()
if (ENABLE_PCH)
  # This sets a global PCH parameter, each project will build its own PCH, which
  # is a good idea if any #define's change
  target_precompile_headers(global_pch INTERFACE <vector> <string> <map> <utility> <array> <type_traits> <cstdint> <memory> <cmath> )
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON CACHE STRING "Compile using position independent code")

