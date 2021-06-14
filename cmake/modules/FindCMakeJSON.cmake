include(FetchContent)

FetchContent_Declare(
  CMakeJSON  
  GIT_REPOSITORY https://github.com/Neumann-A/CMakeJSON.git
  GIT_TAG        47737f2f87b5071dcad7b28248583aa2453e0dd5
)

FetchContent_GetProperties(CMakeJSON)
string(TOLOWER "CMakeJSON" lcName)

if(NOT ${lcName}_POPULATED)
    FetchContent_Populate(CMakeJSON)
    set(CMakeJSON_INCLUDE_FILE "${${lcName}_SOURCE_DIR}/CMakeJSON/CMakeJSON.cmake")
    set(CMakeJSON_FOUND)
    include("${CMakeJSON_INCLUDE_FILE}")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CMakeJSON FOUND_VAR CMakeJSON_FOUND REQUIRED_VARS CMakeJSON_INCLUDE_FILE)