
find_package(Git QUIET REQUIRED)

set(GIT_WORKING_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

string(TIMESTAMP PROJECT_DATE "%Y-%m-%d" UTC)
string(TIMESTAMP PROJECT_TIME "%H:%M:%S" UTC)

if(Git_FOUND)
execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse --abbrev-ref HEAD
                WORKING_DIRECTORY "${GIT_WORKING_DIR}"
                OUTPUT_VARIABLE PROJECT_GIT_BRANCH
                COMMAND_ERROR_IS_FATAL ANY
                OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse HEAD
                WORKING_DIRECTORY "${GIT_WORKING_DIR}"
                OUTPUT_VARIABLE PROJECT_GIT_SHA
                COMMAND_ERROR_IS_FATAL ANY
                OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND "${GIT_EXECUTABLE}" describe --always --dirty --tags
                WORKING_DIRECTORY "${GIT_WORKING_DIR}"
                OUTPUT_VARIABLE PROJECT_GIT_DESCRIBE
                COMMAND_ERROR_IS_FATAL ANY
                OUTPUT_STRIP_TRAILING_WHITESPACE)
else()
    set(PROJECT_GIT_DESCRIBE "Without-Git" CACHE STRING "")
    if(NOT PROJECT_GIT_SHA)
        message(FATAL_ERROR "Please manually set PROJECT_GIT_SHA")
    endif()
    if(NOT PROJECT_GIT_BRANCH)
        message(FATAL_ERROR "Please manually set PROJECT_GIT_BRANCH")
    endif()
endif()
configure_file("meta/metainfo.in.cpp" "${TARGET_DIR}/meta/metainfo.cpp" @ONLY)
