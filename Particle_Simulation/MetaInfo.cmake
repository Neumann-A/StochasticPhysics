
find_package(Git QUIET REQUIRED)

set(GIT_WORKING_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

string(TIMESTAMP PROJECT_DATE "%Y-%m-%d" UTC)
string(TIMESTAMP PROJECT_TIME "%H:%M:%S" UTC)

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

configure_file("meta/metainfo.in.cpp" "${TARGET_DIR}/meta/metainfo.cpp" @ONLY)
