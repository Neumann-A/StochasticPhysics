include(FetchContent)
FetchContent_Declare(
    CMakeModules
    GIT_REPOSITORY https://github.com/Neumann-A/CMakeModules
    GIT_TAG        aed279bbb3de77992b407a49ce6bd79ce189714c
)
FetchContent_GetProperties(CMakeModules)
if(NOT CMakeModules_POPULATED)
    FetchContent_Populate(CMakeModules_POPULATED)
endif()
list(APPEND CMAKE_MODULE_PATH "${CMakeModules_SOURCE_DIR}")
set(Boost_NO_WARN_NEW_VERSIONS ON)