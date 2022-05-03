include(FetchContent)
FetchContent_Declare(
    my-cmake-toolchains
    GIT_REPOSITORY https://github.com/Neumann-A/my-cmake-toolchains.git
    GIT_TAG        6c628f560c2213701a02786e5153ce474d933061
)
FetchContent_GetProperties(my-cmake-toolchains)
if(NOT my-cmake-toolchains_POPULATED)
    FetchContent_Populate(my-cmake-toolchains)
endif()
include("${my-cmake-toolchains_SOURCE_DIR}/select_and_load_toolchain.cmake")