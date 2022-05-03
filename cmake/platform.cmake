cmake_path(CONVERT "a;a" TO_NATIVE_PATH_LIST PATHSEP)
string(REPLACE "a" "" PATHSEP "${PATHSEP}")

if(UNIX)
    set(CMAKE_CXX_VISIBILITY_PRESET hidden)
    set(CMAKE_C_VISIBILITY_PRESET hidden)
    # from https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/RPATH-handling
    set(CMAKE_SKIP_BUILD_RPATH FALSE)
    #set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    if(UNIX AND NOT APPLE)
        set(CMAKE_INSTALL_RPATH "$ORIGIN;$ORIGIN/lib;$ORIGIN/../lib")
    endif()

    # Matlab search
    if(NOT "$ENV{PATH}" MATCHES "/usr/local/MATLAB" AND NOT Matlab_ROOT_DIR)
        set(matlab_versions R2023b R2023a R2022b R2022a R2021a R2021b)
        set(matlab_on_path FALSE)
        foreach(matlab_version IN LISTS matlab_versions)
            if(EXISTS "/usr/local/MATLAB/${matlab_version}/bin/")
                set(ENV{PATH} "$ENV{PATH}${PATHSEP}/usr/local/MATLAB/${matlab_version}/bin/")
                set(matlab_on_path TRUE)
                break()
            endif()
        endforeach()
        if(NOT matlab_on_path)
            message(WARNING "Matlab not on PATH and Matlab_ROOT_DIR not set. CMake will probably not be able to find Matlab!")
        endif()
    else()
        message(STATUS "Matlab found on PATH")
    endif()
endif()