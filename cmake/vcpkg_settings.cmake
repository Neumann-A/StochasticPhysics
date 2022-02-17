## Map vcpkg manifest features to project options
if(VCPKG_MANIFEST_FEATURES)
    set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
    if(VCPKG_MANIFEST_FEATURES MATCHES "tests")
        set(BUILD_TESTING ON CACHE BOOL "" FORCE)
    endif()
    set(BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
    if(VCPKG_MANIFEST_FEATURES MATCHES "benchmarks")
        set(BUILD_BENCHMARKS ON CACHE BOOL "" FORCE)
    endif()
    set(SPhys_WITH_MATLAB OFF CACHE BOOL "" FORCE)
    if(VCPKG_MANIFEST_FEATURES MATCHES "matlab")
        set(SPhys_WITH_MATLAB ON CACHE BOOL "" FORCE)
    endif()
    set(SPhys_WITH_HDF5 OFF CACHE BOOL "" FORCE)
    if(VCPKG_MANIFEST_FEATURES MATCHES "hdf5")
        set(SPhys_WITH_HDF5 ON CACHE BOOL "" FORCE)
    endif()
endif()

set(VCPKG_MANIFEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg_manifest")

## Port overlays
include(FetchContent)
FetchContent_Declare(
    my-vcpkg-ports
    GIT_REPOSITORY https://github.com/Neumann-A/my-vcpkg-ports.git
    GIT_TAG        a93a5fb7c288a0b82ef666da0fa1c962b43360b8
)
FetchContent_GetProperties(my-vcpkg-ports)
if(NOT my-vcpkg-ports_POPULATED)
    FetchContent_Populate(my-vcpkg-ports)
endif()
list(APPEND VCPKG_OVERLAY_PORTS "${my-vcpkg-ports_SOURCE_DIR}")

## Overlay triplets
include(FetchContent)
FetchContent_Declare(
    my-vcpkg-triplets
    GIT_REPOSITORY https://github.com/Neumann-A/my-vcpkg-triplets.git
    GIT_TAG        a93a5fb7c288a0b82ef666da0fa1c962b43360b8
)
FetchContent_GetProperties(my-vcpkg-triplets)
if(NOT my-vcpkg-triplets_POPULATED)
    FetchContent_Populate(my-vcpkg-triplets)
endif()
list(APPEND VCPKG_OVERLAY_TRIPLETS "${my-vcpkg-triplets_SOURCE_DIR}")