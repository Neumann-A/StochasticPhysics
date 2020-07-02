include(CMakeFindDependencyMacro)
find_dependency(VCPKG) # Try to use VCPKG if available
cmake_print_variables(VCPKG_ROOT VCPKG_TARGET_TRIPLET CMAKE_TOOLCHAIN_FILE VCPKG_INSTALLED_DIR CMAKE_PREFIX_PATH)
if(VCPKG_INSTALLED_DIR AND NOT CMAKE_PREFIX_PATH)
    if(NOT CMakeCS_DIR)
        set(_FORCE FORCE)
    endif()
    set(CMakeCS_DIR "${VCPKG_INSTALLED_DIR}/share/CMakeCS/" CACHE PATH "Path to CMakeCS" ${_FORCE})
    set(GTest_DIR "${CMakeCS_DIR}/../gtest" CACHE PATH "Path to GTest" ${_FORCE}) 
    unset(_FORCE)
    # Typically called before the first project() call
    # as such the toolchain file is not yet loaded
endif()
cmake_print_variables(CMakeCS_DIR)
find_dependency(CMakeCS CONFIG)

find_package_handle_standard_args(CMakeCS CONFIG_MODE)