option (BUILD_TESTING "Build Tests" ON)
if(BUILD_TESTING)
    message(STATUS "Tests are enabled!")
    enable_testing()

    #--Setting up the Test Harness
    find_package(GTest CONFIG REQUIRED)
    add_library(TestWrapper INTERFACE)
    #target_sources(TestObjLib PRIVATE gtest_main_body.cpp)
    target_link_libraries(TestWrapper INTERFACE GTest::gtest GTest::gtest_main)
    target_compile_features(TestWrapper INTERFACE cxx_std_17)

    #--Adding Coverage Flag
    if(General_Enable_Test_Coverage)
        if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" AND MSVC)
        # using Clang-CL
        target_compile_options(TestWrapper BEFORE INTERFACE --coverage)
        #Required see https://bugs.llvm.org/show_bug.cgi?id=40877
        target_link_libraries(TestWrapper INTERFACE "\$(LLVMInstallDir)\\lib\\clang\\${CMAKE_CXX_COMPILER_VERSION}\\lib\\windows\\clang_rt.profile-x86_64.lib")
        message(STATUS "COMPILERPATH:${CMAKE_CXX_COMPILER}" )
        message(STATUS "CMAKE_<LANG>_COMPILER_VERSION:${CMAKE_CXX_COMPILER_VERSION}" )
        message(STATUS "COMPILERID:${CMAKE_CXX_COMPILER_ID}" )
        elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" AND NOT MSVC)
        # using clang
        elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        # using gcc
        target_compile_options(TestWrapper INTERFACE --coverage)
        elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
        # using Intel C++
        elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
        # using Visual Studio C++
        else ()
        # Unknown Compiler
        endif()
    endif()

    #set_target_properties(TestObjLib PROPERTIES FOLDER tests)

    #--Creating all the Test Projects
    add_subdirectory (Random_State_Init_Test)
    # add_subdirectory (Anisotropy_Test) #TODO: Currently copy of Random_State_Init_Test
    add_subdirectory (BrownAndNeel_Relaxation_Test)
    add_subdirectory (Implicit_Solver_Test)
    add_subdirectory (Neel_Problem_Test)
    add_subdirectory (Neel_Spherical_Test)
    add_subdirectory (NeelSpherical_BrownEuler_Relaxation_Test)
endif()