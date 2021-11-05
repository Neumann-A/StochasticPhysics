include(GenerateExportHeader)
cmake_minimum_required (VERSION 3.19)

add_custom_target(Project-Build-Info
                    COMMAND "${CMAKE_COMMAND}" "-DTARGET_DIR=${CMAKE_CURRENT_BINARY_DIR}" "-P" "${CMAKE_CURRENT_SOURCE_DIR}/MetaInfo.cmake" 
                    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                    BYPRODUCTS "${CMAKE_CURRENT_BINARY_DIR}/meta/metainfo.cpp"
                    SOURCES "meta/metainfo.cpp.in"
                    COMMENT "Generating build meta info")

add_library(SimulationArchive STATIC "archive/archive_preprocessing.cpp" "archive/archive_preprocessing.hpp")
target_compile_features(SimulationArchive PUBLIC cxx_std_20)
target_link_libraries(SimulationArchive PUBLIC SerAr::AllArchives Eigen3::Eigen)
target_include_directories(SimulationArchive PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

add_library(${PROJECT_NAME}_CommonTarget INTERFACE)
target_compile_definitions(${PROJECT_NAME}_CommonTarget INTERFACE _CRT_SECURE_NO_WARNINGS)
target_compile_features(${PROJECT_NAME}_CommonTarget INTERFACE cxx_std_20)
target_link_libraries(${PROJECT_NAME}_CommonTarget INTERFACE Eigen3::Eigen SimulationArchive)
target_link_libraries(${PROJECT_NAME}_CommonTarget INTERFACE MyCEL::MyCEL)
target_include_directories(${PROJECT_NAME}_CommonTarget INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}")
if(MSVC)
    target_compile_definitions(${PROJECT_NAME}_CommonTarget INTERFACE _HAS_DEPRECATED_RESULT_OF _SILENCE_CXX17_RESULT_OF_DEPRECATION_WARNING)
endif()

add_library(InstructionsSets STATIC "arch/InstructionSets.cpp" "arch/InstructionSets.hpp")
target_compile_features(InstructionsSets PUBLIC cxx_std_20)

add_library(SimulationCommon OBJECT "meta/metainfo.cpp")
target_compile_features(SimulationCommon PUBLIC cxx_std_20)
target_link_libraries(SimulationCommon PUBLIC ${PROJECT_NAME}_CommonTarget SimulationArchive)

generate_export_header(SimulationCommon
          BASE_NAME StoPhySim
)
if(StoPhys_LIMITED_RESOURCE)
    set(EXCLUDE EXCLUDE_FROM_ALL)
endif()
add_executable(${PROJECT_NAME} ${EXCLUDE})

include(cmake/SimulationOptions.cmake)

set(CPP_FILES "")
set(INL_FILES "")
set(H_FILES "")

#Fields
include(Fields/Fields.cmake)
#General
include(General/General.cmake)
#Problems
include(Problems/Problems.cmake)
#Properties
include(Properties/Properties.cmake)
#Provider
include(Provider/Provider.cmake)
#Results
include(Results/Results.cmake)
#SDEFramework
include(SDEFramework/SDEFramework.cmake)
#Selectors
include(Selectors/Selectors.cmake)
#Settings
include(Settings/Settings.cmake)
#Simulator
include(Simulator/Simulator.cmake)

target_sources(${PROJECT_NAME} PRIVATE ${CPP_FILES} ${H_FILES} ${INL_FILES} ${CPP_FILES_Simulator} ${CPP_FILES_General} ${H_FILES_General})
target_sources(SimulationCommon PRIVATE ${CPP_FILES} ${H_FILES} ${INL_FILES} "${CMAKE_CURRENT_BINARY_DIR}/stophysim_export.h")
target_include_directories(SimulationCommon PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_include_directories(${PROJECT_NAME} PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
target_link_libraries(${PROJECT_NAME} PRIVATE MyCEL::MyCEL ${PROJECT_NAME}_CommonTarget Threads::Threads SimulationArchive)

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)
target_compile_definitions(${PROJECT_NAME} PRIVATE _HAS_DEPRECATED_RESULT_OF _SILENCE_CXX17_RESULT_OF_DEPRECATION_WARNING STOPHYSIM_STATIC_DEFINE )
target_compile_options(${PROJECT_NAME} PRIVATE ${ARCH_FLAG})

#set(StoPhys_ARCH_LIST "AVX;AVX2;AVX512")
set(StoPhys_ARCH_LIST "AVX2" CACHE STRING "")
set(StoPhys_ARCH_TARGETS)

set(ARCH_EXTERN)
foreach(ARCH IN LISTS StoPhys_ARCH_LIST)
    string(APPEND ARCH_EXTERN "    extern template class SimulationManager<PREC, MyCEL::SystemInfo::InstructionSet::${ARCH}>;\n")
    list(APPEND StoPhys_ARCH_TARGETS StoPhysSim_${ARCH})
endforeach()
configure_file("app/StoPhys_arch.hpp.in" "app/StoPhys_arch.hpp")
unset(ARCH_EXTERN)

set(_prev_dep)
set(ARCH_DEFS)
set(ARCH_DLL_TARGETS)
# AVX: -target-cpu sandybridge
# AVX2 -target-cpu haswell
# AVX512 -target-cpu skylake-avx512

set(AVX_FLAG -march=sandybridge)
set(AVX2_FLAG -march=haswell)
set(AVX512_FLAG -march=skylake-avx512)

foreach(ARCH IN LISTS StoPhys_ARCH_LIST)
    configure_file("app/StoPhys_arch.cpp.in" "app/StoPhysSim_${ARCH}.cpp")
    add_library(StoPhysSim_${ARCH} SHARED "app/StoPhysSim_${ARCH}.cpp" $<TARGET_OBJECTS:SimulationCommon>)
    if(StoPhys_LIMITED_RESOURCE)
        set_property(TARGET StoPhysSim_${ARCH} PROPERTY JOB_POOL_LINK one_job)
        set_property(TARGET StoPhysSim_${ARCH} PROPERTY JOB_POOL_COMPILE one_job)
    endif()
    if(NOT ARCH MATCHES "NONE")
        if(MSVC)
            target_compile_options(StoPhysSim_${ARCH} PRIVATE /arch:${ARCH})
        else()
            target_compile_options(StoPhysSim_${ARCH} PRIVATE ${${ARCH}_FLAG})
        endif()
    else()
        if(MSVC)
            target_compile_options(StoPhysSim_${ARCH} PRIVATE /arch:SSE2)
        else()
            target_compile_options(StoPhysSim_${ARCH} PRIVATE -msse4)
        endif()
    endif()
    target_compile_definitions(StoPhysSim_${ARCH} PRIVATE SimulationCommon_EXPORTS)
    target_link_libraries(StoPhysSim_${ARCH} PRIVATE MyCEL::MyCEL SimulationArchive InstructionsSets Threads::Threads ${PROJECT_NAME}_CommonTarget)
    target_include_directories(StoPhysSim_${ARCH} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}")
    #if(_pref_dep) # Add artificial dependencies to not run out of memory in parallel builds
        #add_dependencies(StoPhysSim_${ARCH} ${_pref_dep})
        #add_dependencies(${PROJECT_NAME} ${_pref_dep})
    #endif()

    set(_pref_dep StoPhysSim_${ARCH})
    list(APPEND ARCH_DEFS WITH_${ARCH})
    list(APPEND ARCH_DLL_TARGETS StoPhysSim_${ARCH})
endforeach()

if(Simulation_GSL_Solvers)
    find_package(GSL REQUIRED)
    target_link_libraries(${PROJECT_NAME} PUBLIC GSL::gsl GSL::gslcblas)
    target_link_libraries(SimulationCommon PUBLIC GSL::gsl GSL::gslcblas)
    foreach(_target IN LISTS StoPhys_ARCH_TARGETS)
         target_link_libraries(${_target} PUBLIC GSL::gsl GSL::gslcblas)
    endforeach()
endif()

target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}")
if(StoPhys_LIMITED_RESOURCE)
    set_property(TARGET ${PROJECT_NAME} PROPERTY JOB_POOL_LINK one_job)
    set_property(TARGET ${PROJECT_NAME} PROPERTY JOB_POOL_COMPILE one_job)
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

set_target_properties(${PROJECT_NAME} PROPERTIES 
                      BUILD_WITH_INSTALL_RPATH FALSE
                      INSTALL_RPATH_USE_LINK_PATH TRUE
                      INSTALL_RPATH "./:./lib:${Matlab_ROOT_DIR}/bin/glnxa64")

                      set_target_properties(${PROJECT_NAME} PROPERTIES 
                      BUILD_WITH_INSTALL_RPATH FALSE
                      INSTALL_RPATH_USE_LINK_PATH TRUE
                      INSTALL_RPATH "./:./lib:${Matlab_ROOT_DIR}/bin/glnxa64")

set(TARGET_NAME StoPhysApp_MultiArch)
add_executable(${TARGET_NAME} "app/StoPhysApp_MultiArch.cpp" "app/InputParams.cpp" "app/InputParams.h" "app/StoPhys_arch.hpp.in" $<TARGET_OBJECTS:SimulationCommon>)
if(StoPhys_LIMITED_RESOURCE)
    set_property(TARGET ${TARGET_NAME} PROPERTY JOB_POOL_LINK one_job)
    set_property(TARGET ${TARGET_NAME} PROPERTY JOB_POOL_COMPILE one_job)
endif()

target_link_libraries(${TARGET_NAME}  
                     PRIVATE ${StoPhys_ARCH_TARGETS}
                     )
target_link_libraries(${TARGET_NAME}  
                        PRIVATE 
                                InstructionsSets
                                MyCEL::MyCEL 
                                SimulationArchive
                                Boost::program_options
                                Eigen3::Eigen
                                Threads::Threads
                                ${PROJECT_NAME}_CommonTarget
                     )

target_include_directories(${TARGET_NAME} PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>)
target_include_directories(${TARGET_NAME} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/app" "${CMAKE_CURRENT_BINARY_DIR}")
target_compile_definitions(${TARGET_NAME} PRIVATE _SILENCE_CXX17_RESULT_OF_DEPRECATION_WARNING ${ARCH_DEFS})

target_compile_options(${TARGET_NAME} PRIVATE -Wno-extra-semi-stmt -Wno-sign-conversion)
if(CMAKE_GENERATOR MATCHES "Visual Studio" AND Matlab_ROOT_DIR)
    set(runtime_dirs "${Matlab_ROOT_DIR}/bin/win64")
    if(_VCPKG_INSTALLED_DIR)
        string(APPEND runtime_dirs ";${_VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}$<$<CONFIG:DEBUG>:/debug>/bin")
    endif()
    set_property(TARGET ${TARGET_NAME} APPEND_STRING PROPERTY VS_DEBUGGER_ENVIRONMENT "PATH=%PATH%;${runtime_dirs}\n")
endif()

#set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT Particle_Simulation)
cmake_print_variables(Matlab_FOUND Matlab_ROOT_DIR CMAKE_CURRENT_BINARY_DIR VCPKG_APPLOCAL_DEPS CMAKE_GENERATOR)

if(NOT CMAKE_GENERATOR MATCHES "Visual Studio")
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/getRuntimeDependencies.cmake
    [[
include(CMakePrintHelpers)
cmake_print_variables(TARGET_EXE Matlab_ROOT_DIR CMAKE_CURRENT_BINARY_DIR)
file(GET_RUNTIME_DEPENDENCIES
            # ...
            RESOLVED_DEPENDENCIES_VAR OUT_DEPS
            UNRESOLVED_DEPENDENCIES_VAR UNKNOWN_DEPS
            CONFLICTING_DEPENDENCIES_PREFIX CONFLICT_DEPS
            EXECUTABLES ${TARGET_EXE}
            DIRECTORIES ${Matlab_ROOT_DIR}/extern/bin/win64
                        ${Matlab_ROOT_DIR}/bin/win64
                        ${Matlab_ROOT_DIR}/bin/glnxa64
            PRE_INCLUDE_REGEXES "(MATLAB|.+/lib)"
            PRE_EXCLUDE_REGEXES "(KERNEL32|win-core|win-crt||vcruntime140|^/usr/bin/|libgcc(_s)?|libc[.](a|so)|libstdc[+][+]|libunwind)"
            POST_EXCLUDE_REGEXES "((/|\\\\)[Ww][Ii][Nn][Dd][Oo][Ww][Ss](/|\\\\)|^/lib|^/usr/bin/|^/usr/lib|^/usr/local/bin|^/usr/local/lib)"
    )
message(STATUS "OUT_DEPS:${OUT_DEPS}")
message(STATUS "UNKNOWN_DEPS:${UNKNOWN_DEPS}")
foreach(_dep IN LISTS OUT_DEPS)
    message(STATUS "KNOWN:${_dep}")
    file(COPY ${_dep} DESTINATION "${CMAKE_CURRENT_BINARY_DIR}" FOLLOW_SYMLINK_CHAIN)
endforeach()
foreach(_dep IN LISTS UNKNOWN_DEPS)
    message(STATUS "UNKNOWN:${_dep}")
endforeach()
foreach(_dep IN LISTS CONFLICT_DEPS)
    message(STATUS "CONFLICT:${_dep}")
endforeach()
file(COPY ${OUT_DEPS} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
]]  )
    add_custom_command(TARGET ${TARGET_NAME} 
                    COMMAND 
                    ${CMAKE_COMMAND} 
                    "-DMatlab_ROOT_DIR=${Matlab_ROOT_DIR}" 
                    "-DTARGET_EXE=$<TARGET_FILE:${TARGET_NAME}>"
                    -P "${CMAKE_CURRENT_BINARY_DIR}/getRuntimeDependencies.cmake" 
                    COMMENT "Catching required dependencies for startup."
                    )

    # add_custom_target(GetRuntimeDependencies
    #                 ${CMAKE_COMMAND} 
    #                 "-DMatlab_ROOT_DIR=${Matlab_ROOT_DIR}" 
    #                 "-DTARGET_EXE=${CMAKE_CURRENT_BINARY_DIR}/SimulationApplicationNew${CMAKE_EXECUTABLE_SUFFIX}"
    #                 -P "${CMAKE_CURRENT_BINARY_DIR}/getRuntimeDependencies.cmake"
    #                 COMMENT "Catching required dependencies for startup."
    #                 DEPENDS SimulationApplicationNew
    #                     )

endif()

install(CODE "
file(GET_RUNTIME_DEPENDENCIES
            # ...
            RESOLVED_DEPENDENCIES_VAR OUT_DEPS
            UNRESOLVED_DEPENDENCIES_VAR UNKNOWN_DEPS
            CONFLICTING_DEPENDENCIES_PREFIX CONFLICT_DEPS
            EXECUTABLES $<TARGET_FILE:${TARGET_NAME}>
            DIRECTORIES \${Matlab_ROOT_DIR}/extern/bin/win64
                        \${Matlab_ROOT_DIR}/bin/win64
                        \${Matlab_ROOT_DIR}/bin/glnxa64
            PRE_INCLUDE_REGEXES \"(MATLAB)\"
            PRE_EXCLUDE_REGEXES \"(KERNEL32|win-core-app)\"
            POST_EXCLUDE_REGEXES \"(/|\\\\\\\\)[Ww][Ii][Nn][Dd][Oo][Ww][Ss](/|\\\\\\\\)\"
    )
file(COPY \${OUT_DEPS} DESTINATION \${CMAKE_INSTALL_BINDIR} FOLLOW_SYMLINK_CHAIN)
"
)
