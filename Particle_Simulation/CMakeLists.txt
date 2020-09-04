include(GenerateExportHeader)
cmake_minimum_required (VERSION 3.17)

add_library(${PROJECT_NAME}_CommonTarget INTERFACE)
target_compile_features(${PROJECT_NAME}_CommonTarget INTERFACE cxx_std_17)
target_link_libraries(${PROJECT_NAME}_CommonTarget INTERFACE Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME}_CommonTarget INTERFACE MyCEL::MyCEL SerAr::Core)
target_include_directories(${PROJECT_NAME}_CommonTarget INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}")

add_library(InstructionsSets STATIC "arch/InstructionSets.cpp" "arch/InstructionSets.hpp")
target_compile_features(InstructionsSets PUBLIC cxx_std_17)

add_library(SimulationCommon OBJECT)
target_link_libraries(SimulationCommon PUBLIC ${PROJECT_NAME}_CommonTarget)

GENERATE_EXPORT_HEADER(SimulationCommon
          BASE_NAME StoPhySim
)

add_executable(${PROJECT_NAME})

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

target_sources(${PROJECT_NAME} PRIVATE ${CPP_FILES} ${H_FILES} ${INL_FILES} ${CPP_FILES_Simulator} ${CPP_FILES_General})
target_sources(SimulationCommon PRIVATE ${CPP_FILES} ${H_FILES} ${INL_FILES} "${CMAKE_CURRENT_BINARY_DIR}/stophysim_export.h")
target_include_directories(SimulationCommon PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_include_directories(${PROJECT_NAME} PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_link_libraries(${PROJECT_NAME} PRIVATE MyCEL::MyCEL SerAr::AllArchives ${PROJECT_NAME}_CommonTarget Threads::Threads)

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)
target_compile_definitions(${PROJECT_NAME} PRIVATE _SILENCE_CXX17_RESULT_OF_DEPRECATION_WARNING STOPHYSIM_EXPORT STOPHYSIM_STATIC_DEFINE )
target_compile_options(${PROJECT_NAME} PRIVATE ${ARCH_FLAG})

set(StoPhys_ARCH_LIST "AVX;AVX2;AVX512")
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
    target_link_libraries(StoPhysSim_${ARCH} PRIVATE MyCEL::MyCEL SerAr::AllArchives InstructionsSets Threads::Threads ${PROJECT_NAME}_CommonTarget)
    target_include_directories(StoPhysSim_${ARCH} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}")
    if(_pref_dep) # Add artificial dependencies to not run out of memory in parallel builds
        add_dependencies(StoPhysSim_${ARCH} ${_pref_dep})
        add_dependencies(${PROJECT_NAME} ${_pref_dep})
    endif()

    set(_pref_dep StoPhysSim_${ARCH})
    list(APPEND ARCH_DEFS WITH_${ARCH})
    list(APPEND ARCH_DLL_TARGETS StoPhysSim_${ARCH})
endforeach()

# if(Simulation_Boost_Random)
#     target_link_libraries(${PROJECT_NAME} PUBLIC Boost::random Boost::program_options)
#     target_link_libraries(SimulationCommon PUBLIC Boost::random)
#     target_compile_definitions(SimulationCommon PUBLIC -DUSE_BOOST_RANDOM)
#     foreach(_target IN LISTS StoPhys_ARCH_TARGETS)
#          target_link_libraries(${_target} PUBLIC Boost::random)
#     endforeach()
# endif()

# if(Simulation_PCG)
#     find_package(pcg-cpp)
#     target_link_libraries(${PROJECT_NAME} PUBLIC pcg-cpp::pcg)
#     target_link_libraries(SimulationCommon PUBLIC pcg-cpp::pcg)
#     target_compile_definitions(SimulationCommon PUBLIC -DUSE_PCG_RANDOM)
#     foreach(_target IN LISTS StoPhys_ARCH_TARGETS)
#          target_link_libraries(${_target} PUBLIC pcg-cpp::pcg)
#     endforeach()
# endif()

if(Simulation_GSL_Solvers)
    find_package(GSL REQUIRED)
    target_link_libraries(${PROJECT_NAME} PUBLIC GSL::gsl GSL::gslcblas)
    target_link_libraries(SimulationCommon PUBLIC GSL::gsl GSL::gslcblas)
    foreach(_target IN LISTS StoPhys_ARCH_TARGETS)
         target_link_libraries(${_target} PUBLIC GSL::gsl GSL::gslcblas)
    endforeach()
endif()

target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}")

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

set_target_properties(${PROJECT_NAME} PROPERTIES 
                      BUILD_WITH_INSTALL_RPATH FALSE
                      INSTALL_RPATH_USE_LINK_PATH TRUE
                      INSTALL_RPATH "./:./lib:${Matlab_ROOT_DIR}/bin/glnxa64")

                      set_target_properties(${PROJECT_NAME} PROPERTIES 
                      BUILD_WITH_INSTALL_RPATH FALSE
                      INSTALL_RPATH_USE_LINK_PATH TRUE
                      INSTALL_RPATH "./:./lib:${Matlab_ROOT_DIR}/bin/glnxa64")


add_executable(SimulationApplicationNew "app/SimulationApplicationNew.cpp" "app/InputParams.cpp" "app/InputParams.h" "app/StoPhys_arch.hpp" $<TARGET_OBJECTS:SimulationCommon>)
#target_sources(SimulationApplicationNew PRIVATE $<TARGET_OBJECTS:SimulationCommon>)
target_link_libraries(SimulationApplicationNew  
                     PRIVATE ${StoPhys_ARCH_TARGETS}
                     )
target_link_libraries(SimulationApplicationNew  
                        PRIVATE 
                                InstructionsSets
                                MyCEL::MyCEL 
                                SerAr::AllArchives
                                Boost::program_options
                                Eigen3::Eigen
                                Threads::Threads
                                ${PROJECT_NAME}_CommonTarget
                     )


target_include_directories(SimulationApplicationNew PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>)
target_include_directories(SimulationApplicationNew PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/app" "${CMAKE_CURRENT_BINARY_DIR}")
target_compile_definitions(SimulationApplicationNew PRIVATE _SILENCE_CXX17_RESULT_OF_DEPRECATION_WARNING ${ARCH_DEFS})

target_compile_options(SimulationApplicationNew PRIVATE -Wno-extra-semi-stmt -Wno-sign-conversion)
#target_compile_options(SimulationApplicationNew PRIVATE /arch:AVX)



#set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT Particle_Simulation)
cmake_print_variables(Matlab_ROOT_DIR CMAKE_CURRENT_BINARY_DIR)

set(GetRuntimeDep_CODE 
)

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
            PRE_EXCLUDE_REGEXES "(KERNEL32|win-core|^/usr/bin/)"
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
]])
add_custom_command(TARGET SimulationApplicationNew 
                   COMMAND 
                   ${CMAKE_COMMAND} 
                   "-DMatlab_ROOT_DIR=${Matlab_ROOT_DIR}" 
                   "-DTARGET_EXE=${CMAKE_CURRENT_BINARY_DIR}/SimulationApplicationNew${CMAKE_EXECUTABLE_SUFFX}"
                   -P "${CMAKE_CURRENT_BINARY_DIR}/getRuntimeDependencies.cmake" 
                   COMMENT "Catching drequired dependencies for startup."
                    )

# add_custom_target(GetRuntimeDependencies
#                    ${CMAKE_COMMAND} 
#                    "-DMatlab_ROOT_DIR=${Matlab_ROOT_DIR}" 
#                    "-DTARGET_EXE=${CMAKE_CURRENT_BINARY_DIR}/SimulationApplicationNew.exe"
#                    -P "${CMAKE_CURRENT_BINARY_DIR}/getRuntimeDependencies.cmake"
#                    COMMENT "Catching required dependencies for startup."
#                    DEPENDS SimulationApplicationNew
#                     )
install(CODE [[
    file(GET_RUNTIME_DEPENDENCIES
                # ...
                RESOLVED_DEPENDENCIES_VAR OUT_DEPS
                UNRESOLVED_DEPENDENCIES_VAR UNKNOWN_DEPS
                CONFLICTING_DEPENDENCIES_PREFIX CONFLICT_DEPS
                EXECUTABLES $<TARGET_FILE:SimulationApplicationNew>
                DIRECTORIES ${Matlab_ROOT_DIR}/extern/bin/win64
                            ${Matlab_ROOT_DIR}/bin/win64
                            ${Matlab_ROOT_DIR}/bin/glnxa64
                PRE_INCLUDE_REGEXES "(MATLAB)"
                PRE_EXCLUDE_REGEXES "(KERNEL32|win-core-app)"
                POST_EXCLUDE_REGEXES "(/|\\\\)[Ww][Ii][Nn][Dd][Oo][Ww][Ss](/|\\\\)"
        )
    file(COPY ${OUT_DEPS} DESTINATION ${CMAKE_INSTALL_BINDIR} FOLLOW_SYMLINK_CHAIN)
    ]]
)