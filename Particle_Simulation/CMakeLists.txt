include(GenerateExportHeader)
cmake_minimum_required (VERSION 3.17)

add_library(InstructionsSets STATIC "arch/InstructionSets.cpp" "arch/InstructionSets.hpp")
target_compile_features(InstructionsSets PUBLIC cxx_std_17)
target_compile_options(InstructionsSets PUBLIC /std:c++17)

add_library(SimulationCommon OBJECT)
target_link_libraries(SimulationCommon PUBLIC MyCEL::MyCEL SerAr::Core)
target_include_directories(SimulationCommon PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}")

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

target_include_directories(${PROJECT_NAME} PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_link_libraries(${PROJECT_NAME} PRIVATE MyCEL::MyCEL SerAr::AllArchives)

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
# AVX: -target-cpu sandybridge
# AVX2 -target-cpu haswell
# AVX512 -target-cpu skylake-avx512

foreach(ARCH IN LISTS StoPhys_ARCH_LIST)
    configure_file("app/StoPhys_arch.cpp.in" "app/StoPhysSim_${ARCH}.cpp")
    add_library(StoPhysSim_${ARCH} SHARED "app/StoPhysSim_${ARCH}.cpp" $<TARGET_OBJECTS:SimulationCommon>)
    if(NOT ARCH MATCHES "NONE")
        target_compile_options(StoPhysSim_${ARCH} PRIVATE /arch:${ARCH})
    else()
        target_compile_options(StoPhysSim_${ARCH} PRIVATE /arch:SSE2)
    endif()
    target_compile_definitions(StoPhysSim_${ARCH} PRIVATE SimulationCommon_EXPORTS)
    target_link_libraries(StoPhysSim_${ARCH} PRIVATE MyCEL::MyCEL SerAr::AllArchives InstructionsSets )
    target_include_directories(StoPhysSim_${ARCH} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}")
    if(_pref_dep) # Add artificial dependencies to not run out of memory in parallel builds
        add_dependencies(StoPhysSim_${ARCH} ${_pref_dep})
        add_dependencies(${PROJECT_NAME} ${_pref_dep})
    endif()
    set(_pref_dep StoPhysSim_${ARCH})
    list(APPEND ARCH_DEFS WITH_${ARCH})
endforeach()

if(Simulation_Boost_Random)
    target_link_libraries(${PROJECT_NAME} PUBLIC Boost::random Boost::program_options)
    target_link_libraries(SimulationCommon PUBLIC Boost::random)
    foreach(_target IN LISTS StoPhys_ARCH_TARGETS)
         target_link_libraries(${_target} PUBLIC Boost::random)
    endforeach()
endif()

if(Simulation_PCG)
    find_package(pcg-cpp)
    target_link_libraries(${PROJECT_NAME} PUBLIC pcg-cpp::pcg)
    target_link_libraries(SimulationCommon PUBLIC pcg-cpp::pcg)
    foreach(_target IN LISTS StoPhys_ARCH_TARGETS)
         target_link_libraries(${_target} PUBLIC pcg-cpp::pcg)
    endforeach()
endif()

if(Simulation_GSL_Solvers)
    find_package(GSL REQUIRED)
    target_link_libraries(${PROJECT_NAME} PUBLIC GSL::gsl GSL::gslcblas)
    target_link_libraries(SimulationCommon PUBLIC GSL::gsl GSL::gslcblas)
    foreach(_target IN LISTS StoPhys_ARCH_TARGETS)
         target_link_libraries(${_target} PUBLIC GSL::gsl GSL::gslcblas)
    endforeach()
endif()

target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}")

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_BINARY_DIR}/bin)

set_target_properties(${PROJECT_NAME} PROPERTIES 
                      BUILD_WITH_INSTALL_RPATH TRUE
                      INSTALL_RPATH_USE_LINK_PATH TRUE
                      INSTALL_RPATH "./lib:../lib")



add_executable(SimulationApplicationNew "app/SimulationApplicationNew.cpp" "app/InputParams.cpp" "app/InputParams.h" "app/StoPhys_arch.hpp" $<TARGET_OBJECTS:SimulationCommon>)
#target_sources(SimulationApplicationNew PRIVATE $<TARGET_OBJECTS:SimulationCommon>)
target_link_libraries(SimulationApplicationNew  
                        PRIVATE 
                                InstructionsSets
                                #SimulationCommon
                                MyCEL::MyCEL 
                                SerAr::AllArchives
                                Boost::program_options
                                Eigen3::Eigen
                                ${StoPhys_ARCH_TARGETS}
                     )
target_include_directories(SimulationApplicationNew PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>)
target_include_directories(SimulationApplicationNew PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/app" "${CMAKE_CURRENT_BINARY_DIR}")
target_compile_definitions(SimulationApplicationNew PRIVATE _SILENCE_CXX17_RESULT_OF_DEPRECATION_WARNING ${ARCH_DEFS})

target_compile_options(SimulationApplicationNew PRIVATE -Wno-extra-semi-stmt -Wno-sign-conversion)
#target_compile_options(SimulationApplicationNew PRIVATE /arch:AVX)



#set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT Particle_Simulation)