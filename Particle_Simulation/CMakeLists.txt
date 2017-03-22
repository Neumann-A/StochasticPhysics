cmake_minimum_required (VERSION 3.4)
project (Particle_Simulation)

include(FindEigen3)
include(UseEigen3)

find_path(PCG_INCLUDE NAMES "pcg_random.hpp" HINTS "${CMAKE_SOURCE_DIR}../Extern/pcg/include/" "${CMAKE_SOURCE_DIR}../../Extern/pcg/include/" "${CMAKE_SOURCE_DIR}../../../Extern/pcg/include/")
#message(STATUS "Eigen path: " ${EIGEN3_INCLUDE_DIRS})
message(STATUS "PCG path: " ${PCG_INCLUDE})



include_directories(${PCG_INCLUDE})
include_directories(${Archive_INCLUDE_DIRS})
include_directories(${Basic_Library_INCLUDE_DIRS})
include_directories(${Particle_Simulation_SOURCE_DIR})

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

add_executable(${PROJECT_NAME} ${CPP_FILES} ${H_FILES} ${INL_FILES})

set(LINKER_OPT "")
if(UNIX)
#target_compile_definitions(${PROJECT_NAME} PRIVATE )
target_compile_options(${PROJECT_NAME} PRIVATE "-std=c++1z;-stdlib=libc++")
set(LINKER_OPT "-lc++;-lc++experimental")
endif()

target_link_libraries(${PROJECT_NAME} Basic_Library Archive ${LINKER_OPT})


install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_BINARY_DIR}/bin)

#set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT Particle_Simulation)