cmake_minimum_required (VERSION 3.4)
project (Particle_Simulation)

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

target_include_directories(${PROJECT_NAME} PUBLIC ${Archive_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${Basic_Library_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${Particle_Simulation_SOURCE_DIR})

set(LINKER_OPT "")
if(UNIX)
#target_compile_definitions(${PROJECT_NAME} PRIVATE )
target_compile_options(${PROJECT_NAME} PRIVATE "-std=c++1z;-stdlib=libc++")
set(LINKER_OPT "-lc++;-lc++experimental")
endif()


target_link_libraries(${PROJECT_NAME} PUBLIC Basic_Library Archive ${LINKER_OPT})

find_package(Eigen3 REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

if(Simulation_Boost)
find_package(Boost REQUIRED COMPONENTS random)
target_link_libraries(${PROJECT_NAME} PUBLIC Boost::random)
endif()

if(Simulation_PCG)
find_package(pcg-cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC pcg-cpp::pcg)
endif()

if(Simulation_GSL_Solvers)
find_package(GSL REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC GSL::gsl GSL::gslcblas) 
target_link_libraries(${PROJECT_NAME} PUBLIC "$<$<CONFIG:DEBUG>:${GSL_LIBRARY_DEBUG}>")
target_link_libraries(${PROJECT_NAME} PUBLIC "$<$<CONFIG:DEBUG>:${GSL_CBLAS_LIBRARY_DEBUG}>")
endif()


install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_BINARY_DIR}/bin)

set_target_properties(${PROJECT_NAME} PROPERTIES 
					  BUILD_WITH_INSTALL_RPATH TRUE
                      INSTALL_RPATH_USE_LINK_PATH TRUE
                      INSTALL_RPATH "./lib:../lib")

#set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT Particle_Simulation)