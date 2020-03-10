cmake_minimum_required (VERSION 3.16)
project (Particle_Simulation CXX)
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

target_sources(${PROJECT_NAME} PRIVATE ${CPP_FILES} ${H_FILES} ${INL_FILES})
target_include_directories(${PROJECT_NAME} PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_link_libraries(${PROJECT_NAME} PRIVATE Basic_Library Archive)

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)
target_compile_definitions(${PROJECT_NAME} PUBLIC _SILENCE_CXX17_RESULT_OF_DEPRECATION_WARNING)

if(Simulation_Boost_Random)
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