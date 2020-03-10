################################
OPTION(Simulation_Boost_Random "Enable usage of boost random" ON)
if(Simulation_Boost_Random)
    find_package(Boost REQUIRED COMPONENTS random)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DUSE_BOOST_RANDOM)
    target_link_libraries(${PROJECT_NAME} PRIVATE Boost::headers)
endif()
################################
OPTION(Simulation_PCG "Enable usage of PCG random" ON)
if(Simulation_PCG)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DUSE_PCG_RANDOM)
endif()
################################
OPTION(Simulation_SIMD_NORMAL_DIST "Enable usage of SIMD PCG random" OFF)
if(Simulation_SIMD_NORMAL_DIST)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DSIMD_NORMAL_DIST)
endif()
################################
OPTION(Simulation_WITH_GSL_Solvers "Enable usage of GSL Solvers for implicit methods" OFF)
if(Simulation_WITH_GSL_Solvers)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DWITH_GSL_SOLVERS)
endif()

OPTION(Simulation_WITH_ImplicitMidpoint "Enable usage of GSL Solvers for implicit methods" OFF)
if(Simulation_WITH_ImplicitMidpoint)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DWITH_IMPLICIT_MIDPOINT)
endif()