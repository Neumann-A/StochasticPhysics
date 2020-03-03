################################
OPTION(Simulation_Boost_Random "Enable usage of boost random" ON)
if(Simulation_Boost_Random)
    find_package(Boost REQUIRED random)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DUSE_BOOST_RANDOM)
    target_include_directories(${PROJECT_NAME} PRIVATE SYSTEM Boost::headers)
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
OPTION(Simulation_GSL_Solvers "Enable usage of GSL Solvers for implicit methods" OFF)
if(Simulation_GSL_Solvers)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DUSE_GSL_SOLVERS)
endif()