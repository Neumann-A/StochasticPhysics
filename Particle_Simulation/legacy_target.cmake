add_executable(${PROJECT_NAME} ${EXCLUDE})
target_sources(${PROJECT_NAME} PRIVATE ${CPP_FILES} ${H_FILES} ${INL_FILES} ${CPP_FILES_Simulator} ${CPP_FILES_General} ${H_FILES_General})

target_include_directories(${PROJECT_NAME} PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
target_link_libraries(${PROJECT_NAME} PRIVATE MyCEL::MyCEL ${PROJECT_NAME}_CommonTarget Threads::Threads SimulationArchive)

target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)
target_compile_definitions(${PROJECT_NAME} PRIVATE _HAS_DEPRECATED_RESULT_OF _SILENCE_CXX17_RESULT_OF_DEPRECATION_WARNING STOPHYSIM_STATIC_DEFINE )
target_compile_options(${PROJECT_NAME} PRIVATE ${ARCH_FLAG})

if(Simulation_Boost_Random)
    find_package(Boost REQUIRED COMPONENTS random)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DUSE_BOOST_RANDOM)
    target_link_libraries(${PROJECT_NAME} PRIVATE Boost::headers)
endif()

if(Simulation_PCG)
    find_package(pcg-cpp)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DUSE_PCG_RANDOM)
endif()
if(Simulation_SIMD_NORMAL_DIST)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DSIMD_NORMAL_DIST)
endif()
if(Simulation_WITH_GSL_Solvers)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DWITH_GSL_SOLVERS)
    target_link_libraries(${PROJECT_NAME} PUBLIC GSL::gsl GSL::gslcblas)
endif()
if(Simulation_WITH_ImplicitMidpoint)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DWITH_IMPLICIT_MIDPOINT)
endif()

target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}")
if(StoPhys_LIMITED_RESOURCE)
    set_property(TARGET ${PROJECT_NAME} PROPERTY JOB_POOL_LINK one_job)
    set_property(TARGET ${PROJECT_NAME} PROPERTY JOB_POOL_COMPILE one_job)
endif()

install(TARGETS "${PROJECT_NAME}" DESTINATION "${CMAKE_INSTALL_BINDIR}")

set_target_properties(${PROJECT_NAME} PROPERTIES 
                      BUILD_WITH_INSTALL_RPATH FALSE
                      INSTALL_RPATH_USE_LINK_PATH TRUE
                      INSTALL_RPATH "./:./lib:${Matlab_ROOT_DIR}/bin/glnxa64")

                      set_target_properties(${PROJECT_NAME} PROPERTIES 
                      BUILD_WITH_INSTALL_RPATH FALSE
                      INSTALL_RPATH_USE_LINK_PATH TRUE
                      INSTALL_RPATH "./:./lib:${Matlab_ROOT_DIR}/bin/glnxa64")
