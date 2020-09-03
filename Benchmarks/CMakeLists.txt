option (BUILD_BENCHMARKS "Build Benchmarks" ON)
if(BUILD_BENCHMARKS)
    message(STATUS "Benchmarks are enabled!")
    add_subdirectory (Benchmark_NormalDistribution)
    add_subdirectory (Performance_ParticleSimulation)
endif()


