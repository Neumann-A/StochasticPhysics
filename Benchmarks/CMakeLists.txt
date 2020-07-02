option (BUILD_BENCHMARKS "Build Benchmarks" OFF)
if(BUILD_BENCHMARKS)
    add_subdirectory (Benchmark_NormalDistribution)
    add_subdirectory (Performance_ParticleSimulation)
endif()


