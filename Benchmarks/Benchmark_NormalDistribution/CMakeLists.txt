cmake_minimum_required (VERSION 3.17)

set(TARGET_NAME Benchmark_NormalDistribution)

set(CPP_FILES "Benchmark_NormalDistribution_Main.cpp")
set(INL_FILES "")
set(H_FILES "")

add_executable(${TARGET_NAME} ${CPP_FILES} ${H_FILES} ${INL_FILES})

set_target_properties (${TARGET_NAME} PROPERTIES FOLDER bench)	

find_package(benchmark REQUIRED)

if(Simulation_Boost)
find_package(Boost REQUIRED COMPONENTS random)
 target_link_libraries(${TARGET_NAME} PRIVATE Boost::random) 
endif()

if(Simulation_PCG)
 find_package(pcg-cpp REQUIRED)
 target_link_libraries(${TARGET_NAME} PRIVATE pcg-cpp::pcg)
endif()

target_link_libraries(${TARGET_NAME} PRIVATE benchmark::benchmark benchmark::benchmark_main)

target_compile_features(${TARGET_NAME} PRIVATE cxx_std_17)

install(TARGETS ${TARGET_NAME} DESTINATION ${CMAKE_BINARY_DIR}/bench)
