cmake_minimum_required (VERSION 3.9)
project (Benchmark_NormalDistribution CXX)

set(CPP_FILES "Benchmark_NormalDistribution_Main.cpp")
set(INL_FILES "")
set(H_FILES "")

add_executable(${PROJECT_NAME} ${CPP_FILES} ${H_FILES} ${INL_FILES})

set_target_properties (${PROJECT_NAME} PROPERTIES FOLDER bench)	

find_package(benchmark REQUIRED)

#target_include_directories(${PROJECT_NAME} PRIVATE ${benchmark_INCLUDE_DIR})

if(Simulation_Boost)
find_package(Boost REQUIRED COMPONENTS random)
 target_link_libraries(${PROJECT_NAME} PRIVATE Boost::random) 
endif()

if(Simulation_PCG)
 find_package(pcg-cpp REQUIRED)
 target_link_libraries(${PROJECT_NAME} PRIVATE pcg-cpp::pcg)
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE ${LINKER_OPT})
target_link_libraries(${PROJECT_NAME} PRIVATE benchmark::benchmark benchmark::benchmark_main)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_BINARY_DIR}/bench)
