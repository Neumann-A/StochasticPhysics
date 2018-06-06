project (Benchmark_NormalDistribution CXX)

set(CPP_FILES "Benchmark_NormalDistribution_Main.cpp")
set(INL_FILES "")
set(H_FILES "")

add_executable(${PROJECT_NAME} ${CPP_FILES} ${H_FILES} ${INL_FILES})

set_target_properties (${PROJECT_NAME} PROPERTIES FOLDER bench)	

find_package(benchmark REQUIRED)

target_include_directories(${PROJECT_NAME} PRIVATE ${benchmark_INCLUDE_DIR})

if(Simulation_Boost)
find_package(Boost REQUIRED COMPONENTS random)
 target_link_libraries(${PROJECT_NAME} PRIVATE Boost::random) 
endif()

if(Simulation_PCG)
find_package(pcg-cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE pcg-cpp::pcg)
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE ${LINKER_OPT})
target_link_libraries(${PROJECT_NAME} PRIVATE benchmark::benchmark)

if (MSVC AND NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
  if ( "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE /std:c++latest)
 # target_compile_definitions(${PROJECT_NAME} -fms-compatibility-version=19.00)
  else()
	target_compile_options(${PROJECT_NAME} PRIVATE /std:c++latest)
  endif()
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_BINARY_DIR}/bench)
