project (Benchmark_NormalDistribution)


set(CPP_FILES "Benchmark_NormalDistribution_Main.cpp")
set(INL_FILES "")
set(H_FILES "")

add_executable(${PROJECT_NAME} ${CPP_FILES} ${H_FILES} ${INL_FILES})

find_package(benchmark REQUIRED)

target_include_directories(${PROJECT_NAME} PRIVATE ${benchmark_INCLUDE_DIR})

if(Simulation_Boost)
find_package(Boost REQUIRED COMPONENTS random)
target_include_directories(${PROJECT_NAME} PRIVATE ${Boost_INCLUDE_DIRS})
endif()

if(Simulation_PCG)
find_path(PCG_INCLUDE NAMES "pcg_random.hpp" HINTS "${CMAKE_SOURCE_DIR}../Extern/pcg/include/" "${CMAKE_SOURCE_DIR}../../Extern/pcg/include/" "${CMAKE_SOURCE_DIR}../../../Extern/pcg/include/")
message(STATUS "PCG path: " ${PCG_INCLUDE})
target_include_directories(${PROJECT_NAME} PRIVATE ${PCG_INCLUDE})
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE ${LINKER_OPT})
target_link_libraries(${PROJECT_NAME} PRIVATE ${benchmark_LIBRARY})

if(Simulation_Boost AND Boost_FOUND)
 target_link_libraries(${PROJECT_NAME} PRIVATE ${Boost_RANDOM_LIBRARY}) 
endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE "/MT")

if (MSVC AND NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
  if ( "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
 # target_compile_definitions(${PROJECT_NAME} -fms-compatibility-version=19.00)
  else()
	target_compile_options(${PROJECT_NAME} PRIVATE /std=c++latest)
  endif()
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_BINARY_DIR}/bench)
