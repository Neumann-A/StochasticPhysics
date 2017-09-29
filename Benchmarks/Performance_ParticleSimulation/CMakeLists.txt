project (Performance_ParticleSimulation)

set(CPP_FILES "BM_ParticleSimulation_Main.cpp;BM_NeelProblem_F.cpp;BM_NoiseVector_F.cpp;stdafx.cpp")
set(INL_FILES "")
set(H_FILES "BM_NeelProblem_F.h;BM_NoiseVector_F.h;stdafx.h")

add_executable(${PROJECT_NAME} ${CPP_FILES} ${H_FILES} ${INL_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC ${Archive_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${Basic_Library_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${Particle_Simulation_SOURCE_DIR})

set_target_properties (${PROJECT_NAME} PROPERTIES FOLDER bench)	

find_package(benchmark REQUIRED)

if(Simulation_Boost)
find_package(Boost REQUIRED COMPONENTS random)
target_include_directories(${PROJECT_NAME} PRIVATE ${Boost_INCLUDE_DIRS})
endif()

if(Simulation_PCG)
find_path(PCG_INCLUDE NAMES "pcg_random.hpp" HINTS "${CMAKE_SOURCE_DIR}../Extern/pcg/include/" "${CMAKE_SOURCE_DIR}../../Extern/pcg/include/" "${CMAKE_SOURCE_DIR}../../../Extern/pcg/include/")
#message(STATUS "PCG path: " ${PCG_INCLUDE})
target_include_directories(${PROJECT_NAME} PRIVATE ${PCG_INCLUDE})
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE ${LINKER_OPT})
target_link_libraries(${PROJECT_NAME} PRIVATE benchmark::benchmark)
target_link_libraries(${PROJECT_NAME} PRIVATE Basic_Library Archive)

if(Simulation_Boost AND Boost_FOUND)
 target_link_libraries(${PROJECT_NAME} PRIVATE ${Boost_RANDOM_LIBRARY}) 
endif()

if (MSVC AND NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
  if ( "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE /std:c++latest)
 # target_compile_definitions(${PROJECT_NAME} -fms-compatibility-version=19.00)
  else()
	target_compile_options(${PROJECT_NAME} PRIVATE /std:c++latest)
  endif()
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_BINARY_DIR}/bench)
