
set(TARGET_NAME Performance_ParticleSimulation)

set(CPP_FILES "BM_NeelProblem_F.cpp;BM_NoiseVector_F.cpp;stdafx.cpp")
set(INL_FILES "")
set(H_FILES "BM_NeelProblem_F.h;BM_NoiseVector_F.h;stdafx.h")

add_executable(${TARGET_NAME} ${CPP_FILES} ${H_FILES} ${INL_FILES} $<TARGET_OBJECTS:SimulationCommon>)

set_target_properties (${TARGET_NAME} PROPERTIES FOLDER bench)	

find_package(benchmark REQUIRED)
target_link_libraries(${TARGET_NAME} PRIVATE benchmark::benchmark benchmark::benchmark_main)
target_link_libraries(${TARGET_NAME} PRIVATE ${PROJECT_NAME}_CommonTarget)

install(TARGETS ${TARGET_NAME} DESTINATION ${CMAKE_BINARY_DIR}/bench)
