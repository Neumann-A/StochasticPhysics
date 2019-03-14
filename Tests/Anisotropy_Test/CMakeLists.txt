project Anisotropy_Test CXX)


set(CPP_FILES "Anisotropy_Test.cpp")
set(INL_FILES "")
set(H_FILES "Anisotropy_Test.cpp")

add_executable(${PROJECT_NAME} ${CPP_FILES} ${H_FILES} ${INL_FILES})

set_target_properties (${PROJECT_NAME} PROPERTIES FOLDER tests)	

target_link_libraries(${PROJECT_NAME} PRIVATE TestWrapper)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

target_link_libraries(${PROJECT_NAME} PRIVATE Basic_Library)
target_link_libraries(${PROJECT_NAME} PRIVATE Archive)
target_include_directories(${PROJECT_NAME} PRIVATE ${Particle_Simulation_SOURCE_DIR})


add_test(${PROJECT_NAME} ${PROJECT_NAME})
