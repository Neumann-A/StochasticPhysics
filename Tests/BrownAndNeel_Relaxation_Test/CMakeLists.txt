project (BrownAndNeel_Relaxation_Test CXX)


set(CPP_FILES "BrownAndNeel_Relaxation_Test_main.cpp;Test_BrownAndNeel_Relaxation.cpp")
set(INL_FILES "")
set(H_FILES "Test_BrownAndNeel_Relaxation.h;")

add_executable(${PROJECT_NAME} ${CPP_FILES} ${H_FILES} ${INL_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC ${Archive_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${Basic_Library_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${Particle_Simulation_SOURCE_DIR})

set_target_properties (${PROJECT_NAME} PROPERTIES FOLDER tests)	

target_link_libraries(${PROJECT_NAME} PRIVATE GTest::GTest GTest::Main)
target_link_libraries(${PROJECT_NAME} PRIVATE Archive)

target_link_libraries(${PROJECT_NAME} PUBLIC --coverage)
add_test(${PROJECT_NAME} ${PROJECT_NAME})
