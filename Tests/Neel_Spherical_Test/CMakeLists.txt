project (Neel_Sphereical_Test)


set(CPP_FILES "Neel_sphereical_main_Test.cpp;Test_Neel_Spherical_Problem.cpp")
set(INL_FILES "")
set(H_FILES "Test_Neel_Spherical_Problem.h;")

add_executable(${PROJECT_NAME} ${CPP_FILES} ${H_FILES} ${INL_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC ${Archive_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${Basic_Library_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${Particle_Simulation_SOURCE_DIR})

set_target_properties (${PROJECT_NAME} PROPERTIES FOLDER tests)	

find_package(GTest REQUIRED)
target_link_libraries(${PROJECT_NAME} GTest::GTest GTest::Main)

target_link_libraries(${PROJECT_NAME} --coverage)
add_test(${PROJECT_NAME} ${PROJECT_NAME})
