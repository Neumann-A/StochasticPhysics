project (Problem_Neel_Test CXX)



set(CPP_FILES "Test_Problem_Neel_Main.cpp;Test_ParamCalc_Neel.cpp;Test_Problem_Neel.cpp;Test_Uniaxial_Anisotropy.cpp")
set(INL_FILES "")
set(H_FILES "Test_ParamCalc_Neel.h;Test_Problem_Neel.h;Test_Uniaxial_Anisotropy.h")

add_executable(${PROJECT_NAME} ${CPP_FILES} ${H_FILES} ${INL_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC ${Archive_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${Basic_Library_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${Particle_Simulation_SOURCE_DIR})

set_target_properties (${PROJECT_NAME} PROPERTIES FOLDER tests)	

find_package(GTest REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE GTest::GTest GTest::Main)
find_package(Eigen3 REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} PRIVATE Basic_Library ${LINKER_OPT})
target_link_libraries(${PROJECT_NAME} PRIVATE --coverage)
add_test(${PROJECT_NAME} ${PROJECT_NAME})
#install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_BINARY_DIR}/tests)
