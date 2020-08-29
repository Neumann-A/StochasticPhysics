
set(TARGET_NAME Neel_Spherical_Test)

set(CPP_FILES "Test_Neel_Spherical_Problem.cpp")
set(INL_FILES "")
set(H_FILES "Test_Neel_Spherical_Problem.h;")

add_executable(${TARGET_NAME} ${CPP_FILES} ${H_FILES} ${INL_FILES} $<TARGET_OBJECTS:SimulationCommon>)

set_target_properties (${TARGET_NAME} PROPERTIES FOLDER tests)	

target_link_libraries(${TARGET_NAME} PRIVATE ${PROJECT_NAME}_CommonTarget)

add_test(${TARGET_NAME} ${TARGET_NAME})
