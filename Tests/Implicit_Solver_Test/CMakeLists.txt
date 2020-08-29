set(TARGET_NAME Implicit_Solver_Test)
set(CPP_FILES "Test_Function_1.cpp;Test_Function_2.cpp;Test_Function_3.cpp")
set(INL_FILES "")
set(H_FILES "Test_Function_1.h;Test_Function_2.h;Test_Function_3.h")

add_executable(${TARGET_NAME} ${CPP_FILES} ${H_FILES} ${INL_FILES} )

set_target_properties (${TARGET_NAME} PROPERTIES FOLDER tests)	

target_link_libraries(${TARGET_NAME} PRIVATE ${PROJECT_NAME}_CommonTarget)

target_link_libraries(${TARGET_NAME} PRIVATE TestWrapper)

add_test(${TARGET_NAME} ${TARGET_NAME})
