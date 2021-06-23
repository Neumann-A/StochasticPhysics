set(TARGET_NAME Random_State_Init_Test)

set(CPP_FILES "Random_State_Init_Test.cpp")
set(INL_FILES "")
set(H_FILES "Random_State_Init_Test.h")

add_executable(${TARGET_NAME} ${CPP_FILES} ${H_FILES} ${INL_FILES})

set_target_properties (${TARGET_NAME} PROPERTIES FOLDER tests)	

target_link_libraries(${TARGET_NAME} PRIVATE TestWrapper)
target_link_libraries(${TARGET_NAME} PRIVATE ${PROJECT_NAME}_CommonTarget)

gtest_discover_tests(${TARGET_NAME})