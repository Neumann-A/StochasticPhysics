set(TARGET_NAME Problem_Neel_Test)
set(CPP_FILES "Test_ParamCalc_Neel.cpp;Test_Problem_Neel.cpp;Test_Uniaxial_Anisotropy.cpp")
set(INL_FILES "")
set(H_FILES "Test_ParamCalc_Neel.h;Test_Problem_Neel.h;Test_Uniaxial_Anisotropy.h")

add_executable(${TARGET_NAME} ${CPP_FILES} ${H_FILES} ${INL_FILES} $<TARGET_OBJECTS:SimulationCommon>)

set_target_properties (${TARGET_NAME} PROPERTIES FOLDER tests)	

target_link_libraries(${TARGET_NAME} PRIVATE TestWrapper)

target_link_libraries(${TARGET_NAME} PRIVATE ${PROJECT_NAME}_CommonTarget)

add_test(${TARGET_NAME} ${TARGET_NAME})
#install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_BINARY_DIR}/tests)
