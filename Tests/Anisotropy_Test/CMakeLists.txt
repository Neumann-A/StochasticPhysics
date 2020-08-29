set(TARGET_NAME Anisotropy_Test)

set(CPP_FILES "Anisotropy_Test.cpp")
set(INL_FILES "")
set(H_FILES "Anisotropy_Test.h")

add_executable(${TARGET_NAME} ${CPP_FILES} ${H_FILES} ${INL_FILES} $<TARGET_OBJECTS:SimulationCommon>)

set_target_properties (${TARGET_NAME} PROPERTIES FOLDER tests)	

target_link_libraries(${TARGET_NAME} PRIVATE TestWrapper)
target_compile_features(${TARGET_NAME} PRIVATE cxx_std_17)

target_include_directories(${TARGET_NAME} PRIVATE ${PROJECT_NAME}_CommonTarget)

add_test(${TARGET_NAME} ${TARGET_NAME})
