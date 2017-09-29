project (Implicit_Solver_Test)

set(CPP_FILES "Test_Implicit_Solver_Main.cpp;Test_Function_1.cpp;Test_Function_2.cpp;Test_Function_3.cpp")
set(INL_FILES "")
set(H_FILES "Test_Function_1.h;Test_Function_2.h;Test_Function_3.h")

add_executable(${PROJECT_NAME} ${CPP_FILES} ${H_FILES} ${INL_FILES})

set_target_properties (${PROJECT_NAME} PROPERTIES FOLDER tests)	

if(Simulation_Boost)
find_package(Boost REQUIRED COMPONENTS random)
target_include_directories(${PROJECT_NAME} PRIVATE Boost::random)
endif()

if(Simulation_PCG)
find_path(PCG_INCLUDE NAMES "pcg_random.hpp" HINTS "${CMAKE_SOURCE_DIR}../Extern/pcg/include/" "${CMAKE_SOURCE_DIR}../../Extern/pcg/include/" "${CMAKE_SOURCE_DIR}../../../Extern/pcg/include/")
message(STATUS "PCG path: " ${PCG_INCLUDE})
include_directories(${PCG_INCLUDE})
endif()

find_package(GTest REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE GTest::GTest GTest::Main)

target_link_libraries(${PROJECT_NAME} PRIVATE --coverage)
add_test(${PROJECT_NAME} ${PROJECT_NAME})
