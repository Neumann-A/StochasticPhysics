enable_testing()

find_package(GTest CONFIG REQUIRED)

add_subdirectory (Archive_Tests)
#add_subdirectory (Archives_TestManual)
#add_subdirectory (BasicActor_Test)
add_subdirectory (Distribution_Test)
add_subdirectory (Implicit_Solver_Test)
add_subdirectory (Neel_Problem_Test)
add_subdirectory (Neel_Spherical_Test)
add_subdirectory (BrownAndNeel_Relaxation_Test)
add_subdirectory (NeelSpherical_BrownEuler_Relaxation_Test)
#add_subdirectory (Playground)
#add_subdirectory (QtGuiPlayground)
add_subdirectory (Spherical_Wrapping_Test)


