set(INL_FILES_SDEFramework SDEFramework/NoiseField.inl
                            SDEFramework/NoiseField_SIMD.inl)
set(H_FILES_SDEFramework    SDEFramework/DoubleNoiseMatrix.h
                            SDEFramework/GeneralField.h
                            SDEFramework/GeneralSDEProblem.h
                            SDEFramework/NoiseField.h
                            SDEFramework/NoiseField_SIMD.h
                            )
source_group("SDEFramework"        FILES ${CPP_FILES_SDEFramework} ${INL_FILES_SDEFramework} ${H_FILES_SDEFramework})
set(CPP_FILES ${CPP_FILES} ${CPP_FILES_SDEFramework})
set(INL_FILES ${INL_FILES} ${INL_FILES_SDEFramework})
set(H_FILES ${H_FILES} ${H_FILES_SDEFramework})

set(INL_FILES_SDEFramework/Solver   SDEFramework/Solver/EulerMaruyama.inl
                                    SDEFramework/Solver/EulerMaruyama_Normalized.inl
                                    SDEFramework/Solver/Explicit_Strong_1.inl
                                    SDEFramework/Solver/Heun_NotConsistent.inl
                                    SDEFramework/Solver/Implicit_Midpoint.inl
                                    SDEFramework/Solver/Implicit_Midpoint_GSL.inl
                                    SDEFramework/Solver/Millstein.inl
                                    SDEFramework/Solver/WeakTest.inl)
set(H_FILES_SDEFramework/Solver SDEFramework/Solver/EulerMaruyama.h
                                SDEFramework/Solver/EulerMaruyama_Normalized.h
                                SDEFramework/Solver/Explicit_Strong_1.h
                                SDEFramework/Solver/GeneralSDESolver.h
                                SDEFramework/Solver/Heun_NotConsistent.h
                                SDEFramework/Solver/Heun_Strong.h
                                SDEFramework/Solver/Heun_Strong.inl
                                SDEFramework/Solver/Implicit_Midpoint.h
                                SDEFramework/Solver/Implicit_Midpoint_GSL.h
                                SDEFramework/Solver/Implicit_Midpoint_GSL_Derivative_Free.h
                                SDEFramework/Solver/Millstein.h
                                SDEFramework/Solver/SDESolvers.h
                                SDEFramework/Solver/WeakTest.h)
source_group("SDEFramework\\Solver"        FILES ${CPP_FILES_SDEFramework/Solver} ${INL_FILES_SDEFramework/Solver} ${H_FILES_SDEFramework/Solver})
set(CPP_FILES ${CPP_FILES} ${CPP_FILES_SDEFramework/Solver})
set(INL_FILES ${INL_FILES} ${INL_FILES_SDEFramework/Solver})
set(H_FILES ${H_FILES} ${H_FILES_SDEFramework/Solver})

