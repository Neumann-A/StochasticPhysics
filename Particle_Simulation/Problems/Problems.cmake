set(INL_FILES_Problems Problems/BrownAndNeelRelaxation.inl)
set(H_FILES_Problems    Problems/BrownAndNeelRelaxation.h
                        Problems/BrownAndNeelRelaxationEulerSpherical.h
                        Problems/BrownianMotion.h
                        Problems/NeelRelaxation.h
                        Problems/NeelRelaxationQuaternion.h
                        Problems/NeelRelaxationSpherical.h
                        Problems/Problems.h
                        Problems/ProblemsFWD.h)
source_group("Problems"        FILES ${CPP_FILES_Problems} ${INL_FILES_Problems} ${H_FILES_Problems})
set(CPP_FILES ${CPP_FILES} ${CPP_FILES_Problems})
set(INL_FILES ${INL_FILES} ${INL_FILES_Problems})
set(H_FILES ${H_FILES} ${H_FILES_Problems})

#Anisotropy in Problem
set(CPP_FILES_ProblemAnisotropy Problems/Anisotropy/AnisotropyList.cpp)
set(H_FILES_ProblemAnisotropy   Problems/Anisotropy/AnisotropyList.h
                                Problems/Anisotropy/CubicAnisotropy.h
                                Problems/Anisotropy/GeneralAnisotropy.h
                                Problems/Anisotropy/UniaxialAnisotropy.h
                                Problems/Anisotropy/MixedAnisotropy.h
                                Problems/Anisotropy/UniaxialCubicAnisotropy.hpp)
source_group("Problems\\Anisotropy"        FILES ${CPP_FILES_ProblemAnisotropy} ${INL_FILES_ProblemAnisotropy} ${H_FILES_ProblemAnisotropy})
set(CPP_FILES ${CPP_FILES} ${CPP_FILES_ProblemAnisotropy})
set(INL_FILES ${INL_FILES} ${INL_FILES_ProblemAnisotropy})
set(H_FILES ${H_FILES} ${H_FILES_ProblemAnisotropy})

#Boundary_Condition
set(H_FILES_Problems_Boundary Problems/Boundary_Condition/BoundaryCondition.h)
source_group("Problems\\Boundary_Condition"        FILES ${CPP_FILES_Problems_Boundary} ${INL_FILES_Problems_Boundary} ${H_FILES_Problems_Boundary})
set(CPP_FILES ${CPP_FILES} ${CPP_FILES_Problems_Boundary})
set(INL_FILES ${INL_FILES} ${INL_FILES_Problems_Boundary})
set(H_FILES ${H_FILES} ${H_FILES_Problems_Boundary})

#Definitions
set(H_FILES_Problems_Definitions    Problems/Definitions/BrownAndNeelRelaxationEulerSpherical_Definitions.h
                                    Problems/Definitions/BrownAndNeel_Definitions.h
                                    Problems/Definitions/GeneralProblem_Definitions.h
                                    Problems/Definitions/NeelRelaxationQuaternion_Definitions.h
                                    Problems/Definitions/NeelRelaxationSpherical_Definitions.h
                                    Problems/Definitions/NeelRelaxation_Definitions.h)
source_group("Problems\\Definitions"        FILES ${CPP_FILES_Problems_Definitions} ${INL_FILES_Problems_Definitions} ${H_FILES_Problems_Definitions})
set(CPP_FILES ${CPP_FILES} ${CPP_FILES_Problems_Definitions})
set(INL_FILES ${INL_FILES} ${INL_FILES_Problems_Definitions})
set(H_FILES ${H_FILES} ${H_FILES_Problems_Definitions})

#Helpers
set(H_FILES_Problems_Helpers    Problems/Helpers/ParameterCalculatorBrown.h
                                Problems/Helpers/ParameterCalculatorBrownAndNeel.h
                                Problems/Helpers/ParameterCalculatorNeel.h
                                Problems/Helpers/ParticleStateInitializer.h)
source_group("Problems\\Helpers"        FILES ${CPP_FILES_Problems_Helpers} ${INL_FILES_Problems_Helpers} ${H_FILES_Problems_Helpers})
set(CPP_FILES ${CPP_FILES} ${CPP_FILES_Problems_Helpers})
set(INL_FILES ${INL_FILES} ${INL_FILES_Problems_Helpers})
set(H_FILES ${H_FILES} ${H_FILES_Problems_Helpers})