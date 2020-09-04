set(CPP_FILES_Simulator Simulator/SimulationApplication.cpp)
set(H_FILES_Simulator   Simulator/ISingleParticleSimulator.h
                        Simulator/SimulationManager.h
                        Simulator/SimulationManagerTraits.h
                        Simulator/SingleParticleSimulator.h)
source_group("Simulator"        FILES ${CPP_FILES_Simulator} ${INL_FILES_Simulator} ${H_FILES_Simulator})
set(CPP_FILES ${CPP_FILES} )
set(INL_FILES ${INL_FILES} ${INL_FILES_Simulator})
set(H_FILES ${H_FILES} ${H_FILES_Simulator})