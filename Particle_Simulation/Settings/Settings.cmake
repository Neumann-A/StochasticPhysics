include_directories(Settings)
set(CPP_FILES_Settings  Settings/ParticleSimulationSettings.cpp
                        Settings/ProblemSettings.cpp
                        Settings/ResultSettings.cpp
                        Settings/SimulationSettings.cpp
                        Settings/SolverSettings.cpp)
set(H_FILES_Settings    Settings/BrownianMotionParameters.h
                        Settings/ParticleSimulationInitSettings.h
                        Settings/ParticleSimulationParameters.h
                        Settings/ParticleSimulationSettings.h
                        Settings/ProblemSettings.h
                        Settings/ResultSettings.h
                        Settings/SimulationManagerSettings.h
                        Settings/SimulationSettings.h
                        Settings/SolverSettings.h
                        Settings/SystemMatrixSettings.h
                        Settings/SystemMatrix_SimulationManagerSettings_Factory.h)
source_group("Settings"        FILES ${CPP_FILES_Settings} ${INL_FILES_Settings} ${H_FILES_Settings})
set(CPP_FILES ${CPP_FILES} ${CPP_FILES_Settings})
set(INL_FILES ${INL_FILES} ${INL_FILES_Settings})
set(H_FILES ${H_FILES} ${H_FILES_Settings})