set(H_FILES_Results Results/MeanSimulationResult.h
                    Results/ResultFactory.h
                    Results/ResultManagerFactory.h
                    Results/SimulationResultManager.h
                    Results/SingleSimulationResult.h)
source_group("Results"        FILES ${CPP_FILES_Results} ${INL_FILES_Results} ${H_FILES_Results})
set(CPP_FILES ${CPP_FILES} ${CPP_FILES_Results})
set(INL_FILES ${INL_FILES} ${INL_FILES_Results})
set(H_FILES ${H_FILES} ${H_FILES_Results})