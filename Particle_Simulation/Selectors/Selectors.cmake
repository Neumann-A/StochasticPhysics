set(H_FILES_Selectors   Selectors/AllSelectors.h
                        Selectors/AnisotropySelector.h
                        Selectors/BasicSelector.h
                        Selectors/BoundarySelector.h
                        Selectors/FieldSelector.h
                        Selectors/ProblemSelector.h
                        Selectors/Selectors.cmake
                        Selectors/SolverSelector.h)
source_group("Selectors"        FILES ${CPP_FILES_Selectors} ${INL_FILES_Selectors} ${H_FILES_Selectors})
set(CPP_FILES ${CPP_FILES} ${CPP_FILES_Selectors})
set(INL_FILES ${INL_FILES} ${INL_FILES_Selectors})
set(H_FILES ${H_FILES} ${H_FILES_Selectors})