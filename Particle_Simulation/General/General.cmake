
set(CPP_FILES_General General/CommandOptions.cpp)
set(H_FILES_General General/CommandOptions.h
                    General/GeneralDefines.h
                    General/Setup.h)
source_group("General"        FILES ${CPP_FILES_General} ${INL_FILES_General} ${H_FILES_General})
set(CPP_FILES ${CPP_FILES} ${CPP_FILES_General})
set(INL_FILES ${INL_FILES} ${INL_FILES_General})
set(H_FILES ${H_FILES} ${H_FILES_General})