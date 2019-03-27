include_directories(Provider)
set(CPP_FILES_Provider  Provider/ParticleProvider.cpp)
set(H_FILES_Provider    Provider/ParticleProvider.h
                        Provider/PropertyProvider.h)
source_group("Provider"        FILES ${CPP_FILES_Provider} ${INL_FILES_Provider} ${H_FILES_Provider})
set(CPP_FILES ${CPP_FILES} ${CPP_FILES_Provider})
set(INL_FILES ${INL_FILES} ${INL_FILES_Provider})
set(H_FILES ${H_FILES} ${H_FILES_Provider})