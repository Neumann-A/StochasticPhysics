set(CPP_FILES_Properties    Properties/FieldProperties.cpp
                            Properties/HydrodynamicProperties.cpp
                            Properties/MagneticProperties.cpp)
set(H_FILES_Properties  Properties/FieldProperties.h
                        Properties/HydrodynamicProperties.h
                        Properties/MagneticProperties.h
                        Properties/ParticleProperties.h
                        Properties/SpatialInformation.h
                        Properties/VoxelInformation.h
                        Properties/Anisotropy/All.hpp
                        Properties/Anisotropy/Anisotropy.hpp
                        Properties/Anisotropy/Cubic.hpp
                        Properties/Anisotropy/Mixed.hpp
                        Properties/Anisotropy/Uniaxial.hpp
                        Properties/Anisotropy/UniaxialCubic.hpp)
source_group("Properties"        FILES ${CPP_FILES_Properties} ${INL_FILES_Properties} ${H_FILES_Properties})
set(CPP_FILES ${CPP_FILES} ${CPP_FILES_Properties})
set(INL_FILES ${INL_FILES} ${INL_FILES_Properties})
set(H_FILES ${H_FILES} ${H_FILES_Properties})