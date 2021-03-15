#ifndef INC_SystemMatrixSimulationManagerSettingsFactory_H
#define INC_SystemMatrixSimulationManagerSettingsFactory_H

#pragma once
#include <vector>
#include <filesystem>

#include <Eigen/Core>

#include "SimulationManagerSettings.h"
#include "SystemMatrixSettings.h"
#include "Properties/VoxelInformation.h"

namespace Settings
{	

    class SystemMatrix_SimulationManagerSettings_Factory
    {
    public:
        template <typename prec>
        static std::vector<std::tuple<SimulationManagerSettings<prec>,VoxelInformation>> createSimulationManagerSettingsSystemMatrix(const SimulationManagerSettings<prec>& simSet,const SystemMatrixSettings<prec>& sysMatSet)
        {
            //Result Vector with Settings
            std::vector<std::tuple<SimulationManagerSettings<prec>, VoxelInformation> > retVec;

            //Local copy of Settings given to the application
            auto settingsLocal{ simSet };
            auto& resSetLocal = settingsLocal.getResultSettings();
            auto& fieldPropLocal = settingsLocal.getFieldProperties();

            //Get a copy of the offset field! (since we will change the original value)
            const auto offsetfield{ fieldPropLocal._FieldParameter._Amplitudes.at(0) };

            //Get Filenames
            const auto& fPathLocal{ resSetLocal.getFilepath() };
            const auto& fPathLocalSingle{ resSetLocal.getSaveFilepathSingle() };

            //Get Parent, Stem and Extension
            const auto PathParentLocal{ fPathLocal.parent_path() };
            const auto PathStemLocal{ fPathLocal.stem() };
            const auto PathExtLocal{ fPathLocal.extension() };
            
            //Get Parent, Stem and Extension (single)
            const auto PathSingleParentLocal{ fPathLocalSingle.parent_path() };
            const auto PathSingleStemLocal{ fPathLocalSingle.stem() };
            const auto PathSingleExtLocal{ fPathLocalSingle.extension() };

            auto VoxelFieldTupleVector = sysMatSet.generateFieldsAndVoxels();
            for (auto& VoxelFieldTuple : VoxelFieldTupleVector)
            {
                const auto& Voxel = std::get<0>(VoxelFieldTuple);
                const auto& Field = std::get<1>(VoxelFieldTuple);
                fieldPropLocal.getAmplitudes().at(0) = Field + offsetfield;

                //Creating the new filename
                std::filesystem::path pathdetails{ "_Vox_" + std::to_string(Voxel(0)) + "_" + std::to_string(Voxel(1)) + "_" + std::to_string(Voxel(2)) };
                
                // Boilerplate code because path is missing a proper operator+ implementation (only operator+= is avaible) 
                std::filesystem::path path1{ PathParentLocal };
                std::filesystem::path path2{ PathSingleParentLocal };

                path1 /= PathStemLocal;
                path1 += pathdetails;
                path1 += PathExtLocal;

                path2 /= PathSingleStemLocal;
                path2 += pathdetails;
                path2 += PathSingleExtLocal;

                //Setting the new filenames
                resSetLocal.setFilepath(path1);
                resSetLocal.setSaveFilepathSingle(path2);

                //Logger::Log("Aktuelle: File-Ausgabe in der Factory: " + settingsLocal.getResultSettings().getFilepath().string());
                VoxelInformation VoxelInfo{ Voxel };
                retVec.push_back(std::make_tuple(settingsLocal,VoxelInfo));
            }
            return retVec;
        }

    };
}
#endif	// INC_SystemMatrixSimulationManagerSettingsFactory_H
