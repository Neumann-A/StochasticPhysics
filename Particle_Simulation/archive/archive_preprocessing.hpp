#pragma once
#include <Eigen/Core>
#include "Settings/SimulationManagerSettings.h"

// extern template Settings::SimulationManagerSettings<double>;
// extern template Settings::SimulationManagerSettings<float>;
// extern template Archives::LoadConstructor<Settings::SimulationManagerSettings<double>>;
// extern template Archives::LoadConstructor<Settings::SimulationManagerSettings<float>>;


#define SimulationManagerSettings_SAVE(archive, prec) \
extern template void Settings::SimulationManagerSettings<prec>::save< archive  > ( archive & ar) const;
#define SimulationManagerSettings_CONSTRUCT(archive, prec) \
extern template Settings::SimulationManagerSettings<prec> Archives::LoadConstructor<Settings::SimulationManagerSettings<prec>>::construct< archive > ( Archives::InputArchive<archive> & ar);

#ifdef SERAR_HAS_CONFIGFILE
namespace Archives {
class ConfigFile_OutputArchive;
class ConfigFile_InputArchive;
}
SimulationManagerSettings_SAVE(Archives::ConfigFile_OutputArchive,double)
SimulationManagerSettings_SAVE(Archives::ConfigFile_OutputArchive,float)
SimulationManagerSettings_CONSTRUCT(Archives::ConfigFile_InputArchive,double)
SimulationManagerSettings_CONSTRUCT(Archives::ConfigFile_InputArchive,float)
#endif

#ifdef SERAR_HAS_MATLAB
namespace Archives {
class MatlabOutputArchive;
class MatlabInputArchive;
}
SimulationManagerSettings_SAVE(Archives::MatlabOutputArchive,double)
SimulationManagerSettings_SAVE(Archives::MatlabOutputArchive,float)
SimulationManagerSettings_CONSTRUCT(Archives::MatlabInputArchive,double)
SimulationManagerSettings_CONSTRUCT(Archives::MatlabInputArchive,float)
#endif

#ifdef SERAR_HAS_HDF5
namespace Archives {
class HDF5_OutputArchive;
class HDF5_InputArchive;
}
SimulationManagerSettings_SAVE(Archives::HDF5_OutputArchive,double)
SimulationManagerSettings_SAVE(Archives::HDF5_OutputArchive,float)
SimulationManagerSettings_CONSTRUCT(Archives::HDF5_InputArchive,double)
SimulationManagerSettings_CONSTRUCT(Archives::HDF5_InputArchive,float)
#endif

#ifdef SERAR_HAS_JSON
namespace SerAr {
class JSON_OutputArchive;
class JSON_InputArchive;
}
SimulationManagerSettings_SAVE(SerAr::JSON_OutputArchive,double)
SimulationManagerSettings_SAVE(SerAr::JSON_OutputArchive,float)
SimulationManagerSettings_CONSTRUCT(SerAr::JSON_InputArchive,double)
SimulationManagerSettings_CONSTRUCT(SerAr::JSON_InputArchive,float)
#endif

#undef SimulationManagerSettings_SAVE
#undef SimulationManagerSettings_CONSTRUCT