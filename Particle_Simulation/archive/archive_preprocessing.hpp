#pragma once
#include <filesystem>
#include <Eigen/Core>
#include "Settings/SimulationManagerSettings.h"

// extern template Settings::SimulationManagerSettings<double>;
// extern template Settings::SimulationManagerSettings<float>;
// extern template Archives::LoadConstructor<Settings::SimulationManagerSettings<double>>;
// extern template Archives::LoadConstructor<Settings::SimulationManagerSettings<float>>;
namespace SerAr {
    class AllFileInputArchiveWrapper;
    class AllFileOutputArchiveWrapper;

    template<typename T>
    T construct(AllFileInputArchiveWrapper& ar);
}

#define SimulationManagerSettings_SAVE(archive, prec) \
extern template void Settings::SimulationManagerSettings<prec>::save< archive  > ( archive & ar) const;
#define SimulationManagerSettings_CONSTRUCT(archive, prec) \
extern template Settings::SimulationManagerSettings<prec> Archives::LoadConstructor<Settings::SimulationManagerSettings<prec>>::construct< archive > ( Archives::InputArchive<archive> & ar);
#define SimulationManagerSettings_CONSTRUCT_WRAPPER(archive, prec) \
extern template Settings::SimulationManagerSettings<prec> SerAr::construct< Settings::SimulationManagerSettings<prec> > (archive& ar);


SimulationManagerSettings_SAVE(SerAr::AllFileOutputArchiveWrapper,double)
SimulationManagerSettings_SAVE(SerAr::AllFileOutputArchiveWrapper,float)
SimulationManagerSettings_CONSTRUCT_WRAPPER(SerAr::AllFileInputArchiveWrapper,double)
SimulationManagerSettings_CONSTRUCT_WRAPPER(SerAr::AllFileInputArchiveWrapper,float)

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
namespace SerAr {
class Matlab_OutputArchive;
class Matlab_InputArchive;
}
SimulationManagerSettings_SAVE(SerAr::Matlab_OutputArchive,double)
SimulationManagerSettings_SAVE(SerAr::Matlab_OutputArchive,float)
SimulationManagerSettings_CONSTRUCT(SerAr::Matlab_InputArchive,double)
SimulationManagerSettings_CONSTRUCT(SerAr::Matlab_InputArchive,float)
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

#ifdef SERAR_HAS_TOML
namespace SerAr {
class TOML_OutputArchive;
class TOML_InputArchive;
}
SimulationManagerSettings_SAVE(SerAr::TOML_OutputArchive,double)
SimulationManagerSettings_SAVE(SerAr::TOML_OutputArchive,float)
SimulationManagerSettings_CONSTRUCT(SerAr::TOML_InputArchive,double)
SimulationManagerSettings_CONSTRUCT(SerAr::TOML_InputArchive,float)
#endif


#undef SimulationManagerSettings_SAVE
#undef SimulationManagerSettings_CONSTRUCT
#undef SimulationManagerSettings_CONSTRUCT_WRAPPER
