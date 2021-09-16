#include "archive_preprocessing.hpp"
#include "Settings/SimulationManagerSettings.h"
#include <SerAr/Core/ArchiveHelper.h>
// template Settings::SimulationManagerSettings<double>;
// template Settings::SimulationManagerSettings<float>;

// template Archives::LoadConstructor<Settings::SimulationManagerSettings<double>>;
// template Archives::LoadConstructor<Settings::SimulationManagerSettings<float>>;

#define SimulationManagerSettings_SAVE(archive, prec) \
template void Settings::SimulationManagerSettings<prec>::save< archive  > ( archive & ar) const;
#define SimulationManagerSettings_CONSTRUCT(archive, prec) \
template Settings::SimulationManagerSettings<prec>  Archives::LoadConstructor<Settings::SimulationManagerSettings<prec>>::construct< archive > ( Archives::InputArchive<archive> & ar); 

#ifdef SERAR_HAS_CONFIGFILE
#include <SerAr/ConfigFile/ConfigFile_Archive.h>
SimulationManagerSettings_SAVE(Archives::ConfigFile_OutputArchive,double)
SimulationManagerSettings_SAVE(Archives::ConfigFile_OutputArchive,float)
//template Settings::SimulationManagerSettings<double> Archives::LoadConstructor<Settings::SimulationManagerSettings<double>>::construct< Archives::ConfigFile_InputArchive > ( Archives::InputArchive<Archives::ConfigFile_InputArchive> & ar);
SimulationManagerSettings_CONSTRUCT(Archives::ConfigFile_InputArchive,double)
SimulationManagerSettings_CONSTRUCT(Archives::ConfigFile_InputArchive,float)
#endif

#ifdef SERAR_HAS_MATLAB
#include <SerAr/MATLAB/Matlab_Archive.h>
static_assert(std::is_base_of_v< Eigen::EigenBase< Eigen::Matrix<double, 3, 1, 0> > , Eigen::Matrix<double, 3, 1, 0> >);
static_assert(SerAr::ArchiveMemberSaveable<Eigen::Matrix<double, 3, 1, 0> &, Archives::MatlabOutputArchive>);
static_assert(Archives::HasCreateMATLAB<Archives::MatlabOutputArchive, std::remove_cvref_t<Eigen::Matrix<double, 3, 1, 0> &>>);
static_assert(stdext::is_eigen_type_v< std::remove_cvref_t<Eigen::Matrix<double, 3, 1, 0> &> >);
SimulationManagerSettings_SAVE(Archives::MatlabOutputArchive,double)
SimulationManagerSettings_SAVE(Archives::MatlabOutputArchive,float)
SimulationManagerSettings_CONSTRUCT(Archives::MatlabInputArchive,double)
SimulationManagerSettings_CONSTRUCT(Archives::MatlabInputArchive,float)
#endif

#ifdef SERAR_HAS_HDF5
#include <SerAr/HDF5/HDF5_Archive.h>
SimulationManagerSettings_SAVE(Archives::HDF5_OutputArchive,double)
SimulationManagerSettings_SAVE(Archives::HDF5_OutputArchive,float)
SimulationManagerSettings_CONSTRUCT(Archives::HDF5_InputArchive,double)
SimulationManagerSettings_CONSTRUCT(Archives::HDF5_InputArchive,float)
#endif

#ifdef SERAR_HAS_JSON
#include <SerAr/JSON/JSON_Archives.hpp>
SimulationManagerSettings_SAVE(SerAr::JSON_OutputArchive,double)
SimulationManagerSettings_SAVE(SerAr::JSON_OutputArchive,float)
SimulationManagerSettings_CONSTRUCT(SerAr::JSON_InputArchive,double)
SimulationManagerSettings_CONSTRUCT(SerAr::JSON_InputArchive,float)
#endif

#ifdef SERAR_HAS_TOML
#include <SerAr/TOML/TOML_Archives.hpp>
SimulationManagerSettings_SAVE(SerAr::TOML_OutputArchive,double)
SimulationManagerSettings_SAVE(SerAr::TOML_OutputArchive,float)
SimulationManagerSettings_CONSTRUCT(SerAr::TOML_InputArchive,double)
SimulationManagerSettings_CONSTRUCT(SerAr::TOML_InputArchive,float)
#endif