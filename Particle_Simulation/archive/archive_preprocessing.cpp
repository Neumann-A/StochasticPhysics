#include "archive_preprocessing.hpp"
#include <Eigen/Core>
#include "Settings/SimulationManagerSettings.h"
#include <SerAr/Core/ArchiveHelper.h>
#include <SerAr/AllArchiveIncludes.hpp>
#include <SerAr/AllInputArchiveWrapper.hpp>
#include <SerAr/AllOutputArchiveWrapper.hpp>

#define SimulationManagerSettings_SAVE(archive, prec) \
template void Settings::SimulationManagerSettings<prec>::save< archive  > ( archive & ar) const;
#define SimulationManagerSettings_CONSTRUCT(archive, prec) \
template Settings::SimulationManagerSettings<prec>  Archives::LoadConstructor<Settings::SimulationManagerSettings<prec>>::construct< archive > ( Archives::InputArchive<archive> & ar); 
#define SimulationManagerSettings_CONSTRUCT_WRAPPER(archive, prec) \
template Settings::SimulationManagerSettings<prec> SerAr::construct< Settings::SimulationManagerSettings<prec> > (archive& ar);

namespace SerAr {
    template<typename T>
    T construct(AllFileInputArchiveWrapper& ar) {
        return ar.construct<T>();
    };
}

SimulationManagerSettings_SAVE(SerAr::AllFileOutputArchiveWrapper,double)
SimulationManagerSettings_SAVE(SerAr::AllFileOutputArchiveWrapper,float)
SimulationManagerSettings_CONSTRUCT_WRAPPER(SerAr::AllFileInputArchiveWrapper,double)
SimulationManagerSettings_CONSTRUCT_WRAPPER(SerAr::AllFileInputArchiveWrapper,float)

#ifdef SERAR_HAS_CONFIGFILE
SimulationManagerSettings_SAVE(Archives::ConfigFile_OutputArchive,double)
SimulationManagerSettings_SAVE(Archives::ConfigFile_OutputArchive,float)
SimulationManagerSettings_CONSTRUCT(Archives::ConfigFile_InputArchive,double)
SimulationManagerSettings_CONSTRUCT(Archives::ConfigFile_InputArchive,float)
#endif

#ifdef SERAR_HAS_MATLAB
static_assert(std::is_base_of_v< Eigen::EigenBase< Eigen::Matrix<double, 3, 1, 0> > , Eigen::Matrix<double, 3, 1, 0> >);
static_assert(stdext::IsEigen3Type<Eigen::Matrix<double, 3, 1, 0>>);
static_assert(stdext::is_eigen_type_v< std::remove_cvref_t<Eigen::Matrix<double, 3, 1, 0> &> >);
static_assert(SerAr::ArchiveMemberSaveable<Eigen::Matrix<double, 3, 1, 0> &, SerAr::Matlab_OutputArchive>);
static_assert(SerAr::HasCreateMATLAB<SerAr::Matlab_OutputArchive, std::remove_cvref_t<Eigen::Matrix<double, 3, 1, 0> &>>);
SimulationManagerSettings_SAVE(SerAr::Matlab_OutputArchive,double)
SimulationManagerSettings_SAVE(SerAr::Matlab_OutputArchive,float)
SimulationManagerSettings_CONSTRUCT(SerAr::Matlab_InputArchive,double)
SimulationManagerSettings_CONSTRUCT(SerAr::Matlab_InputArchive,float)
#endif

#ifdef SERAR_HAS_HDF5
SimulationManagerSettings_SAVE(Archives::HDF5_OutputArchive,double)
SimulationManagerSettings_SAVE(Archives::HDF5_OutputArchive,float)
SimulationManagerSettings_CONSTRUCT(Archives::HDF5_InputArchive,double)
SimulationManagerSettings_CONSTRUCT(Archives::HDF5_InputArchive,float)
#endif

#ifdef SERAR_HAS_JSON
SimulationManagerSettings_SAVE(SerAr::JSON_OutputArchive,double)
SimulationManagerSettings_SAVE(SerAr::JSON_OutputArchive,float)
SimulationManagerSettings_CONSTRUCT(SerAr::JSON_InputArchive,double)
SimulationManagerSettings_CONSTRUCT(SerAr::JSON_InputArchive,float)
#endif

#ifdef SERAR_HAS_TOML
SimulationManagerSettings_SAVE(SerAr::TOML_OutputArchive,double)
SimulationManagerSettings_SAVE(SerAr::TOML_OutputArchive,float)
SimulationManagerSettings_CONSTRUCT(SerAr::TOML_InputArchive,double)
SimulationManagerSettings_CONSTRUCT(SerAr::TOML_InputArchive,float)
#endif