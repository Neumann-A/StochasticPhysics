#pragma once
#include <filesystem>
#include <string_view>
#include <optional>
#include <boost/program_options.hpp>

#include <string>
#include <memory>
#include <utility>

#include <Eigen/Core> // Required before including the Simulation Manager Traits

#include "General/GeneralDefines.h"

//#include "Simulator/SimulationManager.h" // Should include everything!
#include "Simulator/SimulationManagerTraits.h"
#include "Settings/PhantomSettings.h"

namespace bo_opts = ::boost::program_options;
namespace fs = ::std::filesystem;

template<typename AppTraits>
struct SysMatFileGen_Input;

using ThisAppTraits = typename SimulationApplication::SimulationManagerTraits<SimulationApplication::SimulationManager<PREC>>;

template<>
struct SysMatFileGen_Input<ThisAppTraits>
{
    using AppParams = typename ThisAppTraits::Parameters;
    using InputArchive = typename ThisAppTraits::StartInputArchive;
    using OutputArchive = typename ThisAppTraits::StartOutputArchive;
    using SysMatParams = Settings::SystemMatrixSettings<PREC>;
    using PhantomParams = Settings::PhantomSettings<PREC>;
    static struct CmdOpts
    {
        fs::path parameter_file;
        fs::path matrix_file;
        fs::path phantom_file;
        fs::path output_file_name;
        fs::path save_directory;
    } options;

    static bo_opts::options_description buildOptionDescriptor();
    static const bo_opts::options_description optdesc;


    SysMatFileGen_Input(int argc, char** argv);

    AppParams getAppParams();
    PhantomParams getPhantom();
    SysMatParams getSysMatParams();
    static CmdOpts& parseCmdLineOptions(int argc, char** argv);
    static void displayHelp();

};


