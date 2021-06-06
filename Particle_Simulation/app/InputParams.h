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

namespace bo_opts = ::boost::program_options;
namespace fs = ::std::filesystem;

template<typename AppTraits>
struct InputParams;

using ThisAppTraits = typename SimulationApplication::SimulationManagerTraits<SimulationApplication::SimulationManager<PREC>>;

template<>
struct InputParams<ThisAppTraits>
{
    using AppParams = typename ThisAppTraits::Parameters;
    using InputArchive = typename ThisAppTraits::StartInputArchive;
    using OutputArchive = typename ThisAppTraits::StartOutputArchive;

    static struct CmdOpts
    {
        fs::path parameter_file;
        fs::path matrix_file;
        MyCEL::SystemInfo::InstructionSet arch;
    } options;

    static bo_opts::options_description buildOptionDescriptor();
    static const bo_opts::options_description optdesc;


    InputParams(int argc, char** argv);

    AppParams getAppParams();
    AppParams getDefaultedAppParams();
    static CmdOpts& parseCmdLineOptions(int argc, char** argv);
    static void displayHelp();

};


