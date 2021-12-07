

#include <MyCEL/basics/GlobalParameters.h>
#include <MyCEL/basics/Logger.h>

#include <vector>
#include <stdexcept>

#include <fmt/format.h>
#include <Eigen/Core>


//#include "Settings/SimulationManagerSettings.h"
#include "Settings/SystemMatrixSettings.h"
#include "Properties/ParticleProperties.h"

#include <SerAr/SerAr.hpp>
#include "archive/archive_preprocessing.hpp"

#include "SysMatFileGen_Input.h"
namespace bo_opts = ::boost::program_options;
namespace fs      = ::std::filesystem;

struct opts_string
{
    std::string_view parameter_file;
    std::string_view matrix_file;
    std::string_view phantom_file;
    std::string_view output_file_name;
    std::string_view save_directory;
};

constexpr const opts_string optstr{"parameter_file", "matrix_file","phantom_file", "output_file_name","save_directory"};

SysMatFileGen_Input<ThisAppTraits>::SysMatFileGen_Input(int argc, char** argv) { 
    try {
        parseCmdLineOptions(argc, argv); 
    }
    catch (std::runtime_error& e)
    {
        std::puts(e.what());
    }
}

bo_opts::options_description SysMatFileGen_Input<ThisAppTraits>::buildOptionDescriptor()
{
    bo_opts::options_description desc{"Options"};
    desc.add_options()("help", "Displays the help")
        (
            optstr.parameter_file.data(),
            bo_opts::value(&options.parameter_file)->default_value({}),
            "Path to a valid parameter config file"
        )
        (
            optstr.matrix_file.data(),
            bo_opts::value(&options.matrix_file)->default_value({}),
            "Path to a matrix file (not implemented yet)"
        )
        (
            optstr.phantom_file.data(),
            bo_opts::value(&options.phantom_file)->default_value({}),
            "Path to a phantom file (not implemented yet)"
        )
        (
            optstr.output_file_name.data(), 
            bo_opts::value(&options.output_file_name)->default_value({"Simulation_Settings"}), 
            "Filename to write the settings to!"
        )
        (
            optstr.save_directory.data(), 
            bo_opts::value(&options.save_directory)->default_value({}), 
            "Save directory!"
        );
    return desc;
}

SysMatFileGen_Input<ThisAppTraits>::CmdOpts& SysMatFileGen_Input<ThisAppTraits>::parseCmdLineOptions(int argc, char** argv)
{
    bo_opts::variables_map vm;
    try {
        bo_opts::store(bo_opts::parse_command_line(argc, argv, optdesc), vm);
    }
    catch (std::exception& exp) {
        std::puts(exp.what());
        displayHelp();
        std::exit(1);
    }
    bo_opts::notify(vm);
    if (vm.count("help") > 0) {
        displayHelp();
    }
    return options;
}

SysMatFileGen_Input<ThisAppTraits>::AppParams SysMatFileGen_Input<ThisAppTraits>::getAppParams()
{
    if (!options.parameter_file.empty()) {
        const auto& filepath = options.parameter_file;
        Logger::Log("Trying to load parameters from: %s \n", filepath.u8string().c_str());
        if (std::filesystem::exists(filepath)) {
            const auto archive_enum = SerAr::getArchiveEnumByExtension(filepath.extension().string());
            if(!archive_enum) {
                const auto error = fmt::format("No archive known to support the file extension: '{}'", filepath.extension().string() );
                throw std::runtime_error{error.c_str()};
            }
            auto InputFile = input_archive_from_enum(*archive_enum, filepath) ;
            return std::visit([](auto& archive) -> SysMatFileGen_Input<ThisAppTraits>::AppParams {return Archives::LoadConstructor<AppParams>::construct(archive);}, InputFile.variant);
        }
    }
    Logger::Log("Paramter file required! Use --parmeter_file=<somefile> to load particle parameters!\nmatrix/phantom file required Use --matrix_file or --phantom_file to load parameters! \n");
    throw std::runtime_error("No parameter_file found");
}

SysMatFileGen_Input<ThisAppTraits>::PhantomParams SysMatFileGen_Input<ThisAppTraits>::getPhantom()
{
    if (!options.phantom_file.empty()) {
        const auto& filepath = options.phantom_file;
        Logger::Log("Trying to load parameters from: %s \n", filepath.u8string().c_str());
        if (std::filesystem::exists(filepath)) {
            const auto archive_enum = SerAr::getArchiveEnumByExtension(filepath.extension().string());
            if(!archive_enum) {
                const auto error = fmt::format("No archive known to support the file extension: '{}'", filepath.extension().string() );
                throw std::runtime_error{error.c_str()};
            }
            auto InputFile = input_archive_from_enum(*archive_enum, filepath) ;
            return std::visit([](auto& archive) -> SysMatFileGen_Input<ThisAppTraits>::PhantomParams {return Archives::LoadConstructor<PhantomParams>::construct(archive);}, InputFile.variant);
        }
    }
    Logger::Log("Paramter file required! Use --phantom_file=<somefile> to load parameters!\n");
    throw std::runtime_error("No phantom_file found");
}

SysMatFileGen_Input<ThisAppTraits>::SysMatParams SysMatFileGen_Input<ThisAppTraits>::getSysMatParams()
{
    if (!options.matrix_file.empty()) {
        const auto& filepath = options.matrix_file;
        Logger::Log("Trying to load systemmatrix settings from: %s \n", filepath.u8string().c_str());
        if (std::filesystem::exists(filepath)) {
            const auto archive_enum = SerAr::getArchiveEnumByExtension(filepath.extension().string());
            if(!archive_enum) {
                const auto error = fmt::format("No archive known to support the file extension: '{}'", filepath.extension().string() );
                throw std::runtime_error{error.c_str()};
            }
            auto InputFile = input_archive_from_enum(*archive_enum, filepath) ;
            return std::visit([](auto& archive) -> SysMatFileGen_Input<ThisAppTraits>::SysMatParams {return Archives::LoadConstructor<SysMatParams>::construct(archive);}, InputFile.variant);
        }
    }
    Logger::Log("system matrix file required! Use --matrix_file=<somefile> to load parameters!\n");
    throw std::runtime_error("No matrix_file found");
}

void SysMatFileGen_Input<ThisAppTraits>::displayHelp()
{
    ::std::cerr << "\n";
    ::std::cerr << optdesc << std::endl;
}

const bo_opts::options_description SysMatFileGen_Input<ThisAppTraits>::optdesc = {buildOptionDescriptor()};

  SysMatFileGen_Input<ThisAppTraits>::CmdOpts SysMatFileGen_Input<ThisAppTraits>::options = {
      {}, {},{}, {"Simulation_Settings.ini"},{}};
