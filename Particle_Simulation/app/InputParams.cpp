#include "InputParams.h"

#include <MyCEL/basics/GlobalParameters.h>
#include <MyCEL/basics/Logger.h>

#include <vector>

#include <Eigen/Core>


//Paramter Includes
#include "../arch/InstructionSets.hpp"
#include "Settings/SimulationManagerSettings.h"
#include "Settings/SystemMatrixSettings.h"
#include "Properties/ParticleProperties.h"


namespace bo_opts = ::boost::program_options;
namespace fs      = ::std::filesystem;

struct opts_string
{
    std::string_view parameter_file;
    std::string_view matrix_file;
    std::string_view instruction_set;
};

constexpr const opts_string optstr{"parameter_file", "matrix_file", "instruction_set"};

InputParams<ThisAppTraits>::InputParams(int argc, char** argv) { 
    try {
        parseCmdLineOptions(argc, argv); 
    }
    catch (std::runtime_error& e)
    {
        std::puts(e.what());
    }
}

bo_opts::options_description InputParams<ThisAppTraits>::buildOptionDescriptor()
{
    bo_opts::options_description desc{"Options"};
    desc.add_options()("help", "Displays the help")(optstr.parameter_file.data(),
                                                    bo_opts::value(&options.parameter_file)->default_value({}),
                                                    "Path to a valid parameter config file")(
        optstr.matrix_file.data(), bo_opts::value(&options.matrix_file)->default_value({}), "Path to a matrix file")(
        optstr.instruction_set.data(),
        bo_opts::value(&options.arch)->default_value({MyCEL::SystemInfo::getCPUInstructionSet()}),
        "Architecture to use. Allowed values:AVX,AVX2,AVX512");
    return desc;
}

InputParams<ThisAppTraits>::CmdOpts& InputParams<ThisAppTraits>::parseCmdLineOptions(int argc, char** argv)
{
    bo_opts::variables_map vm;
    bo_opts::store(bo_opts::parse_command_line(argc, argv, optdesc), vm);
    bo_opts::notify(vm);
    if (vm.count("help") > 0) {
        displayHelp();
    }
    return options;
}

InputParams<ThisAppTraits>::AppParams InputParams<ThisAppTraits>::getAppParams()
{
    if (!options.parameter_file.empty()) {
        const auto& filepath = options.parameter_file;
        Logger::Log("Trying to load parameters from: %s \n", filepath.u8string().c_str());
        if (std::filesystem::exists(filepath)) {
            auto CFG_IN = std::make_unique<InputArchive>(filepath);
            return Archives::LoadConstructor<AppParams>::construct(*CFG_IN);
        }
    }
    Logger::Log("Using defaulted parameters for as an example! Use --parmeter_file=<somefile> to load parameters!\n");
    return getDefaultedAppParams();
}

void InputParams<ThisAppTraits>::displayHelp()
{
    ::std::cerr << "\n";
    ::std::cerr << optdesc << std::endl;
}

InputParams<ThisAppTraits>::AppParams InputParams<ThisAppTraits>::getDefaultedAppParams()
{
    //Create Parameters

    /* General Simulation Parameters*/
    constexpr std::size_t NumberOfSteps     = 10'000'000; // 1E7
    constexpr std::size_t OverSampling      = 100; // 1E3
    constexpr std::size_t NumberOfParticles = 10;
    constexpr PREC timestep                 = 5.0E-12;
    constexpr std::size_t NumberOfThreads   = 1;

    /* Particle Parameters*/
    constexpr PREC rmag        = 15E-9;//15.326E-9;
    constexpr PREC MS          = 0.45E6;//0.477464E6;
    constexpr PREC alpha       = 0.1;
    constexpr PREC gyro        = 1.76E11;
    constexpr PREC rhyd        = 80E-9;
    constexpr PREC visc        = 1e-3;
    constexpr PREC temperature = 300; //295;
    constexpr PREC anisotropy  = 50.0e3;
    const Properties::Anisotropy::Uniaxial<PREC> unianisotropy {{},anisotropy};

    using Vec3D                = Eigen::Matrix<PREC, 3, 1>;
    Vec3D ea;
    ea << 0, 0, 1;

    /* Field Parameters */
    Vec3D ampl;
    ampl << 50E-3, 50E-3, 50E-3;

    using FieldVector = Eigen::Matrix<PREC, 3, 1>;

    std::vector<FieldVector> amps{ampl};
    FieldVector freq{25E3,26E3 ,27E3 };
    FieldVector phases{1,2,3};

    /* Creating Parameters*/
    std::unique_ptr<Distribution::IDistributionHelper<double>> pDist =
        std::make_unique<Distribution::DistributionHelper<double, std::normal_distribution<double>>>(
            std::pair<double, double>{5, 1});

    Vec3D Pos;
    Pos << 0, 0, 0;

    Vec3D Dir;
    Dir << 0, 0, 1;

    Properties::Fields::Lissajous<PREC> fieldprops{ {},Pos,ampl,freq, phases };

    Properties::MagneticProperties<PREC> Mag{rmag,
                                             MS,
                                             alpha,
                                             gyro,
                                             {Properties::IAnisotropy::Anisotropy_uniaxial,
                                             unianisotropy}};
    Properties::Anisotropy::Uniaxial<PREC>::Distribution Aniso_Dist { true, ::Distribution::IDistribution::Distribution_normal, 1E-8};
    Properties::HydrodynamicProperties<PREC> Hydro{rhyd, visc};
    // Create Particle Properties
    Properties::ParticlesProperties<PREC> ParProp{temperature, Mag, Hydro};

    Settings::ParticleSimulationInitSettings<PREC> ParSimInit{false, true, true, Pos, Pos, Dir};
    Settings::ParticleSimulationSettings<PREC> ParSimSet{
        Aniso_Dist,
        true, Distribution::IDistribution::Distribution_lognormal, 0.05,
        true, Distribution::IDistribution::Distribution_lognormal, 0.025};

    //Create Particle Simulation Parameters
    Parameters::ParticleSimulationParameters<PREC> ParSimParams{ParSimSet, ParSimInit, ParProp};

    //typedef std::tuple<ParticleName, PathToFile, NumberOfParticles, ParticleSimulationParameters> ParticleInformation;
    Provider::ParticleProvider<PREC>::ParticleInformation Tuple1{"Particle1", "Example_Particle_1.ini", 100,
                                                                 ParSimParams};
    Provider::ParticleProvider<PREC>::ParticleInformation Tuple2{"Particle2", "Example_Particle_2.ini", 1000,
                                                                 ParSimParams};
    std::vector<Provider::ParticleProvider<PREC>::ParticleInformation> PARS{{Tuple1, Tuple2}};
    Provider::ParticleProvider<PREC> ParProvider{PARS, true, true};

    Settings::ResultSettings ResSet{true, 1, "Example_Results.mat", "Simulation",
                                    Settings::IResultFileType::ResultFileType_MATLAB};


    Properties::FieldProperties<PREC> FieldSet{{Properties::IField::Field_Lissajous,
                                               fieldprops}};

    Settings::SimulationSettings<PREC> SimSet{Settings::ISimulator::Simulator_AllSingle,
                                              timestep,
                                              NumberOfSteps,
                                              OverSampling,
                                              NumberOfThreads,
                                              NumberOfParticles};
    Settings::SolverSettings<PREC> SolverSet{Settings::ISolver::Solver_EulerMaruyama, -1};

    std::unique_ptr<Settings::IProblemSettings<PREC>> test{
        std::make_unique<Settings::BrownAndNeelEulerSphericalProblemSettings<PREC>>(
            Settings::BrownAndNeelEulerSphericalProblemSettings<PREC>{})};
    //std::unique_ptr< Settings::IProblemSettings<PREC> > test{ std::make_unique<Settings::BrownAndNeelProblemSettings<PREC>>(Settings::BrownAndNeelProblemSettings<PREC>{false}) };
    //std::unique_ptr< Settings::IProblemSettings<PREC> > test{ std::make_unique<Settings::NeelProblemSettings<PREC>>(Settings::NeelProblemSettings<PREC>{}) };
    Settings::SimulationManagerSettings<PREC> SimManSet{ParProvider, SimSet, SolverSet, ResSet, *test, FieldSet};
    //! Parameters created

    //Create the Archive
    const std::filesystem::path filename{"Example_Simulation_Settings.ini"};
    OutputArchive CFG_OUT{filename};

    //Write the Settings to the CFG
    CFG_OUT(SimManSet);

    //Loading Application Parameters vom Archive
    return SimManSet;
}

const bo_opts::options_description InputParams<ThisAppTraits>::optdesc = {buildOptionDescriptor()};

InputParams<ThisAppTraits>::CmdOpts InputParams<ThisAppTraits>::options = {
    {}, {}, MyCEL::SystemInfo::getCPUInstructionSet()};
