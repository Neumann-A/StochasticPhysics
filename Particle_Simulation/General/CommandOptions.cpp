#include "CommandOptions.h"

#include <vector>
#include <stdexcept>
#include <utility>
#include <memory>
#include <fmt/format.h>

#include <MyCEL/basics/GlobalParameters.h>
#include <MyCEL/basics/Logger.h>

//Paramter Includes
#include "Settings/SimulationManagerSettings.h"
#include "Settings/SystemMatrixSettings.h"

StartOptions CommandOptions<SimulationApplication::SimulationManager<PREC>>::startOptions{};
bool CommandOptions<SimulationApplication::SimulationManager<PREC>>::useSystemMatrix{ false };

void CommandOptions<SimulationApplication::SimulationManager<PREC>>::SimulationParametersCreate()
{
    //Create Parameters

    /* General Simulation Parameters*/
    constexpr std::size_t NumberOfSteps = 10'000; // 1E7
    constexpr std::size_t OverSampling = 100; // 1E3
    constexpr std::size_t NumberOfParticles = 10;
    constexpr PREC timestep = 1.0E-11;
    constexpr std::size_t NumberOfThreads = 1;

    /* Particle Parameters*/
    constexpr PREC rmag = 15.326E-9;
    constexpr PREC MS = 0.477464E6;
    constexpr PREC alpha = 0.1;
    constexpr PREC gyro = 1.76E11;
    constexpr PREC rhyd = 80E-9;
    constexpr PREC visc = 1e-3;
    constexpr PREC temperature = 295;
    constexpr PREC anisotropy = 50.0e3;
    using Vec3D = Eigen::Matrix<PREC, 3, 1>;
    Vec3D ea {0.0, 0.0, 1.0};

    /* Field Parameters */
    Vec3D ampl {20E-3, 0.0, 0.0};

    using FieldVector = Eigen::Matrix<PREC, 3, 1>;

    std::vector<FieldVector> amps{ ampl };
    FieldVector freq{ 25E3,26E3 ,27E3 };
    FieldVector phases{ 1,2,3 };

    /* Creating Parameters*/
    std::unique_ptr<Distribution::IDistributionHelper<double>> pDist = std::make_unique<Distribution::DistributionHelper<double, std::normal_distribution<double>>>(std::pair<double, double>{5, 1});

    Vec3D Pos {0.0, 0.0, 0.0};
    Vec3D Dir {0.0, 0.0, 0.0};

    Properties::MagneticProperties<PREC> Mag{ rmag,MS,alpha,gyro,{Properties::IAnisotropy::Anisotropy_uniaxial, Properties::Anisotropy::Uniaxial<PREC>{ {}, anisotropy } }};
    Properties::HydrodynamicProperties<PREC> Hydro{ rhyd, visc };
    // Create Particle Properties
    Properties::ParticlesProperties<PREC> ParProp{ temperature, Mag, Hydro };

    Settings::ParticleSimulationInitSettings<PREC> ParSimInit{ false, true, true, Pos, Pos,  Dir };
    Properties::Anisotropy::Uniaxial<PREC>::Distribution Aniso_Dist { true, ::Distribution::IDistribution::Distribution_lognormal, 0.0001};
    Settings::ParticleSimulationSettings<PREC>     ParSimSet{ Aniso_Dist ,true,Distribution::IDistribution::Distribution_lognormal,0.05, true, Distribution::IDistribution::Distribution_lognormal ,0.025 };

    //Create Particle Simulation Parameters
    Parameters::ParticleSimulationParameters<PREC> ParSimParams{ ParSimSet,ParSimInit,ParProp };

    //typedef std::tuple<ParticleName, PathToFile, NumberOfParticles, ParticleSimulationParameters> ParticleInformation;
    Provider::ParticleProvider<PREC>::ParticleInformation Tuple1{ "Particle1","Par1.ini",100,    ParSimParams };
    Provider::ParticleProvider<PREC>::ParticleInformation Tuple2{ "Particle2","Par2.ini",1000,  ParSimParams };
    std::vector<Provider::ParticleProvider<PREC>::ParticleInformation> PARS{ { Tuple1,Tuple2 } };
    Provider::ParticleProvider<PREC> ParProvider{ PARS, true, true };

    Settings::ResultSettings ResSet{
        true, 1, "Results.mat", "SingleResults.mat", "Simulation", Settings::IResultFileType::ResultFileType_MATLAB};
    

    Properties::Fields::Lissajous<PREC> fieldprops{ {},Pos,ampl,freq, phases };
    Properties::FieldProperties<PREC> FieldSet{{Properties::IField::Field_Lissajous,
                                               fieldprops}};

    Settings::SimulationSettings<PREC>    SimSet{ Settings::ISimulator::Simulator_AllSingle,timestep,NumberOfSteps,OverSampling,NumberOfThreads,NumberOfParticles };
    Settings::SolverSettings<PREC>        SolverSet{ Settings::ISolver::Solver_EulerMaruyama,-1 };

    //std::unique_ptr< Settings::IProblemSettings<PREC> > test{ std::make_unique<Settings::BrownAndNeelEulerSphericalProblemSettings<PREC>>(Settings::BrownAndNeelEulerSphericalProblemSettings<PREC>{}) };
    //std::unique_ptr< Settings::IProblemSettings<PREC> > test{ std::make_unique<Settings::BrownAndNeelProblemSettings<PREC>>(Settings::BrownAndNeelProblemSettings<PREC>{false}) };
    //std::unique_ptr< Settings::IProblemSettings<PREC> > test{ std::make_unique<Settings::NeelProblemSettings<PREC>>(Settings::NeelProblemSettings<PREC>{}) };
    Settings::SimulationManagerSettings<PREC> SimManSet{ParProvider,
                                                        SimSet,
                                                        SolverSet,
                                                        ResSet,
                                                        {Settings::IProblem::Problem_BrownAndNeelEulerSpherical,
                                                         Settings::BrownAndNeelEulerSphericalProblemSettings<PREC>{}},
                                                        FieldSet};
    //! Parameters created

    //Create the Archive
    const std::filesystem::path filename{ "Example_Simulation_Settings.ini" };
    {
        SerAr::AllFileOutputArchiveWrapper archive(filename,SerAr::ArchiveOutputMode::Overwrite);
        archive(SimManSet);
    }

    //Make Output to Input!
    input_archive() = std::make_unique<InputArchive>(filename);
}
void CommandOptions<SimulationApplication::SimulationManager<PREC>>::SimulationParametersLoad(std::string filestr)
{
    std::filesystem::path filename{filestr};
    Logger::Log("Parameterfile to load: " + filename.string() + '\n');
    if (std::filesystem::exists(filename)) {
        input_archive() = std::make_unique<InputArchive>(filename);
    }
    else {
        Logger::Log("File: " + filename.string() + " could not be found!");
        std::exit(-1);
    }
}
void CommandOptions<SimulationApplication::SimulationManager<PREC>>::SimulationParametersRegister()
{
    startOptions.registerOption("SimulationParameters", "^-parfile:",
        &CommandOptions<Application>::SimulationParametersLoad,
        &CommandOptions<Application>::SimulationParametersCreate);
}

void CommandOptions<SimulationApplication::SimulationManager<PREC>>::HelpLoad(std::string)
{
    startOptions.printOptions();
}
void CommandOptions<SimulationApplication::SimulationManager<PREC>>::RegisterHelp()
{
    std::string comment{ "Displays this help text with all commands" };
    startOptions.registerOption("Help", "^-help",
        &CommandOptions<Application>::HelpLoad,
        nullptr, comment);
}

void CommandOptions<SimulationApplication::SimulationManager<PREC>>::RegisterAll()
{
    RegisterHelp();
    SimulationParametersRegister();
    SystemMatrixParametersRegister();
}

void CommandOptions<SimulationApplication::SimulationManager<PREC>>::SystemMatrixParametersLoad(std::string filestr)
{
    std::filesystem::path filename{filestr};
    Logger::Log("Systemmatrixfile to load: " + filename.string() + '\n');
        if (std::filesystem::exists(filename)) {
        input_archive_sysmat() = std::make_unique<InputArchive>(filename);
    }
    else {
        Logger::Log("File: " + filename.string() + " could not be found!");
        std::exit(-1);
    }
    useSystemMatrix = true;
}
void CommandOptions<SimulationApplication::SimulationManager<PREC>>::SystemMatrixParametersCreate()
{
    Eigen::Matrix<PREC, 3, 1>           startfield {-0.002, -0.002, 0.0};
    Eigen::Matrix<PREC, 3, 1>           stopfield {0.002, 0.002, 0.0};
    Eigen::Matrix<std::size_t, 3, 1>    slices{2, 2, 1};

    Settings::SystemMatrixSettings<PREC> sysMatSet{ startfield, stopfield, slices };
    //Create the Archive
    const std::filesystem::path filename{ "Example_Systemmatrix_Settings.ini" };
    const auto archive_enum = SerAr::getArchiveEnumByExtension(filename.extension().string());
    if(!archive_enum) {
        const auto error = fmt::format("No archive known to support the file extension: '{}'", filename.extension().string() );
        throw std::runtime_error{error.c_str()};
    }
    SerAr::AllFileOutputArchiveWrapper archive(filename,SerAr::ArchiveOutputMode::Overwrite);
    archive(Archives::createNamedValue(sysMatSet.getSectionName(),sysMatSet));
    useSystemMatrix = false;
}

void CommandOptions<SimulationApplication::SimulationManager<PREC>>::SystemMatrixParametersRegister()
{
    startOptions.registerOption("SystemMatrixParameters", "^-sysmatrix:",
        &CommandOptions<Application>::SystemMatrixParametersLoad,
        &CommandOptions<Application>::SystemMatrixParametersCreate);
}

