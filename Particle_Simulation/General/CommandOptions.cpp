#include "CommandOptions.h"

#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "basics/GlobalParameters.h"
#include "basics/Logger.h"

StartOptions CommandOptions<SimulationApplication::SimulationManager<PREC>>::_StartOptions{};
std::unique_ptr<CommandOptions<SimulationApplication::SimulationManager<PREC>>::InputArchive> CommandOptions<SimulationApplication::SimulationManager<PREC>>::_pCFG_Input{ nullptr };
bool CommandOptions<SimulationApplication::SimulationManager<PREC>>::useSystemMatrix{ false };
std::unique_ptr<CommandOptions<SimulationApplication::SimulationManager<PREC>>::InputArchive> CommandOptions<SimulationApplication::SimulationManager<PREC>>::_pCFG_InputSysMat{ nullptr };

void CommandOptions<SimulationApplication::SimulationManager<PREC>>::SimulationParametersCreate()
{
	//Create Parameters

	/* General Simulation Parameters*/
	constexpr uint64_t NumberOfSteps = 10'000'000; // 1E7
	constexpr uint64_t OverSampling = 100; // 1E3
	constexpr uint64_t NumberOfParticles = 10;
	constexpr PREC timestep = 1.0E-11;
	constexpr uint64_t NumberOfThreads = 1;

	/* Particle Parameters*/
	constexpr PREC rmag = 15.326E-9;
	constexpr PREC MS = 0.477464E6;
	constexpr PREC alpha = 0.1;
	constexpr PREC gyro = 1.76E11;
	constexpr PREC rhyd = 80E-9;
	constexpr PREC visc = 1e-3;
	constexpr PREC temperature = 295;
	constexpr PREC anisotropy = 50.0e3;
	Eigen::Matrix<PREC, 3, 1> ea;
	ea << 0, 0, 1;

	/* Field Parameters */
	Eigen::Matrix<PREC, 3, 1> ampl;
	ampl << 200E-3, 0, 0;
	std::vector<Eigen::Matrix<PREC, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<PREC, 3, 1>>> amps{ ampl };
	std::vector<PREC> freq{ 25E3 };
	std::vector<PREC> phases{ 0 };

	/* Creating Parameters*/
	std::unique_ptr<Distribution::IDistributionHelper<double>> pDist = std::make_unique<Distribution::DistributionHelper<double, std::normal_distribution<double>>>(std::pair<double, double>{5, 1});

	Eigen::Matrix<PREC, 3, 1> Pos;
	Pos << 0, 0, 0;

	Eigen::Matrix<PREC, 3, 1> Dir;
	Dir << 0, 0, 1;

	Properties::MagneticProperties<PREC> Mag{ rmag,MS,alpha,gyro,Properties::IAnisotropy::Anisotropy_uniaxial, std::vector<PREC>{ anisotropy } };
	Properties::HydrodynamicProperties<PREC> Hydro{ rhyd, visc };
	// Create Particle Properties
	Properties::ParticlesProperties<PREC> ParProp{ temperature, Mag, Hydro };

	Settings::ParticleSimulationInitSettings<PREC> ParSimInit{ false, true, true, Pos, Pos,  Dir };
	Settings::ParticleSimulationSettings<PREC>	 ParSimSet{ true, Distribution::IDistribution::Distribution_lognormal, std::vector<PREC>{ 0.5 } ,true,Distribution::IDistribution::Distribution_lognormal,0.05, true, Distribution::IDistribution::Distribution_lognormal ,0.025 };

	//Create Particle Simulation Parameters
	Parameters::ParticleSimulationParameters<PREC> ParSimParams{ ParSimSet,ParSimInit,ParProp };

	//typedef std::tuple<ParticleName, PathToFile, NumberOfParticles, ParticleSimulationParameters> ParticleInformation;
	Provider::ParticleProvider<PREC>::ParticleInformation Tuple1{ "Particle1","Par1.ini",100,	ParSimParams };
	Provider::ParticleProvider<PREC>::ParticleInformation Tuple2{ "Particle2","Par2.ini",1000,  ParSimParams };
	std::vector<Provider::ParticleProvider<PREC>::ParticleInformation> PARS{ { Tuple1,Tuple2 } };
	Provider::ParticleProvider<PREC> ParProvider{ PARS, true, true };

	Settings::ResultSettings ResSet{ true, 1, "Results.mat", "Simulation" };
	Properties::FieldProperties<PREC> FieldSet{ Properties::IField::Field_Sinusoidal,std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>({ Pos,ampl }),std::vector<double>(1,25E3),std::vector<double>(1,0) };

	Settings::SimulationSettings<PREC>	SimSet{ Settings::ISimulator::Simulator_AllSingle,timestep,NumberOfSteps,OverSampling,NumberOfThreads,NumberOfParticles };
	Settings::SolverSettings<PREC>		SolverSet{ Settings::ISolver::Solver_EulerMaruyama,-1 };

	std::unique_ptr< Settings::IProblemSettings<PREC> > test{ std::make_unique<Settings::BrownAndNeelProblemSettings<PREC>>(Settings::BrownAndNeelProblemSettings<PREC>{false}) };
	//std::unique_ptr< Settings::IProblemSettings<PREC> > test{ std::make_unique<Settings::NeelProblemSettings<PREC>>(Settings::NeelProblemSettings<PREC>{}) };
	Settings::SimulationManagerSettings<PREC> SimManSet{ ParProvider, SimSet, SolverSet,ResSet,*test,FieldSet };
	//! Parameters created

	//Create the Archive
	const std::experimental::filesystem::path filename{ "Example_Simulation_Settings.ini" };
	OutputArchive CFG_OUT{ filename };

	//Write the Settings to the CFG
	CFG_OUT(SimManSet);

	//Make Output to Input!
	_pCFG_Input = std::make_unique<InputArchive>(CFG_OUT.getStorage());
};
void CommandOptions<SimulationApplication::SimulationManager<PREC>>::SimulationParametersLoad(std::string filename)
{
	Logger::Log("Parameterfile to load: " + filename);
	_pCFG_Input = std::make_unique<InputArchive>(filename);
};
void CommandOptions<SimulationApplication::SimulationManager<PREC>>::SimulationParametersRegister()
{
	_StartOptions.registerOption("SimulationParameters", "^-parfile:",
		&CommandOptions<Application>::SimulationParametersLoad,
		&CommandOptions<Application>::SimulationParametersCreate);
};



void CommandOptions<SimulationApplication::SimulationManager<PREC>>::HelpLoad(std::string)
{
	_StartOptions.printOptions();
};
void CommandOptions<SimulationApplication::SimulationManager<PREC>>::RegisterHelp()
{
	std::string comment{ "Displays this help text with all commands" };
	_StartOptions.registerOption("Help", "^-help",
		&CommandOptions<Application>::HelpLoad,
		nullptr, comment);
};

void CommandOptions<SimulationApplication::SimulationManager<PREC>>::RegisterAll()
{
	RegisterHelp();
	SimulationParametersRegister();
	SystemMatrixParametersRegister();
};

void CommandOptions<SimulationApplication::SimulationManager<PREC>>::SystemMatrixParametersLoad(std::string filename)
{
	Logger::Log("Systemmatrixfile to load: " + filename);
	_pCFG_InputSysMat = std::make_unique<InputArchive>(filename);
	useSystemMatrix = true;
};
void CommandOptions<SimulationApplication::SimulationManager<PREC>>::SystemMatrixParametersCreate()
{
	Eigen::Matrix<PREC, 3, 1>					startfield;
	Eigen::Matrix<PREC, 3, 1>					stopfield;
	Eigen::Matrix<std::size_t, 3, 1>			slices;
	startfield << -0.002, -0.002, 0;
	stopfield << 0.002, 0.002, 0;
	slices << 2, 2, 1;
	Settings::SystemMatrixSettings<PREC> sysMatSet{ startfield, stopfield, slices };
	//Create the Archive
	const std::experimental::filesystem::path filename{ "Example_Systemmatrix_Settings.ini" };
	//Funktioniert nicht --> Ungenannter Fehler waehrend der Laufzeit.
	OutputArchive CFG_OUT{ filename };
	//Ohne Section name kann nicht gespeichert werden! (Funktioniert beim SimManager da er ein compound type ist!)
	CFG_OUT(Archives::createNamedValue(sysMatSet.getSectionName(),sysMatSet));

	useSystemMatrix = false;
};

void CommandOptions<SimulationApplication::SimulationManager<PREC>>::SystemMatrixParametersRegister()
{
	_StartOptions.registerOption("SystemMatrixParameters", "^-sysmatrix:",
		&CommandOptions<Application>::SystemMatrixParametersLoad,
		&CommandOptions<Application>::SystemMatrixParametersCreate);
};

