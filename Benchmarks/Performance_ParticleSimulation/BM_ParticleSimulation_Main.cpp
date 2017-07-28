///-------------------------------------------------------------------------------------------------
// file:	Performance_PartikelSimulation.cpp
//
// summary:	Test the different part of the particle simulations to see where the bottleneck is 
///-------------------------------------------------------------------------------------------------

#include "stdafx.h"

#include "BM_NoiseVector_F.h"
#include "BM_NeelProblem_F.h"

int main(int argc, char** argv)
{
	//for (auto& test_input : { /* ... */ })
	//	benchmark::RegisterBenchmark(test_input.name(), BM_test, test_input);
//Could also use the Makro: BENCHMARK_MAIN()                   
    ::benchmark::Initialize(&argc, argv); 
    ::benchmark::RunSpecifiedBenchmarks();


//	//TODO: Add informations to global parameters for easier access; 
//	Logger::Log("Buildtime: " + std::string{ __TIMESTAMP__ } +"\tBuilddate: " + __DATE__);
//
//	std::vector<std::tuple<std::string, std::size_t >> TimingResults;
//
//#ifdef _USEBOOST_
//	Logger::Log("Using BOOST!");
//#endif
//#ifdef _USE_PCG_RANDOM_
//	Logger::Log("Using PCG Random!");
//#endif
//
//	Logger::Log("Used Instructions Sets in Eigen:" + std::string{ Eigen::SimdInstructionSetsInUse() });
//
//	Logger::Log("Eigen_Comp_MSCV: " + std::to_string(EIGEN_COMP_MSVC));
//	Logger::Log("Eigen_Comp_LLVM: " + std::to_string(EIGEN_COMP_LLVM));
//	Logger::Log("Eigen_Comp_MINGW: " + std::to_string(EIGEN_COMP_MINGW));
//
//	const std::experimental::filesystem::path pathToExe{ argv[0] };
//	const std::experimental::filesystem::path path{ pathToExe.parent_path() };
//
//	Logger::Log("Path I run in: " + path.string());
//	GlobalParameters::Path = path;
//
//	Logger::Log("*********************Starting Application*********************");
//	///*****************************************************************************************************************************************************/
//	///* BEGIN Simulation Parameters*/
//	/* General Simulation Parameters*/
//	constexpr uint64_t NumberOfSteps = 10'000'000; // 1E7
//	constexpr uint64_t OverSampling = 100; // 1E3
//	constexpr uint64_t NumberOfParticles = 10;
//	constexpr PREC timestep = 1.0E-11;
//	constexpr uint64_t NumberOfThreads = 1;
//
//	/* Particle Parameters*/
//	constexpr PREC rmag = 15.326E-9;
//	constexpr PREC MS = 0.477464E6;
//	constexpr PREC alpha = 0.1;
//	constexpr PREC gyro = 1.76E11;
//	constexpr PREC rhyd = 1E-9;
//	constexpr PREC visc = 1e-3;
//	constexpr PREC temperature = 295;
//	constexpr PREC anisotropy = 50.0e3;
//	Eigen::Matrix<PREC, 3, 1> ea;
//	ea << 0, 0, 1;
//
//	/* Field Parameters */
//	Eigen::Matrix<PREC, 3, 1> ampl;
//	ampl << 200E-3, 0, 0;
//	std::vector<Eigen::Matrix<PREC, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<PREC, 3, 1>>> amps{ ampl };
//	std::vector<PREC> freq{ 25E3 };
//	std::vector<PREC> phases{ 0 };
//
//	/* Creating Parameters*/
//	std::unique_ptr<Distribution::IDistributionHelper<double>> pDist = std::make_unique<Distribution::DistributionHelper<double, std::normal_distribution<double>>>(std::pair<double, double>{5, 1});
//
//	Eigen::Matrix<PREC, 3, 1> Pos;
//	Pos << 0, 0, 0;
//
//	Eigen::Matrix<PREC, 3, 1> Dir;
//	Dir << 1, 0, 0;
//
//	Properties::MagneticProperties<PREC> Mag{ rmag,MS,alpha,gyro,Properties::IAnisotropy::Anisotropy_uniaxial, std::vector<PREC>{ anisotropy } };
//	Properties::HydrodynamicProperties<PREC> Hydro{ rhyd, visc };
//	// Create Particle Properties
//	Properties::ParticlesProperties<PREC> ParProp{ temperature, Mag, Hydro };
//
//	Settings::ParticleSimulationInitSettings<PREC> ParSimInit{ false, false, false, Pos, Pos,  Dir };
//	Settings::ParticleSimulationSettings<PREC>	 ParSimSet{ true, Distribution::IDistribution::Distribution_delta, std::vector<PREC>{ 0.5 } ,true,Distribution::IDistribution::Distribution_delta,0.05, true, Distribution::IDistribution::Distribution_delta ,0.025 };
//
//	//Create Particle Simulation Parameters
//	Parameters::ParticleSimulationParameters<PREC> ParSimParams{ ParSimSet,ParSimInit,ParProp };
//
//	//typedef std::tuple<ParticleName, PathToFile, NumberOfParticles, ParticleSimulationParameters> ParticleInformation;
//	Provider::ParticleProvider<PREC>::ParticleInformation Tuple1{ "Particle1","Par1.ini",100,	ParSimParams };
//	Provider::ParticleProvider<PREC>::ParticleInformation Tuple2{ "Particle2","Par2.ini",1000,  ParSimParams };
//	std::vector<Provider::ParticleProvider<PREC>::ParticleInformation> PARS{ { Tuple1,Tuple2 } };
//	Provider::ParticleProvider<PREC> ParProvider{ PARS, true, true };
//
//	Settings::ResultSettings ResSet{ true, 1, "Results.mat", "Simulation" };
//	Properties::FieldProperties<PREC> FieldSet{ Properties::IField::Field_Sinusoidal,std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>({ Pos,ampl }),std::vector<double>(1,25E3),std::vector<double>(1,0) };
//
//	Settings::SimulationSettings<PREC>	SimSet{ Settings::ISimulator::Simulator_AllSingle,timestep,NumberOfSteps,OverSampling,NumberOfThreads,NumberOfParticles };
//	Settings::SolverSettings<PREC>		SolverSet1{ Settings::ISolver::Solver_EulerMaruyama,-1 };
//	Settings::SolverSettings<PREC>		SolverSet2{ Settings::ISolver::Solver_ExplicitStrong1_0, 4 };
//
//	std::unique_ptr< Settings::IProblemSettings<PREC> > BrownAndNeel{ std::make_unique<Settings::BrownAndNeelProblemSettings<PREC>>(Settings::BrownAndNeelProblemSettings<PREC>{false}) };
//	std::unique_ptr< Settings::IProblemSettings<PREC> > Neel{ std::make_unique<Settings::NeelProblemSettings<PREC>>(Settings::NeelProblemSettings<PREC>{}) };
//	
//	Settings::SimulationManagerSettings<PREC> SimManSetBrownAndNeel{ ParProvider, SimSet, SolverSet1,ResSet,*BrownAndNeel,FieldSet };
//	Settings::SimulationManagerSettings<PREC> SimManSetNeel{ ParProvider, SimSet, SolverSet1,ResSet,*Neel,FieldSet };
//	///* END Simulation Parameters*/
//	///*****************************************************************************************************************************************************/
//
//	Problems::BrownAndNeelRelaxation<PREC,UniaxialAnisotropy<PREC>> Problem1{ (dynamic_cast<Settings::BrownAndNeelProblemSettings<PREC> &>(*BrownAndNeel)) ,ParSimParams.getNewParticleProperties(),ParSimParams.getParticleSimulationInitialization() };
//	Problems::NeelRelaxation<PREC, UniaxialAnisotropy<PREC>> Problem2{ (dynamic_cast<Settings::NeelProblemSettings<PREC> &>(*Neel)) ,ParSimParams.getNewParticleProperties(),ParSimParams.getParticleSimulationInitialization() };
//
//#ifdef SIMTEST
//	SingleParticleSimulator<EulerMaruyama<Problems::BrownAndNeelRelaxation<PREC, UniaxialAnisotropy<PREC>>, NOISEFIELD>, SinusoidalField<PREC>> Sim{ Problem1,FieldSet,timestep };
//	SingleParticleSimulator<EulerMaruyama<Problems::NeelRelaxation<PREC, UniaxialAnisotropy<PREC>>, NoiseField<PREC, 3, pcg64_k1024_fast>>, SinusoidalField<PREC>> Sim2{ Problem2,FieldSet,timestep };
//	Sim.doSimulation(NumberOfSteps, OverSampling);
//	Sim2.doSimulation(NumberOfSteps, OverSampling);
//#endif
//#ifdef NOISETEST
//	/*****************************************************************************************************************************************************/
//	/* BEGIN NoiseField Test*/
//
//	DOUBLENOISEMATRIX DoubleNoise{ 1'000'000,timestep };
//	{
//		NOISEFIELD noise{ 1'000'000, timestep }; // 1000000 is needed to initialize std::mt19937 correctly
//		auto nv = noise.getField();
//		//auto tmpDN = DoubleNoise.getNoiseMatrix(nv);
//		std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds> Noise_begin, Noise_finished;
//		Noise_begin = std::chrono::high_resolution_clock::now();
//		for (auto i = NumberOfSteps; --i; )
//		{
//			nv = noise.getField();
//			//tmpDN = DoubleNoise.getNoiseMatrix(nv);
//		}
//		Noise_finished = std::chrono::high_resolution_clock::now();
//		std::cout << "It took " << (Noise_finished - Noise_begin).count() / 1E9 << " s to generate " << NumberOfSteps << " Noisefields." << std::endl;
//		nv;
//	}
//	{
//		NoiseField<PREC, 3, pcg64_k1024_fast> noise{ 1'000'000, timestep }; // 1000000 is needed to initialize std::mt19937 correctly
//		auto nv = noise.getField();
//		//auto tmpDN = DoubleNoise.getNoiseMatrix(nv);
//		std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds> Noise_begin, Noise_finished;
//		Noise_begin = std::chrono::high_resolution_clock::now();
//		for (auto i = NumberOfSteps; --i; )
//		{
//			nv = noise.getField();
//			//tmpDN = DoubleNoise.getNoiseMatrix(nv);
//		}
//		Noise_finished = std::chrono::high_resolution_clock::now();
//		std::cout << "It took " << (Noise_finished - Noise_begin).count() / 1E9 << " s to generate " << NumberOfSteps << " Noisefields." << std::endl;
//		nv;
//	}
//	//tmpDN;
//	//NoiseField 0.1 GSteps = 7.9s
//	//DoubleNoise with p=-1 and 0.1 GSteps = 16s (includes time for 0.1GSteps NoiseField)
//	//DoubleNoise with p=0 and 0.1 GSteps =  60s (includes time for 0.1GSteps NoiseField)
//	//DoubleNoise with p=1 and 0.1 GSteps = 151s (includes time for 0.1GSteps NoiseField)
//	//DoubleNoise with p=2 and 0.1 GSteps = 234s (includes time for 0.1GSteps NoiseField)
//	//DoubleNoise with p=3 and 0.1 GSteps = 338s (includes time for 0.1GSteps NoiseField)
//	//DoubleNoise with p=4 and 0.1 GSteps = 422s (includes time for 0.1GSteps NoiseField)
//	//DoubleNoise with p=5 and 0.1 GSteps = 510s (includes time for 0.1GSteps NoiseField)
//	//Dont think i can make it faster
//	/* END NoiseField Test*/
//	/*****************************************************************************************************************************************************/
//	/*****************************************************************************************************************************************************/
//
//#endif
//	/*****************************************************************************************************************************************************/
//
//	/* END Problem Definition*/
//
//	/*****************************************************************************************************************************************************/
//
//	/* BEGIN Solver Setup*/
//
//
//	/* END Solver Setup*/
//
//	/*****************************************************************************************************************************************************/
//
//	/* BEGIN Solving*/
//	//SOLVERMETHOD SolMethod{ sProbF, timestep };
//
//	///*Initialise Start*/
//	//Eigen::Matrix<PREC, 3, 1> t1;
//	//Eigen::Matrix<PREC, 3, 1> t2;
//	//Eigen::Matrix<PREC, 6, 1> t;
//	//t1 << 5, 3, 6;
//	//t2 << 8, 20, 3;
//	//t1.normalize();
//	//t2.normalize();
//	//t << t1, t2;
//
//	//Eigen::Matrix<PREC, 6, 1> t3;
//	//t3 << 5,9,6,1,1,1;
//	//t3.head<3>().normalize();
//	//t3.tail<3>().normalize();
//
//	//std::cout << t3 << std::endl;
//
//	//std::vector<Eigen::Matrix<PREC,6 , 1>> resvec;
//	//resvec.resize(NumberOfSteps);
//
//	//std::cout << "Starting Simulation" << std::endl;
//
//	//std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds> solver_begin, solver_finished;
//	//solver_begin = std::chrono::high_resolution_clock::now();
//	//for (long i = 0; i < NumberOfSteps; ++i)
//	//{
//	//	t3 = SolMethod.getResultNextFixedTimestep(v, field.getField(i*timestep));
//
//	//	
//	//	t3.head<3>().normalize();
//	//	t3.tail<3>().normalize();
//
//	//	resvec[i] = t3;
//
//	//}
//	//solver_finished = std::chrono::high_resolution_clock::now();
//	//std::cout << "It took " << (solver_finished - solver_begin).count() / 1E9 << " s to simulate " << NumberOfSteps <<" steps. This corresponds to a time of " << NumberOfSteps * timestep << " s" << std::endl << std::endl;
//
//	/* END Solving*/
    return 0;
}

