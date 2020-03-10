///---------------------------------------------------------------------------------------------------
// file:		SimulationManager.h
//
// summary: 	Declares the simulation manager v 2 class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 25.06.2016

#ifndef INC_SimulationManager_H
#define INC_SimulationManager_H
///---------------------------------------------------------------------------------------------------
#pragma once



//General Includes
#include <cstdlib>
#include <cmath>
#include <memory>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <exception>
#include <type_traits>

#include "basics/BasicMacros.h"
#include "basics/Logger.h"
#include "basics/ThreadManager.h"

#include "SimulationManagerTraits.h"

//Simulation Includes
//#include "ISingleParticleSimulator.h"
#include "SingleParticleSimulator.h"

#include "Results/ResultManagerFactory.h"
#include "Settings/SystemMatrix_SimulationManagerSettings_Factory.h"

// Anisotropy Includes
#include "Problems/Anisotropy/UniaxialAnisotropy.h"
#include "Problems/Anisotropy/CubicAnisotropy.h"

//Field Includes
//#include "Fields/SinusoidalField.h"
//#include "Fields/LissajousField.h"



//Problem Includes
#include "Problems/Problems.h"

//Solver includes
#include "SDEFramework/Solver/SDESolvers.h"

//Include all Selectors;
#include "Selectors/AllSelectors.h"



////Task this object has to do;
////Define the simulation which should run
////Create threads and manage threads to run simulations 
////Create Simulators and get the result and combine them
namespace SimulationApplication
{
	template<typename prec>
	class SimulationManager
	{
        static_assert(std::is_floating_point_v<prec>, "Template type must be a floating point type!");
    private:
        using ThisClass = SimulationManager<prec>;
		//TODO: Refactor this class because it is doing to much. It is starting every simulation and handling/saving the results; Maybe move that part out of the class.
	public:
        using Traits = SimulationApplication::SimulationManagerTraits<ThisClass>;
		using StartInputArchive = typename Traits::StartInputArchive;			// Defines where to read the configs from
		using StartOutputArchive = typename Traits::StartOutputArchive;			// Defines where to write the configs to
		
		using ResultOutputArchive = typename Traits::ResultOutputArchive;

		//TODO:: Create Typelist for all possible OutputArchives
		using PossibleConcreteResultArchives = std::false_type;

		using Parameters = typename Traits::Parameters;

		static prec ProgressModifier;
		static prec ProgressFactor;
		static std::atomic<std::size_t> ProgressCache; // In Percentage: 0 - 100 
        static const bool ishpcjob;

	private:
		using ResultManagerFactory = typename Results::ResultManagerFactory;
		
		DISALLOW_COPY_AND_ASSIGN(SimulationManager)

		// This Class is Owner! (Pointer only needed for polymorphic storage)
		std::unique_ptr<Results::ISimulationResultManager<prec>>			_ResultManager;

		Settings::SimulationManagerSettings<prec>		_SimManagerSettings;

		// Since the ThreadManager is dependent on _SimParameters in the constructors intialiser list it has to be declared after it! See standard 12.6.2.5
		ThreadManager									_ThreadManager;

		std::atomic<uint64_t>				_NumberOfStartedSimulations{ 0 };
		std::atomic<uint64_t>				_NumberOfRunningSimulations{ 0 };
		std::atomic<uint64_t>				_NumberOfFinishedSimulations{ 0 };

		std::atomic<bool>					_earlyAbort{ false };			//! Flag to abort the simulation early
		std::atomic<bool>					_hasStarted{ false };			//! Flag to show if the simulator has started
		std::atomic<bool>					_Finished{ false };				//! Flag that the simulation finished

		std::mutex							_ManagerMutex;					//! Mutex for the Manager	
		std::condition_variable				_ManagerConditionVariable;		//! ResultCondition Variable!

		std::mutex							_TaskCreationMutex;				//! Mutex used to synchronize task creation

		template <typename Simulator>
		void singleSimulationTask(typename Simulator::Problem prob, typename Simulator::Field field, prec timestep, const typename Simulator::ProblemParameters &params)
		{
			if (_earlyAbort)
			{
				return;
			}
			else
			{
				++_NumberOfStartedSimulations;

				Simulator Sim { std::move(prob), std::move(field), std::move(timestep), _SimManagerSettings.getSolverSettings(),params };

				const auto SimSet = _SimManagerSettings.getSimulationSettings();
				Sim.doSimulation(SimSet.getNumberOfSteps(), SimSet.getOverSampling());

				addNewResult<typename Simulator::Traits::SingleResultType>(Sim.getSimulationResult());

				finishSimulationTask<Simulator>(Sim);

				//Check if we did something wrong and started too many Simulations
				if (_NumberOfStartedSimulations >= SimSet.getNumberOfSimulations() || _earlyAbort)
				{
					if (_NumberOfStartedSimulations != _NumberOfFinishedSimulations)
						return; // Wait until all started Simulations have finished

					if (!_Finished.exchange(true))
					{
						finalizeSimulation();
						Simulator::resetClassStatics();
						Logger::Log("Simulation Manager: Simulation finished!\n");
					}
					return;
				}
			}

		};

		template <typename Simulator>
		void quickabortfinish(Simulator&&)
		{
			finalizeSimulation();
			Logger::Log("Simulation Manager: Quick Abort finished!\n");
		}

		// Returns the simulations result to the Simulation Manager
		template <typename SimulatorResult>
		void addNewResult(SimulatorResult&& singleResult)
		{
			std::lock_guard<std::mutex> lock(_ManagerMutex);
			_ResultManager->addSingleSimulationResult(std::forward<SimulatorResult>(singleResult));
		};

		template <typename Simulator>
		void finishSimulationTask(Simulator&)
		{            
			++_NumberOfFinishedSimulations;

			//HACK: Fast hack to get the progress of the job in the HPC job manager. Should be replaced with something more sophistatced.
			//	    Spams a little in std::cerr due to the multithreaded nature of the programm.  
			const std::size_t progress = static_cast<std::size_t>(std::round((static_cast<double>(_NumberOfFinishedSimulations) / 
				static_cast<double>(_SimManagerSettings.getSimulationSettings().getNumberOfSimulations()))*100.0*ProgressFactor + ProgressModifier*100.0));
			auto tmp = ProgressCache.load();
			
            if (tmp != progress && ProgressCache.compare_exchange_weak(tmp, progress)) //To avoid spamming the system from a lot of threads!
            {
                ProgressCache.store(progress); //Store the new value in the cache
                if (ishpcjob)
                {
                    std::string str{ "Job modify %CCP_JOBID% /progress:" };
                    str += std::to_string(progress) +'\n';
                    std::system(str.c_str());
                }

                constexpr const int barlength = 70;
                const auto actuallength = progress*barlength;
                std::stringstream str;
                str << '[';
                for (std::size_t i = 0; i < barlength; i++)
                {
                    const auto itmp = i * 100;
                    if (itmp < actuallength)
                        str << '=';
                    else
                        str << ' ';
                }
                str << "] " << std::to_string(progress) << "%\n";
                Logger::Log(str);
            }
		};

		void finalizeSimulation()
		{
			Logger::Log("Simulation Manager: Finsihed results of %d!\n", _NumberOfFinishedSimulations.load());
			_ResultManager->writeSimulationManagerSettings(_SimManagerSettings); //Writing Settings to file
			_ResultManager->finish();
		};

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Creates the next simulation task. </summary>
		///
		/// <typeparam name="Simulator">	Type of the simulator. </typeparam>
		///-------------------------------------------------------------------------------------------------
		template <typename Simulator>
		void createNextSimulationTask()
		{
			RuntimeFieldSelector();
		};

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Creates a single simulation task. </summary>
		///
		/// <typeparam name="Simulator">	Type of the simulator. </typeparam>
		/// <param name="prob">	The already instantiated Problem. </param>
		///-------------------------------------------------------------------------------------------------
		template <typename Simulator>
		void createSimulationTask(const typename Simulator::Problem &prob,const typename Simulator::ProblemParameters &params)
		{
			// If the Simulation is started for the first time we need to create the MeanResult object.
			// Since we are already multithreaded here we use an atomic flag to check if the simulation was already started
			if (!_hasStarted.exchange(true))
			{
				auto ptr = Results::ResultManagerFactory::template createResultManager<Simulator>(_SimManagerSettings.getResultSettings());
				_ResultManager.swap(ptr);
				_ManagerConditionVariable.notify_all();
			}

			//Create thread local copies of necessary datastructures
			const prec dt = _SimManagerSettings.getSimulationSettings().getTimestep();
			const typename Simulator::Field extfield{ _SimManagerSettings.getFieldProperties() };

			//Create the simulation task
			this->singleSimulationTask<Simulator>(prob, std::move(extfield), std::move(dt), params);
		};

	
		///*****************************************************************************************************************************************************/
		///* BEGIN Runtime Selectors; The Selectors are necessary to convert runtime enums into compile time constants. We need to have the concrete compile  
		///*						  time types so that most of the functions can be inlined. We do not want to have virtual functions calls in our simulation  
		///*						  steps. The price we pay are this very ugly switch statements. */  

		/// <summary>	Runtime field selector. Selects the external appliad Field</summary>
		
#define FIELDSWITCH(Value) \
 case Value : \
 { RuntimeSolverSelection< Value >(); break; }

		void RuntimeFieldSelector()
		{
			switch (_SimManagerSettings.getFieldProperties().getTypeOfField())
			{
			case Properties::IField::Field_undefined:
			{
				Logger::Log("Simulation Manager: Field is not defined!\n");
				break;
			}
			FIELDSWITCH(Properties::IField::Field_Zero)
			FIELDSWITCH(Properties::IField::Field_Constant)
			FIELDSWITCH(Properties::IField::Field_Sinusoidal)
			FIELDSWITCH(Properties::IField::Field_Lissajous)
			FIELDSWITCH(Properties::IField::Field_Triangular)
			default:
				Logger::Log("Simulation Manager: Field is not defined!\n");
				break;
			}
		}

#undef FIELDSWITCH



		///-------------------------------------------------------------------------------------------------
		/// <summary> Selects the correct Solver </summary>
		///
		/// <typeparam name="FieldID">	Identifier of the external Field Type. </typeparam>
		///-------------------------------------------------------------------------------------------------
		
#define SOLVERSWITCH(Value) \
 case Value : \
 { RuntimeProblemSelector<FieldID, Value >(); break; }
		
		template <Properties::IField FieldID>
		void RuntimeSolverSelection()
		{
			switch (_SimManagerSettings.getSolverSettings().getTypeOfSolver())
			{
			case Settings::ISolver::Solver_undefined: {
				Logger::Log("Simulation Manager: Solver not defined\n");
				break; }
			SOLVERSWITCH(Settings::ISolver::Solver_EulerMaruyama)
			SOLVERSWITCH(Settings::ISolver::Solver_EulerMaruyamaNormalized) 
			SOLVERSWITCH(Settings::ISolver::Solver_Implicit_Midpoint)
#ifdef WITH_GSL_SOLVERS
			SOLVERSWITCH(Settings::ISolver::Solver_Implicit_Midpoint_GSL)
			SOLVERSWITCH(Settings::ISolver::Solver_Implicit_Midpoint_GSL_Derivative_Free)
#endif
			
			//Does not work
			//SOLVERSWITCH(Settings::ISolver::Solver_Millstein)
			//SOLVERSWITCH(Settings::ISolver::Solver_Heun_Strong) //Works. But name may be misleading
			//SOLVERSWITCH(Settings::ISolver::Solver_ExplicitStrong1_0) //Seems to work not really better than EulerMaruyama
			//SOLVERSWITCH(Settings::ISolver::Solver_Heun_NotConsistent) //Works. But not consistent
			//SOLVERSWITCH(Settings::ISolver::Solver_WeakTest) //
			default: {
				Logger::Log("Simulation Manager: Solver not defined\n");
				break; }
			}
		}

#undef SOLVERSWITCH
		///-------------------------------------------------------------------------------------------------
		/// <summary>	Selects the correct Problem at runtime </summary>
		///
		/// <typeparam name="FieldID"> 	Identifier of the external field type. </typeparam>
		/// <typeparam name="SolverID">	Identifier of the solver type. </typeparam>
		///-------------------------------------------------------------------------------------------------
#define PROBLEMSWITCH(Value) \
 case Value: \
 { buildProblemType<FieldID,SolverID,Value>();break;}
		template <Properties::IField FieldID, Settings::ISolver SolverID>
		void RuntimeProblemSelector()
		{
			// First identify the Problem
			switch (_SimManagerSettings.getProblemSettings().getProblemType())
			{
			case Settings::IProblem::Problem_undefined:
				Logger::Log("Simulation Manager: Problem not defined\n");
				break;
			PROBLEMSWITCH(Settings::IProblem::Problem_Neel)
			PROBLEMSWITCH(Settings::IProblem::Problem_NeelSpherical)
			PROBLEMSWITCH(Settings::IProblem::Problem_BrownAndNeel)
			PROBLEMSWITCH(Settings::IProblem::Problem_BrownAndNeelEulerSpherical)
			default:
				Logger::Log("Simulation Manager: Problem not defined\n");
				break;
			}
		}
#undef PROBLEMSWITCH
		///-------------------------------------------------------------------------------------------------
		/// <summary>	Builds the Type of a Magnetic Problem. </summary>
		///
		/// <typeparam name="ProblemID">   	Identifier of the Problem type </typeparam>
		/// <typeparam name="AnisotropyID">	Identifier of the Anisotropy type </typeparam>
		/// <param name="SimParams">	[in,out] PAramters of the simulation. </param>
		///
		/// <returns>	The constructed problem </returns>
		///-------------------------------------------------------------------------------------------------
		template<Settings::IProblem ProblemID, Properties::IAnisotropy AnisotropyID,
			typename SimulationParameters, typename SolverBuilder>
			std::enable_if_t<ProblemID == Settings::IProblem::Problem_Neel 
			|| ProblemID == Settings::IProblem::Problem_NeelSpherical
			|| ProblemID == Settings::IProblem::Problem_BrownAndNeelEulerSpherical>
			buildMagneticProblem(SimulationParameters &SimParams, SolverBuilder&& SolvBuilder)
		{
			std::unique_lock<std::mutex> lock(_ManagerMutex);
			auto Particle = SimParams.getNewParticleProperties();
			auto ParticleInit = SimParams.getParticleSimulationInitialization();
			lock.unlock();
			lock.release();

			using ProblemSettings = typename Selectors::ProblemTypeSelector<ProblemID>::template ProblemSettings<prec>;				// Type of the Problem Settings
			const ProblemSettings ProblemSet = *dynamic_cast<const ProblemSettings*>(&_SimManagerSettings.getProblemSettings());

			using ProblemType = typename Selectors::ProblemTypeSelector<ProblemID>::template ProblemType_Select<prec, AnisotropyID>;
			static_assert(std::is_same<ProblemSettings, typename ProblemType::ProblemSettings >::value, "Not the correct Settings for the Problem");

			auto Prob = ProblemType{ ProblemSet, Particle, ParticleInit };

			SolvBuilder(std::move(Prob),std::move(Particle), std::move(ParticleInit));
		};

		template<Settings::IProblem ProblemID, Properties::IAnisotropy AnisotropyID,
			typename SimulationParameters, typename SolverBuilder>
			std::enable_if_t<ProblemID == Settings::IProblem::Problem_BrownAndNeel> buildMagneticProblem(SimulationParameters &SimParams, SolverBuilder&& SolvBuilder)
		{
			std::unique_lock<std::mutex> lock(_ManagerMutex);
			auto Particle = SimParams.getNewParticleProperties();
			auto ParticleInit = SimParams.getParticleSimulationInitialization();
			lock.unlock();
			lock.release();

			using ProblemSettings = typename Selectors::ProblemTypeSelector<ProblemID>::template ProblemSettings<prec>;				// Type of the Problem Settings
			const ProblemSettings ProblemSet = *dynamic_cast<const ProblemSettings*>(&_SimManagerSettings.getProblemSettings());

			if (ProblemSet.getUseSimpleModel())
			{
				using ProblemType = typename Selectors::ProblemTypeSelector<ProblemID>::template ProblemType_Select<prec, AnisotropyID, true>;
				static_assert(std::is_same<ProblemSettings, typename ProblemType::ProblemSettings >::value, "Not the correct Settings for the Problem");
				auto Prob = ProblemType{ ProblemSet, Particle, ParticleInit };
				SolvBuilder(std::move(Prob), std::move(Particle), std::move(ParticleInit));
			}
			else
			{
				using ProblemType = typename Selectors::ProblemTypeSelector<ProblemID>::template ProblemType_Select<prec, AnisotropyID, false>;
				static_assert(std::is_same<ProblemSettings, typename ProblemType::ProblemSettings >::value, "Not the correct Settings for the Problem");
				auto Prob = ProblemType{ ProblemSet, Particle, ParticleInit };
				SolvBuilder(std::move(Prob), std::move(Particle), std::move(ParticleInit));
			}
		};

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Build the Problem </summary>
		///
		/// <typeparam name="FieldID">  	Identifier of the external field type. </typeparam>
		/// <typeparam name="SolverID"> 	Identifier of the solver type. </typeparam>
		/// <typeparam name="ProblemID">	Identifier of the Problem type. </typeparam>
		///-------------------------------------------------------------------------------------------------
		template <Properties::IField FieldID, Settings::ISolver SolverID, Settings::IProblem ProblemID>
		std::enable_if_t<ProblemID == Settings::IProblem::Problem_BrownAndNeel 
			|| ProblemID == Settings::IProblem::Problem_BrownAndNeelEulerSpherical
			|| ProblemID == Settings::IProblem::Problem_Neel 
			|| ProblemID == Settings::IProblem::Problem_NeelSpherical>
			buildProblemType()
		{
			//TODO: Change the enable if!

			//Need to find Anisotropy!
			using SimulationParameters = typename Selectors::ProblemTypeSelector<ProblemID>::template SimulationParameters<prec>;	// Type of the Simulation Paramters
			using Provider = typename Selectors::ProblemTypeSelector<ProblemID>::template NecessaryProvider<prec>;					// Provider for Simulation Parameters 
			
			auto createParams = [this]() -> SimulationParameters {
				std::lock_guard<std::mutex> lck{ this->_TaskCreationMutex};
				return (dynamic_cast<Provider*>(&_SimManagerSettings.getProvider())->getProvidedObject()); };

			SimulationParameters SimParams= createParams();

			auto buildSolver = [&](auto problem, auto properties, auto init) {
				std::tuple<std::decay_t<decltype(properties)>, std::decay_t<decltype(init)>> parameters{ properties, init };
				buildSolverType<FieldID, SolverID>(std::move(problem), std::move(parameters));
			};

			//Select Problem depending on Anisotropy
			const auto& Aniso = SimParams.getParticleProperties().getMagneticProperties().getTypeOfAnisotropy();
			switch (Aniso)
			{
				case Properties::IAnisotropy::Anisotropy_undefined:
				{
					Logger::Log("Simulation Manager: Ansiotropy not defined\n");
					break;
				}
				case Properties::IAnisotropy::Anisotropy_uniaxial:
				{
					buildMagneticProblem<ProblemID, Properties::IAnisotropy::Anisotropy_uniaxial>(SimParams, buildSolver);
					break;
				}
				case Properties::IAnisotropy::Anisotropy_cubic:
				{
					buildMagneticProblem<ProblemID, Properties::IAnisotropy::Anisotropy_cubic>(SimParams, buildSolver);
					break;
				}
				default:
				{
					Logger::Log("Simulation Manager: Ansiotropy switch default; Not implemented\n");
					break;
				}
			}

		}

		template<Properties::IField FieldID, Settings::ISolver SolverID, typename Problem, typename Parameters>
		std::enable_if_t<Selectors::SolverSelector<SolverID>::UsesDoubleNoiseMatrix::value> buildSolverType(const Problem& prob, const Parameters &params)
		{
			RuntimeDoubleNoiseMatrixSelection<FieldID, SolverID>(prob,params);
		}

		template<Properties::IField FieldID, Settings::ISolver SolverID, typename Problem, typename Parameters>
		std::enable_if_t<!Selectors::SolverSelector<SolverID>::UsesDoubleNoiseMatrix::value> buildSolverType(const Problem& prob, const Parameters &params)
		{
			using Solver = typename Selectors::SolverSelector<SolverID>::template SolverType<Problem>;
			buildFieldType<FieldID, Solver>(prob, params);
		}

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Selects the DoubleNoiseMatrix for stronger Solvers  </summary>
		///
		/// <typeparam name="FieldID">	Identifier of the external field type. </typeparam>
		/// <param name="prob">	[in] The Build Problem. </param>
		///-------------------------------------------------------------------------------------------------

#define BUILDNOISEMATRIX(Value) \
case Value: \
{buildFieldType<FieldID, typename Selectors::SolverSelector<SolverID>::template SolverType<Problem,Value>>(prob,params); break;}
		template <Properties::IField FieldID, Settings::ISolver SolverID, typename Problem, typename Parameters>
		void RuntimeDoubleNoiseMatrixSelection(const Problem& prob, const Parameters& params)
		{
			switch (_SimManagerSettings.getSolverSettings().getDoubleNoiseApprox())
			{
				BUILDNOISEMATRIX(-1);
				BUILDNOISEMATRIX(0);
				BUILDNOISEMATRIX(1);
				BUILDNOISEMATRIX(2);
				BUILDNOISEMATRIX(3);
				BUILDNOISEMATRIX(4);
				BUILDNOISEMATRIX(5);
				BUILDNOISEMATRIX(6);
				BUILDNOISEMATRIX(7);
				BUILDNOISEMATRIX(8);
				BUILDNOISEMATRIX(9);
				BUILDNOISEMATRIX(10);
			default:
			{
				Logger::Log("Simulation Manager: Level of DoubleNoise Approximation is not supported!\n");
				throw std::runtime_error{ "Simulation Manager : Value of DoubleNoise Approximation is not supported!\n" };
			}
			}
		};

#undef BUILDNOISEMATRIX

		template <Properties::IField FieldID, typename Solver, typename Problem, typename Parameters>
		void buildFieldType(const Problem& prob, const Parameters& params)
		{
			using Field = typename Selectors::FieldSelector<FieldID>::template FieldType<prec>;
			using Simulator = SingleParticleSimulator<Solver, Field>;
			typename Simulator::ProblemParameters probparameters{std::get<0>(params),std::get<1>(params)};
			createSimulationTask<Simulator>(prob, probparameters);
		};

		///* End Runtime Selectors;
		///*****************************************************************************************************************************************************/

	protected:
		SimulationManager() = default;
	public:
		SimulationManager(const Parameters &SimManSet)
			: _SimManagerSettings(SimManSet), _ThreadManager(SimManSet.getSimulationSettings().getNumberOfSimulators())
		{};

		void StartSimulationManager()
		{
				if (_SimManagerSettings.getSimulationSettings().getNumberOfSimulators() <= 1)
				{
					auto counter{ _SimManagerSettings.getSimulationSettings().getNumberOfSimulations() };
					while (counter--)
					{
						RuntimeFieldSelector();
					}
				}
				else
				{
					const std::size_t TasksToAdd = _SimManagerSettings.getSimulationSettings().getNumberOfSimulations();
					for (std::size_t i = 0; i < TasksToAdd; i++)
					{
						// TODO: Use Factory classes instead of template functions to create the problem
						// Contra: Would need a thread-local abstraction to hold the variable for the problem
						// Have to think about it!
						// Maybe create a runable object instead (operator() overload in class)
						// (Seems to be a good way! may try it later)
						// Alternative: Use command pattern here?
						_ThreadManager.AddTask([this]() { RuntimeFieldSelector(); });
					}

				}
		};

		// Blocks the current thread until the manager and all simulation finished!
		void waitUntilFinsihed()
		{
			{
				std::unique_lock<std::mutex> lck(_ManagerMutex);
				_ManagerConditionVariable.wait(lck, [this]() {return (this->_ResultManager != nullptr); });
			}
			_ResultManager->waitUntilFinished(_SimManagerSettings.getSimulationSettings().getNumberOfSimulations());
		};

		//Returns wether the Simulation Manager has finished
		bool isFinished()
		{
			if (_ResultManager == nullptr)
				return false;
			return _ResultManager->isFinished();
		};

		//Aborts the simulation
		void abort()
		{
			Logger::Log("Simulation Manager: Aborting Simulation!\n");
			_earlyAbort = true;
		};
	};
	template<typename prec>
	prec SimulationManager<prec>::ProgressModifier = { 0.0 }; 
	template<typename prec>
	prec SimulationManager<prec>::ProgressFactor = { 1.0 }; 
	template<typename prec>
	std::atomic<std::size_t> SimulationManager<prec>::ProgressCache = { 0 }; //Generetas missing ; <end of parse> error without the equal sign
    template<typename prec>
    const bool SimulationManager<prec>::ishpcjob = { std::getenv("CCP_JOBID") ? true : false };
};




#endif	// INC_SimulationManager_H
// end of SimulationManager.h
///---------------------------------------------------------------------------------------------------
