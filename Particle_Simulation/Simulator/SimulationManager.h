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

#include "basics/BasicMacros.h"
#include "basics/Logger.h"
#include "basics/ThreadManager.h"

#include "ConfigFile_Archive/ConfigFile_Archive.h"
//Simulation Includes
//#include "ISingleParticleSimulator.h"
#include "SingleParticleSimulator.h"

#include "Results/ResultManagerFactory.h"
#include "Settings/SystemMatrix_SimulationManagerSettings_Factory.h"

// Anisotropy Includes
#include "Problems/Anisotropy/UniaxialAnisotropy.h"

//Field Includes
#include "Fields/SinusoidalField.h"
#include "Fields/LissajousField.h"

//Paramter Includes
#include "Settings/SimulationManagerSettings.h"

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
		//TODO: Refactor this class because it is doing to much. It is starting every simulation and handling/saving the results; Maybe move that part out of the class.
	public:
		using StartInputArchive = typename Archives::ConfigFile_InputArchive;			// Defines where to read the configs from
		using StartOutputArchive = typename Archives::ConfigFile_OutputArchive;			// Defines where to write the configs to
		
		using ResultOutputArchive = typename Archives::IOutputArchive;

		//TODO:: Create Typelist for all possible OutputArchives
		using PossibleConcreteResultArchives = std::false_type;

		using Parameters = Settings::SimulationManagerSettings<prec>;

		static prec ProgressModifier;
		static prec ProgressFactor;
		static std::atomic<std::size_t> ProgressCache; // In Percantage: 0 - 100 

	private:
		using ThisClass = SimulationManager<prec>;
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

		template <typename Simulator, typename Problem>
		void singleSimulationTask(Problem prob, typename Simulator::Field field, prec timestep)
		{
			if (_earlyAbort)
			{
				return;
			}
			else
			{
				++_NumberOfStartedSimulations;
				//prob, Field{ _SimManagerSettings.getFieldProperties() }, _SimManagerSettings.getSimulationSettings().getTimestep()
				Simulator Sim { std::move(prob), std::move(field), std::move(timestep) };

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
						Logger::Log("Simulation Manager: Simulation finished!");
					}
					return;
				}
			}

		};

		template <typename Simulator>
		void quickabortfinish(Simulator&&)
		{
			finalizeSimulation();
			Logger::Log("Simulation Manager: Quick Abort finished!");
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

#ifdef _MSC_VER
			//HACK: Fast hack to get the progress of the job in the HPC job manager. Should be replaced with something more sophistatced.
			//	    Spams a little in std::cerr due to the multithreaded nature of the programm.  
			const std::size_t test = std::round((static_cast<double>(_NumberOfFinishedSimulations) / static_cast<double>(_SimManagerSettings.getSimulationSettings().getNumberOfSimulations()))*100.0*ProgressFactor+ProgressModifier*100.0);
			auto tmp = ProgressCache.load();
			
			if (ProgressCache.compare_exchange_weak(tmp,test)) //To avoid spamming the system from a lot of threads!
			{
				ProgressCache.store(test); //Store the new value in the cache
				std::string str{ "Job modify %CCP_JOBID% /progress:" };
				str += std::to_string(test);
				std::system(str.c_str());
			}
#endif
		};

		void finalizeSimulation()
		{
			Logger::Log("Simulation Manager: Finsihed results of %d!", _NumberOfFinishedSimulations.load());
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
		void createSimulationTask(const typename Simulator::Problem &prob)
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
			this->singleSimulationTask<Simulator>(prob, std::move(extfield), std::move(dt));
		};

	
		///*****************************************************************************************************************************************************/
		///* BEGIN Runtime Selectors; The Selectors are necessary to convert runtime enums into compile time constants. We need to have the concrete compile  
		///*						  time types so that most of the functions can be inlined. We do not want to have virtual functions calls in our simulation  
		///*						  steps. The price we pay are this very ugly switch statements. */  

		/// <summary>	Runtime field selector. Selects the external appliad Field</summary>
		void RuntimeFieldSelector()
		{
			switch (_SimManagerSettings.getFieldProperties().getTypeOfField())
			{
			case Properties::IField::Field_undefined:
				Logger::Log("Simulation Manager: Field is not defined!");
				break;
			case Properties::IField::Field_Sinusoidal:
			{
				RuntimeSolverSelection<Properties::IField::Field_Sinusoidal>();
				break;
			}
			case Properties::IField::Field_Lissajous:
			{
				RuntimeSolverSelection<Properties::IField::Field_Lissajous>();
				break;
			}
			default:
				Logger::Log("Simulation Manager: Field is not defined!");
				break;
			}
		}

		///-------------------------------------------------------------------------------------------------
		/// <summary> Selects the correct Solver </summary>
		///
		/// <typeparam name="FieldID">	Identifier of the external Field Type. </typeparam>
		///-------------------------------------------------------------------------------------------------
		template <Properties::IField FieldID>
		void RuntimeSolverSelection()
		{
			switch (_SimManagerSettings.getSolverSettings().getTypeOfSolver())
			{
			case Settings::ISolver::Solver_undefined:
				Logger::Log("Simulation Manager: Solver not defined");
				break;
			case Settings::ISolver::Solver_EulerMaruyama:
				RuntimeProblemSelector<FieldID, Settings::ISolver::Solver_EulerMaruyama>();
				break;
			case Settings::ISolver::Solver_ExplicitStrong1_0:
				RuntimeProblemSelector<FieldID, Settings::ISolver::Solver_ExplicitStrong1_0>();
				break;
			default:
				Logger::Log("Simulation Manager: Solver not defined");
				break;
			}
		}

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Selects the correct Problem at runtime </summary>
		///
		/// <typeparam name="FieldID"> 	Identifier of the external field type. </typeparam>
		/// <typeparam name="SolverID">	Identifier of the solver type. </typeparam>
		///-------------------------------------------------------------------------------------------------
		template <Properties::IField FieldID, Settings::ISolver SolverID>
		void RuntimeProblemSelector()
		{
			// First identify the Problem
			switch (_SimManagerSettings.getProblemSettings().getProblemType())
			{
			case Settings::IProblem::Problem_undefined:
				Logger::Log("Simulation Manager: Problem not defined");
				break;
			case Settings::IProblem::Problem_BrownAndNeel:
				// In the case find the necessary anisotropy type
				buildProblemType<FieldID,SolverID,Settings::IProblem::Problem_BrownAndNeel>();
				break;
			case Settings::IProblem::Problem_Neel:
				// In the case find the necessary anisotropy type
				buildProblemType<FieldID,SolverID,Settings::IProblem::Problem_Neel>();
				break;
			default:
				Logger::Log("Simulation Manager: Problem not defined");
				break;
			}
		}

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Builds the Type of a Magnetic Problem. </summary>
		///
		/// <typeparam name="ProblemID">   	Identifier of the Problem type </typeparam>
		/// <typeparam name="AnisotropyID">	Identifier of the Anisotropy type </typeparam>
		/// <param name="SimParams">	[in,out] PAramters of the simulation. </param>
		///
		/// <returns>	The constructed problem </returns>
		///-------------------------------------------------------------------------------------------------
		template <Settings::IProblem ProblemID, Properties::IAnisotropy AnisotropyID, 
			typename ProblemType = typename Selectors::ProblemTypeSelector<ProblemID>::template ProblemType_Select<prec, AnisotropyID>,
		typename SimulationParameters = typename Selectors::ProblemTypeSelector<ProblemID>::template SimulationParameters<prec> >
			ProblemType buildMagneticProblem(SimulationParameters &SimParams)
		{
			static_assert((ProblemID == Settings::IProblem::Problem_BrownAndNeel || ProblemID == Settings::IProblem::Problem_Neel), "RuntimeAnisotropySelector called with invalid ProblemID!");

			std::unique_lock<std::mutex> lock(_ManagerMutex);

			using ProblemSettings = typename Selectors::ProblemTypeSelector<ProblemID>::template ProblemSettings<prec>;				// Type of the Problem Settings
			const ProblemSettings ProblemSet = *dynamic_cast<const ProblemSettings*>(&_SimManagerSettings.getProblemSettings());

			static_assert(std::is_same<ProblemSettings, typename ProblemType::ProblemSettings >::value, "Not the correct Settings for the Problem");
			return ProblemType{ ProblemSet, SimParams.getNewParticleProperties(), SimParams.getParticleSimulationInitialization() };
		};

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Build the Problem </summary>
		///
		/// <typeparam name="FieldID">  	Identifier of the external field type. </typeparam>
		/// <typeparam name="SolverID"> 	Identifier of the solver type. </typeparam>
		/// <typeparam name="ProblemID">	Identifier of the Problem type. </typeparam>
		///-------------------------------------------------------------------------------------------------
		template <Properties::IField FieldID, Settings::ISolver SolverID, Settings::IProblem ProblemID>
		std::enable_if_t<ProblemID == Settings::IProblem::Problem_BrownAndNeel || ProblemID == Settings::IProblem::Problem_Neel> buildProblemType()
		{
			//Need to find Anisotropy!

			using SimulationParameters = typename Selectors::ProblemTypeSelector<ProblemID>::template SimulationParameters<prec>;	// Type of the Simulation Paramters
			using Provider = typename Selectors::ProblemTypeSelector<ProblemID>::template NecessaryProvider<prec>;					// Provider for Simulation Parameters 

			SimulationParameters SimParams= (dynamic_cast<Provider*>(&_SimManagerSettings.getProvider())->getProvidedObject());

			//Select Problem depending on Anisotropy
			const auto& Aniso = SimParams.getParticleProperties().getMagneticProperties().getTypeOfAnisotropy();
			switch (Aniso)
			{
			case Properties::IAnisotropy::Anisotropy_undefined:
				Logger::Log("Simulation Manager: Ansiotropy not defined");
				break;
			case Properties::IAnisotropy::Anisotropy_uniaxial:
			{
				using ProblemType = typename Selectors::ProblemTypeSelector<ProblemID>::template ProblemType_Select<prec, Properties::IAnisotropy::Anisotropy_uniaxial>;
				ProblemType prob{ buildMagneticProblem<ProblemID, Properties::IAnisotropy::Anisotropy_uniaxial>(SimParams) };
				buildSolverType<FieldID, SolverID>(std::move(prob));										//Maybe install callback?
				break;
			}
			default:
				Logger::Log("Simulation Manager: Ansiotropy switch default; Not implemented");
				break;
			}

		}

		template<Properties::IField FieldID, Settings::ISolver SolverID, typename Problem>
		std::enable_if_t<Selectors::SolverSelector<SolverID>::UsesDoubleNoiseMatrix::value> buildSolverType(const Problem& prob)
		{
			RuntimeDoubleNoiseMatrixSelection<FieldID, SolverID>(prob);
		}

		template<Properties::IField FieldID, Settings::ISolver SolverID, typename Problem>
		std::enable_if_t<!Selectors::SolverSelector<SolverID>::UsesDoubleNoiseMatrix::value> buildSolverType(const Problem& prob)
		{
			using Solver = typename Selectors::SolverSelector<SolverID>::template SolverType<Problem>;
			buildFieldType<FieldID, Solver>(prob);
		}

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Selects the DoubleNoiseMatrix for stronger Solvers  </summary>
		///
		/// <typeparam name="FieldID">	Identifier of the external field type. </typeparam>
		/// <param name="prob">	[in] The Build Problem. </param>
		///-------------------------------------------------------------------------------------------------
		template <Properties::IField FieldID, Settings::ISolver SolverID, typename Problem>
		void RuntimeDoubleNoiseMatrixSelection(const Problem& prob)
		{
			//template<typename Problem, int order>
			//using Solver = typename Selectors::SolverSelector<SolverID>::template SolverType<Problem,order>;
	
			if ((_SimManagerSettings.getSolverSettings().getDoubleNoiseApprox() > 10) || (_SimManagerSettings.getSolverSettings().getDoubleNoiseApprox() < -1))
			{
				Logger::Log("Simulation Manager: DoubleNoiseMatrix with Approximation higher than 10 or lower than -1 are not supported! Current Level %d", _SimManagerSettings.getSolverSettings().getDoubleNoiseApprox());
				return;
			}
			// Das schreit nach nem Präprozessor Makro .....
			switch (_SimManagerSettings.getSolverSettings().getDoubleNoiseApprox())
			{
			case -1:
				buildFieldType<FieldID, typename Selectors::SolverSelector<SolverID>::template SolverType<Problem,-1>>(prob);
				break;
			case 0:
				buildFieldType<FieldID, typename Selectors::SolverSelector<SolverID>::template SolverType<Problem, 0>>(prob);
				break;
			case 1:
				buildFieldType<FieldID, typename Selectors::SolverSelector<SolverID>::template SolverType<Problem, 1>>(prob);
				break;
			case 2:
				buildFieldType<FieldID, typename Selectors::SolverSelector<SolverID>::template SolverType<Problem, 2>>(prob);
				break;
			case 3:
				buildFieldType<FieldID, typename Selectors::SolverSelector<SolverID>::template SolverType<Problem, 3>>(prob);
				break;
			case 4:
				buildFieldType<FieldID, typename Selectors::SolverSelector<SolverID>::template SolverType<Problem, 4>>(prob);
				break;
			case 5:
				buildFieldType<FieldID, typename Selectors::SolverSelector<SolverID>::template SolverType<Problem, 5>>(prob);
				break;
			case 6:
				buildFieldType<FieldID, typename Selectors::SolverSelector<SolverID>::template SolverType<Problem, 6>>(prob);
				break;
			case 7:
				buildFieldType<FieldID, typename Selectors::SolverSelector<SolverID>::template SolverType<Problem, 7>>(prob);
				break;
			case 8:
				buildFieldType<FieldID, typename Selectors::SolverSelector<SolverID>::template SolverType<Problem, 8>>(prob);
				break;
			case 9:
				buildFieldType<FieldID, typename Selectors::SolverSelector<SolverID>::template SolverType<Problem, 9>>(prob);
				break;
			case 10:
				buildFieldType<FieldID, typename Selectors::SolverSelector<SolverID>::template SolverType<Problem, 10>>(prob);
				break;
			default:
				Logger::Log("Simulation Manager: Level of Double Noise Approximation is not supported!");
			}
		};

		template <Properties::IField FieldID, typename Solver, typename Problem>
		void buildFieldType(const Problem& prob)
		{
			using Field = typename Selectors::FieldSelector<FieldID>::template FieldType<prec>;
			using Simulator = SingleParticleSimulator<Solver, Field>;
			createSimulationTask<Simulator>(prob);
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
					std::size_t TasksToAdd = _SimManagerSettings.getSimulationSettings().getNumberOfSimulations();
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

				};
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
			Logger::Log("Simulation Manager: Aborting Simulation!");
			_earlyAbort = true;
		};
	};
	template<typename prec>
	prec SimulationManager<prec>::ProgressModifier = { 0.0 }; 
	template<typename prec>
	prec SimulationManager<prec>::ProgressFactor = { 1.0 }; 
	template<typename prec>
	std::atomic<std::size_t> SimulationManager<prec>::ProgressCache = { 0.0 }; //Generetas missing ; <end of parse> error without the equal sign
};




#endif	// INC_SimulationManager_H
// end of SimulationManager.h
///---------------------------------------------------------------------------------------------------
