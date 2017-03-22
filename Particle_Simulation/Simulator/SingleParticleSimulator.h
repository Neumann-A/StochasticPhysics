///---------------------------------------------------------------------------------------------------
// file:		SingleParticleSimulator.h
//
// summary: 	Declares the single particle simulator v 2 class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 26.06.2016
#pragma once
#ifndef INC_SingleParticleSimulator_H
#define INC_SingleParticleSimulator_H
///---------------------------------------------------------------------------------------------------

#include <chrono>
#include <memory>
#include <type_traits>

#include "ISingleParticleSimulator.h" 
// We can use the interface / ABC approach here since the simulator is only called a few times to start and get the result of the simulation
// As this the pure virtual call to the interface class will not cost alot of performance !!! That is very nice

#include "Results/SingleSimulationResult.h"
#include "Results/MeanSimulationResult.h"

#include "../Basic_Library/basics/Timer.h"

template<typename T>
class SimulatorTraits;

//Task of this class; Simulate a particle with a given problem and field;
// Needs a Solver and a actual problem

template <typename solver, typename extfield>
class SingleParticleSimulator
{
public:
	using Solver = solver;
	using Field = extfield;
	using ThisClass = SingleParticleSimulator<Solver, Field>;

	using Traits = SimulatorTraits<ThisClass>;

	using Precision = typename Traits::Precision;

	using Problem = typename Traits::Problem;
				
	using FieldVectorList = typename Traits::FieldVectorList;
	using ResultType = typename Traits::ResultType;
	using StepList = typename Traits::StepList;
	
private:
	using StepResult = typename Traits::StepResult;
	using SingleSimulationResult = typename Traits::SingleResultType;
	using MeanSimulationResult = typename Traits::MeanResultType;

	ResultType	_resvec;			//Vector with the results
	Problem		_problem;			//SDE describing the problem;
	Solver		_solver;			//Used solver
	Field		_field;				//Used external field
	const Precision _timestep;		//Used Timestep
	
	const uint64_t _SID;			//SimulatorID
	uint64_t _NumberOfSteps{ 0 };
	uint64_t _OverSampling{ 0 };

	Timer<std::chrono::high_resolution_clock, std::chrono::nanoseconds> _Timer;

	static std::atomic<uint64_t> _NumberOfActiveSimulators;
	static std::atomic<uint64_t> _NumberOfRunSimulation;
	static std::atomic<bool>	 _FieldandTimeCached;
	static std::mutex			 _FieldandTimeMutex;
	static FieldVectorList		 _Fields;
	static StepList				 _Times;

	void initSimulation(const uint64_t &NumberOfSteps, const uint64_t &OverSampling = 1)
	{
		this->_NumberOfSteps = NumberOfSteps;
		this->_OverSampling = OverSampling;

		if (NumberOfSteps % OverSampling != 0)
		{
			Log("OverSampling is not divider of NumberOfSteps. Last point in results will be wrong");
		};

		//Starting Clock
		_Timer.start();

		//Allocating Memory for the result
		this->_resvec.resize(static_cast<std::size_t>(floor(NumberOfSteps / OverSampling)), StepResult::Zero());

		//Randomize starting Point
		auto it = this->_resvec.begin();
		*it = _problem.getStart();
	};

	void startSimulation()
	{
		Log("Starting a new simulation");
		StepResult tmp{ StepResult::Zero() }; // Temp Result Storage (for oversampling)

		StepResult yi{ (*this->_resvec.begin()) };				// Last Result Storage
		size_t counter{ 0 };
		for (auto it = ++(this->_resvec.begin()); it != this->_resvec.end(); ++it)
		{
			for (size_t l = _OverSampling; l--;)
			{
				yi = this->_solver.getResultNextFixedTimestep(yi, _field.getField(_timestep*++counter));
				_problem.afterStepCheck(yi);
				tmp += yi;
			};
			tmp /= static_cast<double>(_OverSampling);
			_problem.afterStepCheck(tmp);

			std::swap(*it, tmp);
		};
	};

	void stopSimulation()
	{
		//Stopping Clock
		auto time = _Timer.stop();

		Log("Finished Simulation after " + std::to_string(time*_Timer.unitFactor()) + " s");
	};

	void Log(std::string s1)
	{
		std::stringstream msg;
		msg << "Simulator " << _SID << ": " << s1;
		Logger::Log(msg.str());
	};

public:
	ALLOW_DEFAULT_MOVE_AND_ASSIGN(SingleParticleSimulator)
	
	explicit SingleParticleSimulator(const Problem &problem, const Field &field, const Precision &timestep) :
		_problem(problem), // Copy the Problem
		_solver(_problem, timestep), //Link problem with Solver
		_field(field),
		_timestep(timestep),
		_SID(++(SingleParticleSimulator::_NumberOfRunSimulation))

	{
		++(SingleParticleSimulator::_NumberOfActiveSimulators);
	};
	~SingleParticleSimulator()
	{
		--(SingleParticleSimulator::_NumberOfActiveSimulators);
	};

	bool doSimulation(const std::size_t &NumberOfSteps, const std::size_t &OverSampling)
	{
		this->initSimulation(NumberOfSteps, OverSampling);
		this->startSimulation();
		this->stopSimulation();
		return true;
	};

	SingleSimulationResult getSimulationResult()
	{
		{
			std::lock_guard<std::mutex> lg(_FieldandTimeMutex);
			if (!_FieldandTimeCached.exchange(true))
			{
				calculateTimeandField(_NumberOfSteps,_OverSampling);
			}
		}

		SingleSimulationResult result{ _problem._ParParams, std::move(_resvec), _problem.getWeighting(), ThisClass::_Times, ThisClass::_Fields };
		
		return result;
	}

	void calculateTimeandField(const std::size_t &NumberOfSteps, const std::size_t &OverSampling)
	{
		//FieldVectorList Fields;
		//StepList Times;

		//Allocating Memory for the result
		const std::size_t points{ static_cast<std::size_t>(floor(NumberOfSteps / OverSampling)) };
		
		ThisClass::_Fields.resize(points, Problem::IndependentVectorType::Zero());
		ThisClass::_Times.resize(points);

		typename Traits::FieldVector tmp{ Problem::IndependentVectorType::Zero() }; // mean vector
				
		Precision start{ 0 };
		Precision end{ 0 };
		const Precision timeshift { _timestep*(static_cast<Precision>(OverSampling))*0.5 } ;

		auto timeit = ThisClass::_Times.begin();
		auto fieldit = ThisClass::_Fields.begin();
		*timeit = start;						//Startpunkt ist t = 0
		*fieldit = _field.getField(start);	//Feld bei t = 0;

		++timeit; ++fieldit;

		for (std::size_t counter{ 0 }; fieldit != ThisClass::_Fields.end() || timeit != ThisClass::_Times.end(); ++fieldit, ++timeit)
		{
			start = end + _timestep; //Neue Startzeit ist die alte Endzeit + ein zeitschritt
			for (std::size_t l = OverSampling; l--;)
			{
				tmp += _field.getField(_timestep*++counter);
			}
			end = _timestep*counter + start; //Endzeit (+ start gehört logischerweise in die erste Zeile aber der compiler kann a*b+c mittels cpu befehl besser optimieren!)
			tmp /= static_cast<Precision>(OverSampling);
			std::swap(*fieldit, tmp);
			*timeit = (end - start) - timeshift;
		}

	}

};

template <typename solver, typename field>
std::atomic<uint64_t> SingleParticleSimulator<solver, field>::_NumberOfActiveSimulators = { 0 };

template <typename solver, typename field>
std::atomic<uint64_t> SingleParticleSimulator<solver, field>::_NumberOfRunSimulation = { 0 };

template <typename solver, typename field>
std::atomic<bool> SingleParticleSimulator<solver, field>::_FieldandTimeCached = { false };

template <typename Solver, typename Field>
typename SingleParticleSimulator<Solver, Field>::FieldVectorList SingleParticleSimulator<Solver, Field>::_Fields = {};

template <typename Solver, typename Field>
typename SingleParticleSimulator<Solver, Field>::StepList SingleParticleSimulator<Solver, Field>::_Times = {};

template <typename Solver, typename Field>
std::mutex	SingleParticleSimulator<Solver, Field>::_FieldandTimeMutex = {};




template <typename solver, typename field>
class SimulatorTraits<SingleParticleSimulator<solver,field>>
{
public:
	using SimulatorType = SingleParticleSimulator<solver, field>;
	using Solver = solver;
	using Field = field;
	using Problem = typename Solver::Problem;

	static_assert(std::is_same<typename Field::Precision, typename Solver::Precision>::value, "Cannot mix classes with different floating point precisions!");

	using Precision = typename Solver::Precision;

	using FieldProperties = typename Field::FieldProperties;
	using FieldVector = typename Field::FieldVector;
	using FieldAlloc = typename Field::Traits::FieldVectorStdAllocator;
	using FieldVectorList = std::vector<FieldVector, FieldAlloc>;

	using StepList = std::vector<Precision>;
	using StepResult = typename Solver::ResultType;
	using StepResultAllocator = typename Solver::ResultTypeAllocator;

	using ResultType = std::vector<StepResult, StepResultAllocator>;

	using SingleResultType = Results::SingleSimulationResult<SimulatorType>;
	using MeanResultType = Results::MeanSimulationResult<SimulatorType>;
};


#endif	// INC_SingleParticleSimulator_H
// end of SingleParticleSimulator.h
///---------------------------------------------------------------------------------------------------
