///---------------------------------------------------------------------------------------------------
// file:		SDEFramework\Solver\Implicit_Midpoint_GSL_Derivative_Free.h
//
// summary: 	Declares the implicit midpoint gsl derivative free class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 03.09.2017

#ifndef INC_Implicit_Midpoint_GSL_Derivative_Free_H
#define INC_Implicit_Midpoint_GSL_Derivative_Free_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <type_traits>
#include <limits>

#include "GeneralSDESolver.h"
#include "Settings/SolverSettings.h"

#include <MyCEL/math/GSL_Implicit_Solver_Derivative_Free.h>

namespace SDE_Framework::Solvers
{
	//Euler Maruyama uses Ito intepretation
	//converges with strong order 0.5 and weak order 1
	template<typename problem, typename nfield>
	class Implicit_Midpoint_GSL_Derivative_Free : public GeneralSDESolver<Implicit_Midpoint_GSL_Derivative_Free<problem, nfield>, problem, nfield>
	{
		template<bool IsIto>
		friend struct detail::FixedTimestepSelector;
	public:
		using Problem = problem;
		using Precision = typename Problem::Traits::Precision;

		using ResultType = typename Problem::Traits::DependentType;

		using NoiseField = nfield;
		using Settings = Settings::SolverSettings<Precision>;
	private:
		using IsIto = typename Problems::SDEProblem_Traits<problem>::IsIto;

		using DependentType = typename Problem::DependentType;
		using IndependentType = typename Problem::IndependentType;
		using DeterministicType = typename Problem::DeterministicType;
		using StochasticMatrixType = typename Problem::StochasticMatrixType;

		const std::size_t MaxIteration;
		const Precision   AccuracyGoal;

		GSL_Implicit_Solver_Derivative_Free<Precision> mSolver;
	public:

		Implicit_Midpoint_GSL_Derivative_Free(const Settings& SolverSet, Problem &prob, Precision tstep)
			: GeneralSDESolver<Implicit_Midpoint_GSL_Derivative_Free<problem, nfield>, problem, nfield>(prob, std::move(tstep)),
			MaxIteration(SolverSet.getMaxIteration()), AccuracyGoal(SolverSet.getAccuracyGoal()), mSolver(SolverSet.getAccuracyGoal(), SolverSet.getAccuracyGoal(), MaxIteration, Problem::Dimension::NumberOfDependentVariables, SolverSet.getImplicitGSL2SolverType())
		{
			if (AccuracyGoal <= std::numeric_limits<Precision>::epsilon())
			{
				std::cout << "WARNING: Set AccuracyGoal may not be reachable! Goal: " << AccuracyGoal << " Epsilon: " << std::numeric_limits<Precision>::epsilon() << "\n";
			}

		};

		template<typename IndependentFunctor>
		auto getResultNextFixedTimestep(const Precision &time, const DependentType &yi, const IndependentFunctor &xifunc)
		{
			//1. Step: Calculate Guess

			const auto dt = this->m_timestep;
			const auto dW = this->m_dWgen.getField();
			
			//Guess
			DependentType yj{ yi }; //Copy the value!
			this->m_problem.prepareCalculations(yj);
			//const auto xi = xifunc(time);
			//const auto aguess = (this->m_problem).getDeterministicVector(yj, xi);
			//const auto bguess = (this->m_problem).getStochasticMatrix(yj);
			//yj += aguess*dt + bguess*dW;
			//this->m_problem.finishCalculations(yj);
			//std::cout << "guess: " << yj.transpose() << '\n';
			const auto xj = xifunc(time + 0.5*dt).eval();

			auto f_functor = [&](const auto &yval) -> DependentType
			{
				DependentType res{ (yval+yi)*0.5 }; //Copy the value!
				this->m_problem.prepareCalculations(res);
				const auto a = (this->m_problem).getDeterministicVector(res, xj);
				const auto b = (this->m_problem).getStochasticMatrix(res);
				res += (a*dt + b*dW).eval(); //res needs to be a valid coordinate to be transformed back 
				this->m_problem.finishCalculations(res);
				res += 0.5*yi-1.5*yval;
				return res.eval();
			};

#ifdef SOLVER_TIMING
			Timer<std::chrono::high_resolution_clock, std::chrono::nanoseconds> _Timer;
			_Timer.start();
#endif
			auto result = mSolver.getResult(std::move(f_functor), yj);
			//std::cout << "Result: " << result.transpose() << " Result (Norm): " << result.norm() << '\n';
#ifdef SOLVER_TIMING
			const auto watch = _Timer.stop();
			const auto numberofiter = (MaxIteration - Iter + 1);
			std::cout << "Finished Implicit Solver after " << std::to_string(watch*_Timer.unitFactor()) + " s (" << std::to_string(watch / (numberofiter)) << " ns/iteration) Iterations:" << (numberofiter) << std::endl;
#endif

			return result;
		}; // -> ResultType;

	};

	namespace detail
	{
		template<typename problem, typename nfield>
		struct is_implicit_solver<Implicit_Midpoint_GSL_Derivative_Free<problem, nfield>> : std::true_type {};
	}

}

#endif	// INC_Implicit_Midpoint_GSL_Derivative_Free_H
// end of SDEFramework\Solver\Implicit_Midpoint_GSL_Derivative_Free.h
///---------------------------------------------------------------------------------------------------
