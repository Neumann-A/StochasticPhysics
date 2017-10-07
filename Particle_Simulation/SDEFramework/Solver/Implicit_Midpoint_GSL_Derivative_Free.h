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

#include "../Basic_Library/math/GSL_Implicit_Solver_Derivative_Free.h"

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
		typedef typename problem::Precision																			   Precision;
		typedef	problem																								   Problem;
		typedef typename problem::DependentType																   ResultType;

		using ResultTypeAllocator = typename Problem::Traits::DependentVectorStdAllocator;
		using NoiseField = nfield;

		using Settings = Settings::SolverSettings<Precision>;

	private:
		using IsIto = typename Problems::SDEProblem_Traits<problem>::IsIto;
		using IsExplicitSolver = typename  std::false_type;
		using IsImplicitSolver = typename  std::true_type;

		typedef typename problem::Dimension																			   Dimensions;
		typedef typename problem::DependentType																   DependentType;
		//typedef typename problem::IndependentType															   IndependentType;
		typedef typename problem::DeterministicType															   DeterministicType;
		typedef typename problem::StochasticMatrixType																   StochasticMatrixType;

		const std::size_t MaxIteration;
		const Precision   AccuracyGoal;

		GSL_Implicit_Solver_Derivative_Free<Precision> mSolver;
	public:

		Implicit_Midpoint_GSL_Derivative_Free(const Settings& SolverSet, Problem &prob, Precision tstep)
			: GeneralSDESolver<Implicit_Midpoint_GSL<problem, nfield>, problem, nfield>(prob, std::move(tstep)),
			MaxIteration(SolverSet.getMaxIteration()), AccuracyGoal(SolverSet.getAccuracyGoal()), mSolver(SolverSet.getAccuracyGoal(), SolverSet.getAccuracyGoal(), MaxIteration, Problem::Dimension::NumberOfDependentVariables, SolverSet.getImplicitGSL2SolverType())
		{
			if (AccuracyGoal <= std::numeric_limits<Precision>::epsilon())
			{
				std::cout << "WARNING: Set AccuracyGoal may not be reachable! Goal: " << AccuracyGoal << " Epsilon: " << std::numeric_limits<Precision>::epsilon() << "\n";
			}

		};

		template<typename IndependentFunctor>
		auto getResultNextFixedTimestep(const Precision&, const DependentType &yi, const IndependentFunctor &xifunc)
		{
			//1. Step: Calculate Guess

			const auto dt = this->m_timestep;
			const auto dW = this->m_dWgen.getField();

			const auto xi = xifunc(time);
			const auto a_guess = (this->m_problem).getDeterministicVector(yi, xi);
			const auto b_drift = (this->m_problem).getDrift(yi);
			const auto b_guess = (this->m_problem).getStochasticMatrix(yi);
			DependentType yj{ (yi + (a_guess - b_drift)*dt + b_guess*dW).eval() }; //Initial Guess! First Step! y_i+1; Also storage for result!
			(this->m_problem).finishCalculations(yj);			  //Check and correct step!

															  //Ignore the guess!
															  //DependentType yj{ yi };

															  //2. Step: Start Newton-Raphson Algorithm
			const auto xj = xifunc(time + 0.5*dt).eval();
			/*std::cout << "xj: " << xj.transpose() << "\n";*/

			auto f_functor = [&](const auto &yval) -> DependentType
			{
				//std::cout << "yval: " << yval.transpose() << "\n";
				//std::cout << "xj: " << xj.transpose() << "\n";
				//std::cout << "dt: " << dt << "\n";
				//std::cout << "dW: " << dW.transpose() << "\n";
				const auto a = (this->m_problem).getDeterministicVector(yval, xj);
				//std::cout << "a: " << a.transpose() << "\n";
				const auto b = (this->m_problem).getStochasticMatrix(yval);
				//std::cout << "b: " << b << "\n";
				DependentType res{ (-a*dt - b*dW).eval() };
				return res;
			};

#ifdef SOLVER_TIMING
			Timer<std::chrono::high_resolution_clock, std::chrono::nanoseconds> _Timer;
			_Timer.start();
#endif
			auto result = mSolver.getResult(std::move(f_functor), yj);

#ifdef SOLVER_TIMING
			const auto watch = _Timer.stop();
			const auto numberofiter = (MaxIteration - Iter + 1);
			std::cout << "Finished Implicit Solver after " << std::to_string(watch*_Timer.unitFactor()) + " s (" << std::to_string(watch / (numberofiter)) << " ns/iteration) Iterations:" << (numberofiter) << std::endl;
#endif

			return result;
		}; // -> ResultType;

	};

}

#endif	// INC_Implicit_Midpoint_GSL_Derivative_Free_H
// end of SDEFramework\Solver\Implicit_Midpoint_GSL_Derivative_Free.h
///---------------------------------------------------------------------------------------------------
