///-------------------------------------------------------------------------------------------------
// file:	SDEFramework\Solver\Implicit_Midpoint_GSL.inl
//
// summary:	Implicit midpoint solver
///-------------------------------------------------------------------------------------------------
#pragma once

#include <iostream>
#include <cmath>

#include "Implicit_Midpoint_GSL.h"

#include "../Basic_Library/basics/BasicIncludes.h"

//#define SOLVER_TIMING 0

#ifdef SOLVER_TIMING
#include "../Basic_Library/basics/Timer.h"
#endif

#include <Eigen/LU>

namespace SDE_Framework
{
	template<typename problem, typename nfield>
	Implicit_Midpoint_GSL<problem, nfield>::Implicit_Midpoint_GSL(const Settings& SolverSet, const Problem &prob, Precision tstep)
		: GeneralSDESolver<Implicit_Midpoint_GSL<problem, nfield>, problem, nfield>(prob, std::move(tstep)),
		MaxIteration(SolverSet.getMaxIteration()), AccuracyGoal(SolverSet.getAccuracyGoal()), mSolver(SolverSet.getAccuracyGoal(), SolverSet.getAccuracyGoal(),MaxIteration, Problem::Dimension::NumberOfDependentVariables, SolverSet.getImplicitGSLSolverType())
	{
		if (AccuracyGoal <= std::numeric_limits<Precision>::epsilon())
		{
			std::cout << "WARNING: Set AccuracyGoal may not be reachable! Goal: " << AccuracyGoal << " Epsilon: " << std::numeric_limits<Precision>::epsilon() << "\n";
		}

	};

	template<typename problem, typename nfield>
	template<typename IndependentVectorFunctor>
	auto Implicit_Midpoint_GSL<problem, nfield>::getResultNextFixedTimestep(const Precision &time, const DependentVectorType &yi, const IndependentVectorFunctor &xifunc) //-> ResultType
	{
		//1. Step: Calculate Guess

		const auto dt = this->m_timestep;
		const auto dW = this->m_dWgen.getField();

		const auto xi = xifunc(time);
		const auto a_guess = (this->m_problem).getDeterministicVector(yi, xi);
		const auto b_drift = (this->m_problem).getDrift(yi);
		const auto b_guess = (this->m_problem).getStochasticMatrix(yi);
		DependentVectorType yj{ (yi + (a_guess-b_drift)*dt + b_guess*dW).eval() }; //Initial Guess! First Step! y_i+1; Also storage for result!
		(this->m_problem).afterStepCheck(yj);			  //Check and correct step!

		//Ignore the guess!
		//DependentVectorType yj{ yi };

		//2. Step: Start Newton-Raphson Algorithm
		const auto xj = xifunc(time + 0.5*dt).eval();
		/*std::cout << "xj: " << xj.transpose() << "\n";*/

		auto f_functor = [&](const auto &yval) -> DependentVectorType
		{
			//std::cout << "yval: " << yval.transpose() << "\n";
			//std::cout << "xj: " << xj.transpose() << "\n";
			//std::cout << "dt: " << dt << "\n";
			//std::cout << "dW: " << dW.transpose() << "\n";
			const auto a = (this->m_problem).getDeterministicVector(yval, xj);
			//std::cout << "a: " << a.transpose() << "\n";
			const auto b = (this->m_problem).getStochasticMatrix(yval);
			//std::cout << "b: " << b << "\n";
			DependentVectorType res{ (-a*dt - b*dW).eval() };
			return res;
		};
		auto df_functor = [&](const auto &yval) -> typename Problem::Traits::JacobiMatrixType
		{
			const auto Jac_a = (this->m_problem).getJacobiDeterministic(yval, xj, dt);
			const auto Jac_b = (this->m_problem).getJacobiStochastic(dW);
			auto S_Jacobi{ (Problem::Traits::JacobiMatrixType::Identity() - 0.5*dt*Jac_a - 0.5*Jac_b).eval() };
			return S_Jacobi;
		};

#ifdef SOLVER_TIMING
		Timer<std::chrono::high_resolution_clock, std::chrono::nanoseconds> _Timer;
		_Timer.start();
#endif
		auto result = mSolver.getResult(std::move(f_functor), std::move(df_functor), yj);
#ifdef SOLVER_TIMING
		const auto watch = _Timer.stop();
		const auto numberofiter = (MaxIteration - Iter + 1);
		std::cout << "Finished Newton-Raphson after " << std::to_string(watch*_Timer.unitFactor()) + " s (" << std::to_string(watch / (numberofiter)) << " ns/iteration) Iterations:" << (numberofiter) << std::endl;
#endif

		return result;
	};
};

